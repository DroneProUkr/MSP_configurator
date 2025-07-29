
#include "msp_connection.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <cstring>
#include <iomanip> // Required for std::hex, std::setw, std::setfill
#include <algorithm> // Required for std::min
#include <thread> // Required for std::this_thread::sleep_for
#include <chrono> // Required for std::chrono::milliseconds

MSPConnection::MSPConnection(const std::string& device) 
    : serial_fd(-1), connected(false), device_path(device) {
}

MSPConnection::~MSPConnection() {
    disconnect();
}

bool MSPConnection::connect() {
    if (connected) {
        std::cout << "DEBUG: Already connected" << std::endl;
        return true;
    }
    
    std::cout << "Connecting to drone at " << device_path << std::endl;
    
    serial_fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial device: " << strerror(errno) << std::endl;
        std::cerr << "DEBUG: open() failed with errno=" << errno << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: Serial device opened successfully, fd=" << serial_fd << std::endl;
    
    if (!configureSerial()) {
        std::cerr << "DEBUG: configureSerial failed" << std::endl;
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    
    std::cout << "DEBUG: Serial configuration successful" << std::endl;
    
    connected = true;
    std::cout << "Successfully connected to drone" << std::endl;
    return true;
}

void MSPConnection::disconnect() {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
    connected = false;
    std::cout << "Disconnected from drone" << std::endl;
}

bool MSPConnection::configureSerial() {
    struct termios tty;
    
    std::cout << "DEBUG: Configuring serial port..." << std::endl;
    
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting serial attributes: " << strerror(errno) << std::endl;
        std::cerr << "DEBUG: tcgetattr failed with errno=" << errno << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: Got current serial attributes" << std::endl;
    
    // Set baud rate
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    
    std::cout << "DEBUG: Set baud rate to 115200" << std::endl;
    
    // 8 bits, no parity, 1 stop bit
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    std::cout << "DEBUG: Set 8N1 configuration" << std::endl;
    
    // No hardware flow control
    tty.c_cflag &= ~CRTSCTS;
    
    // Enable receiver, ignore modem control lines
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Raw input mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output mode
    tty.c_oflag &= ~OPOST;
    
    // No input processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    
    // Set timeouts
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    std::cout << "DEBUG: Set raw mode and timeouts" << std::endl;
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes: " << strerror(errno) << std::endl;
        std::cerr << "DEBUG: tcsetattr failed with errno=" << errno << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: Serial attributes set successfully" << std::endl;
    
    // Flush any existing data
    tcflush(serial_fd, TCIOFLUSH);
    
    std::cout << "DEBUG: Flushed serial buffers" << std::endl;
    
    return true;
}

bool MSPConnection::sendMessage(const MSPMessage& msg) {
    if (!connected) {
        std::cerr << "DEBUG: sendMessage called but not connected" << std::endl;
        return false;
    }
    
    std::vector<uint8_t> buffer = MSPProtocol::encodeMessage(msg);
    
    std::cout << "DEBUG: Sending MSP message - cmd=" << (int)msg.cmd 
              << ", size=" << (int)msg.size 
              << ", buffer_size=" << buffer.size() << std::endl;
    
    // Debug: Print first few bytes of the message
    std::cout << "DEBUG: Message bytes: ";
    for (size_t i = 0; i < std::min(buffer.size(), size_t(10)); i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
    }
    if (buffer.size() > 10) std::cout << "...";
    std::cout << std::dec << std::endl;
    
    ssize_t bytes_written = write(serial_fd, buffer.data(), buffer.size());
    if (bytes_written != static_cast<ssize_t>(buffer.size())) {
        std::cerr << "DEBUG: Failed to write complete message - wrote " << bytes_written 
                  << " of " << buffer.size() << " bytes" << std::endl;
        std::cerr << "DEBUG: write() error: " << strerror(errno) << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: Successfully sent " << bytes_written << " bytes" << std::endl;
    return true;
}

bool MSPConnection::sendBytes(const uint8_t* data, size_t length) {
    if (!connected) {
        std::cout << "DEBUG: sendBytes called but not connected" << std::endl;
        return false;
    }
    
    ssize_t bytes_written = write(serial_fd, data, length);
    if (bytes_written != static_cast<ssize_t>(length)) {
        std::cout << "DEBUG: sendBytes failed - wrote " << bytes_written << " of " << length << " bytes" << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: sendBytes succeeded - wrote " << bytes_written << " bytes" << std::endl;
    return true;
}

int MSPConnection::readBytes(uint8_t* buffer, int max_bytes, int timeout_ms) {
    fd_set read_fds;
    struct timeval timeout;
    
    FD_ZERO(&read_fds);
    FD_SET(serial_fd, &read_fds);
    
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    std::cout << "DEBUG: readBytes: waiting for " << max_bytes << " bytes with timeout " << timeout_ms << "ms" << std::endl;
    
    int result = select(serial_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (result <= 0) {
        if (result == 0) {
            std::cout << "DEBUG: readBytes: select timeout" << std::endl;
        } else {
            std::cout << "DEBUG: readBytes: select error: " << strerror(errno) << std::endl;
        }
        return result; // Timeout or error
    }
    
    std::cout << "DEBUG: readBytes: data available, reading..." << std::endl;
    
    int bytes_read = read(serial_fd, buffer, max_bytes);
    
    if (bytes_read > 0) {
        std::cout << "DEBUG: readBytes: read " << bytes_read << " bytes" << std::endl;
    } else if (bytes_read == 0) {
        std::cout << "DEBUG: readBytes: read returned 0 (EOF)" << std::endl;
    } else {
        std::cout << "DEBUG: readBytes: read error: " << strerror(errno) << std::endl;
    }
    
    return bytes_read;
}

bool MSPConnection::receiveMessage(MSPMessage& msg, int timeout_ms) {
    if (!connected) {
        std::cerr << "DEBUG: receiveMessage called but not connected" << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: Starting to receive message with timeout=" << timeout_ms << "ms" << std::endl;
    
    std::vector<uint8_t> buffer;
    uint8_t byte;
    auto start_time = std::chrono::steady_clock::now();
    
    // Look for header - accept $M>, $M<, or $M!
    int header_pos = 0;
    const char* header = "$M>";
    
    std::cout << "DEBUG: Looking for MSP header: $M> or $M! or $M<" << std::endl;
    
    while (header_pos < 3) {
        int bytes_read = readBytes(&byte, 1, 100);
        if (bytes_read <= 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            if (elapsed.count() > timeout_ms) {
                std::cout << "DEBUG: Timeout while looking for header after " << elapsed.count() << "ms" << std::endl;
                return false;
            }
            continue;
        }
        
        std::cout << "DEBUG: Read byte: 0x" << std::hex << (int)byte << std::dec 
                  << " (char: '" << (char)byte << "')" << std::endl;
        
        if (byte == header[header_pos]) {
            buffer.push_back(byte);
            header_pos++;
            std::cout << "DEBUG: Header match at position " << header_pos << std::endl;
        } else if (header_pos == 2 && (byte == '!' || byte == '<')) {
            // Accept $M! or $M< as valid headers
            buffer.push_back(byte);
            header_pos++;
            std::cout << "DEBUG: Alternative header match at position " << header_pos << std::endl;
        } else {
            buffer.clear();
            header_pos = 0;
            if (byte == header[0]) {
                buffer.push_back(byte);
                header_pos = 1;
                std::cout << "DEBUG: Reset to position 1" << std::endl;
            } else {
                std::cout << "DEBUG: No header match, continuing search" << std::endl;
            }
        }
    }
    
    std::cout << "DEBUG: Found complete header, reading size and command" << std::endl;
    
    // Read size and command
    for (int i = 0; i < 2; i++) {
        int bytes_read = readBytes(&byte, 1, 100);
        if (bytes_read <= 0) {
            std::cout << "DEBUG: Failed to read size/command byte " << i << std::endl;
            return false;
        }
        buffer.push_back(byte);
        std::cout << "DEBUG: Size/command byte " << i << ": 0x" << std::hex << (int)byte << std::dec << std::endl;
    }
    
    uint8_t payload_size = buffer[3];
    std::cout << "DEBUG: Payload size: " << (int)payload_size << std::endl;
    
    // Read payload and checksum
    for (int i = 0; i < payload_size + 1; i++) {
        int bytes_read = readBytes(&byte, 1, 100);
        if (bytes_read <= 0) {
            std::cout << "DEBUG: Failed to read payload/checksum byte " << i << std::endl;
            return false;
        }
        buffer.push_back(byte);
        std::cout << "DEBUG: Payload/checksum byte " << i << ": 0x" << std::hex << (int)byte << std::dec << std::endl;
    }
    
    std::cout << "DEBUG: Complete message received, size: " << buffer.size() << std::endl;
    std::cout << "DEBUG: Message bytes: ";
    for (size_t i = 0; i < std::min(buffer.size(), size_t(15)); i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
    }
    if (buffer.size() > 15) std::cout << "...";
    std::cout << std::dec << std::endl;
    
    bool decode_result = MSPProtocol::decodeMessage(buffer, msg);
    std::cout << "DEBUG: Message decode result: " << (decode_result ? "SUCCESS" : "FAILED") << std::endl;
    
    return decode_result;
}

bool MSPConnection::sendAndReceive(const MSPMessage& request, MSPMessage& response, int timeout_ms) {
    std::cout << "DEBUG: sendAndReceive called with timeout=" << timeout_ms << "ms" << std::endl;
    
    if (!sendMessage(request)) {
        std::cout << "DEBUG: sendMessage failed" << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: sendMessage succeeded, now receiving response" << std::endl;
    
    bool receive_result = receiveMessage(response, timeout_ms);
    std::cout << "DEBUG: receiveMessage result: " << (receive_result ? "SUCCESS" : "FAILED") << std::endl;
    
    return receive_result;
}

int MSPConnection::testReadByte(int timeout_ms) {
    if (!connected) {
        std::cout << "DEBUG: testReadByte called but not connected" << std::endl;
        return -1;
    }
    
    uint8_t byte;
    return readBytes(&byte, 1, timeout_ms);
}

bool MSPConnection::testBaudRate(int baud_rate) {
    if (!connected) {
        std::cout << "DEBUG: testBaudRate called but not connected" << std::endl;
        return false;
    }
    
    std::cout << "DEBUG: Testing baud rate: " << baud_rate << std::endl;
    
    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cout << "DEBUG: Failed to get current attributes for baud rate test" << std::endl;
        return false;
    }
    
    // Set the new baud rate
    speed_t baud_constant;
    switch (baud_rate) {
        case 9600: baud_constant = B9600; break;
        case 19200: baud_constant = B19200; break;
        case 38400: baud_constant = B38400; break;
        case 57600: baud_constant = B57600; break;
        case 115200: baud_constant = B115200; break;
        case 230400: baud_constant = B230400; break;
        default:
            std::cout << "DEBUG: Unsupported baud rate: " << baud_rate << std::endl;
            return false;
    }
    
    cfsetispeed(&tty, baud_constant);
    cfsetospeed(&tty, baud_constant);
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cout << "DEBUG: Failed to set baud rate " << baud_rate << std::endl;
        return false;
    }
    
    // Flush buffers
    tcflush(serial_fd, TCIOFLUSH);
    
    // Wait a moment for the change to take effect
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Test if we can read any data
    uint8_t test_byte;
    int result = readBytes(&test_byte, 1, 500);
    
    if (result > 0) {
        std::cout << "DEBUG: SUCCESS! Found data at baud rate " << baud_rate << std::endl;
        return true;
    } else {
        std::cout << "DEBUG: No data at baud rate " << baud_rate << std::endl;
        return false;
    }
}

bool MSPConnection::readAvailableBytes(std::vector<uint8_t>& buffer, int timeout_ms) {
    if (!connected) {
        std::cout << "DEBUG: readAvailableBytes called but not connected" << std::endl;
        return false;
    }
    
    buffer.clear();
    
    auto start_time = std::chrono::steady_clock::now();
    uint8_t byte;
    bool got_newline = false;
    
    while (true) {
        int bytes_read = readBytes(&byte, 1, 50); // Use shorter timeout for individual byte reads
        if (bytes_read <= 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            if (elapsed.count() > timeout_ms) {
                break; // Timeout for overall read operation
            }
            // If we already have some data and got a newline, we're done
            if (!buffer.empty() && got_newline) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Short sleep to avoid busy waiting
            continue;
        }
        
        buffer.push_back(byte);
        
        // Track if we got a newline
        if (byte == '\n') {
            got_newline = true;
        }
        
        // If we get a '#' prompt after a newline, we're done
        if (got_newline && byte == '#') {
            break;
        }
        
        // Reset start_time to keep reading as long as new data arrives
        start_time = std::chrono::steady_clock::now(); 
    }
    
    return !buffer.empty();
}