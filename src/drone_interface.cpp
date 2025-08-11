#include "drone_interface.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include "msp_connection.h"

DroneInterface::DroneInterface(const std::string& device) 
    : device(device), serial_fd(-1), config("/tmp/config.ini") {
}

DroneInterface::~DroneInterface() {
    shutdown();
}

bool DroneInterface::initialize() {
    logStatus("Initializing drone interface...");
    
    // Check available serial devices
    logStatus("DEBUG: Checking available serial devices...");
    system("ls -la /dev/serial* /dev/tty* 2>/dev/null | head -10");
    
    logStatus("DEBUG: Loading configuration...");
    if (!config.loadConfig()) {
        logError("Failed to load configuration");
        logError("DEBUG: config.loadConfig() returned false");
        return false;
    }
    logStatus("DEBUG: Configuration loaded successfully");
    
    logStatus("DEBUG: Connecting to drone...");
    if (!connect()) {
        logError("Failed to connect to drone");
        logError("DEBUG: connect() returned false");
        return false;
    }
    logStatus("DEBUG: Connection established successfully");
    
    // Give the connection a moment to stabilize
    logStatus("DEBUG: Waiting 500ms for connection to stabilize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Test if there's any data coming from the serial port
    logStatus("DEBUG: Testing if any data is available from serial port...");
    int test_read = testReadByte(1000);
    if (test_read > 0) {
        logStatus("DEBUG: Found data on serial port");
    } else if (test_read == 0) {
        logStatus("DEBUG: No data available on serial port (EOF)");
    } else {
        logStatus("DEBUG: Serial port read test timeout or error");
    }
    
    logStatus("Drone interface initialized successfully");
    return true;
}

bool DroneInterface::shutdown() {
    disconnect();
    config.saveConfig();
    logStatus("Drone interface shutdown complete");
    return true;
}

bool DroneInterface::connect() {
    // Open serial device
    serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd == -1) {
        logError("Failed to open serial device: " + device);
        return false;
    }
    
    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd, &tty) != 0) {
        logError("Failed to get serial attributes");
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    
    // Set baud rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // 8N1 configuration
    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CSIZE;   // Clear data size bits
    tty.c_cflag |= CS8;      // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    
    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output
    tty.c_oflag &= ~OPOST;
    
    // Set timeouts
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5; // 0.5 second timeout
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        logError("Failed to set serial attributes");
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    
    // Flush buffers
    tcflush(serial_fd, TCIOFLUSH);
    
    logStatus("Serial device opened successfully, fd=" + std::to_string(serial_fd));
    return true;
}

void DroneInterface::disconnect() {
    if (serial_fd != -1) {
        close(serial_fd);
        serial_fd = -1;
        logStatus("Disconnected from drone");
    }
}

int DroneInterface::testReadByte(int timeout_ms) {
    if (serial_fd == -1) return -1;
    
    fd_set readfds;
    struct timeval tv;
    
    FD_ZERO(&readfds);
    FD_SET(serial_fd, &readfds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int result = select(serial_fd + 1, &readfds, NULL, NULL, &tv);
    if (result > 0) {
        char byte;
        ssize_t bytes_read = read(serial_fd, &byte, 1);
        if (bytes_read > 0) {
            return 1;
        } else if (bytes_read == 0) {
            return 0; // EOF
            }
        }
    return -1; // Timeout or error
}

bool DroneInterface::sendCLICommand(const std::string& command, const std::string& expected_response) {
    if (serial_fd == -1) return false;
    
    // Send command with newline
    std::string cmd = command + "\n";
    ssize_t bytes_written = write(serial_fd, cmd.c_str(), cmd.length());
    if (bytes_written != static_cast<ssize_t>(cmd.length())) {
        logError("Failed to send CLI command: " + command);
        return false;
    }
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Read response
    std::string response;
    char buffer[1024];
    ssize_t bytes_read;
    
    // Try to read multiple times to get complete response
    for (int attempt = 0; attempt < 5; attempt++) {
        while ((bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1)) > 0) {
            buffer[bytes_read] = '\0';
            response += buffer;
        }
        
        // Check if we got the expected response
        if (response.find(expected_response) != std::string::npos) {
    return true;
}

        // For CLI entry, also accept other valid responses
        if (command == "#") {
            if (response.find("CLI") != std::string::npos || 
                response.find("Entering CLI Mode") != std::string::npos ||
                response.find("#") != std::string::npos) {
                return true;
    }
        }
        // For exit command, accept empty response or any response
        else if (command == "exit") {
            return true;
        }
        
        // Small delay before next attempt
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    logError("CLI command failed - expected: '" + expected_response + "', got: '" + response + "'");
    return false;
}

bool DroneInterface::sendCLICommandWithResponse(const std::string& command, std::string& response) {
    if (serial_fd == -1) return false;
    
    // Send command with newline
    std::string cmd = command + "\n";
    ssize_t bytes_written = write(serial_fd, cmd.c_str(), cmd.length());
    if (bytes_written != static_cast<ssize_t>(cmd.length())) {
        logError("Failed to send CLI command: " + command);
        return false;
    }
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Read response
    response.clear();
    char buffer[1024];
    ssize_t bytes_read;
        
    // Try to read multiple times to get complete response
    for (int attempt = 0; attempt < 5; attempt++) {
        while ((bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1)) > 0) {
            buffer[bytes_read] = '\0';
            response += buffer;
        }
        
        // If we got some response, return it
        if (!response.empty()) {
    return true;
}

        // Small delay before next attempt
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return !response.empty();
}

bool DroneInterface::enterCLI() {
    logStatus("CLI: Sending '#'");
    return sendCLICommand("#", "#");
    }
    
void DroneInterface::exitCLI() {
    sendCLICommand("exit", "exit");
    // Add delay after exiting CLI to ensure connection is ready
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

bool DroneInterface::checkFirmwareVersion() {
    logStatus("Checking firmware version...");
    
    if (!enterCLI()) {
        logError("Failed to enter CLI mode");
        return false;
    }
    
    std::string response;
    if (!sendCLICommandWithResponse("version", response)) {
        logError("Failed to get version information");
        exitCLI();
        return false;
    }
    
    logStatus("CLI: Version response received");
    logStatus("CLI: Full version response: '" + response + "'");
    
    // Parse version information
    // Expected format: # Betaflight / STM32F7X2 (S7X2) 4.5.1 Apr 30 2025 / 17:08:33 () MSP API: 1.46
    if (response.find("Betaflight") != std::string::npos) {
        // Extract version number
        size_t pos = response.find("4.5.");
        if (pos != std::string::npos) {
            logStatus("Firmware version compatible: 4.5.x");
            logStatus("CLI: " + response.substr(0, response.find('\n')));
        } else {
            logError("Incompatible firmware version - expected 4.5.x");
            logStatus("CLI: " + response.substr(0, response.find('\n')));
            
            // Write error to /tmp/error.txt
            std::ofstream error_file("/tmp/error.txt");
            if (error_file.is_open()) {
                error_file << "Incompatible betaflight version" << std::endl;
                error_file.close();
                logStatus("Error written to /tmp/error.txt");
            }
            
            exitCLI();
            return false;
        }
    } else {
        logError("Not a Betaflight firmware");
        logStatus("CLI: " + response.substr(0, response.find('\n')));
        exitCLI();
        return false;
    }
    
    exitCLI();
    return true;
}

bool DroneInterface::setAUXConfigurationViaCLI() {
    logStatus("Setting AUX4 to MSP_OVERRIDE mode via CLI commands...");
    
    if (!enterCLI()) {
        logError("Failed to enter CLI mode");
        return false;
    }
    
    // Check current aux configuration
    logStatus("CLI: Checking current aux configuration...");
    std::string response;
    if (!sendCLICommandWithResponse("aux", response)) {
        logError("Failed to get aux configuration");
        exitCLI();
        return false;
    }
    
    logStatus("CLI: Current aux configuration received");
    logStatus("CLI: DEBUG - Full aux response: '" + response + "'");
    
    // Parse aux configuration to find MSP_OVERRIDE (function 50)
    std::istringstream iss(response);
    std::string line;
    int msp_override_channel = -1;
    int msp_override_aux_index = -1;
    int first_free_index = -1;
    bool aux_changed = false;
    
    while (std::getline(iss, line)) {
        if (line.find("aux ") == 0) {
            logStatus("CLI: DEBUG - Parsing aux line: '" + line + "'");
            // Parse aux line like "aux 2 50 3 1700 2100 0 0"
            std::istringstream line_iss(line);
            std::string aux_label;
            int aux_index;
            int function;
            int rc_channel;
            
            if (line_iss >> aux_label >> aux_index >> function >> rc_channel) {
                logStatus("CLI: DEBUG - Parsed: aux_index=" + std::to_string(aux_index) + ", function=" + std::to_string(function) + ", rc_channel=" + std::to_string(rc_channel));
                if (function == 50) { // MSP_OVERRIDE function
                    msp_override_channel = rc_channel + 4; // Convert to AUX index (value + 4)
                    msp_override_aux_index = aux_index;
                    logStatus("CLI: Found existing MSP_OVERRIDE on aux index " + std::to_string(aux_index) + " -> RC channel " + std::to_string(rc_channel) + " (saved as " + std::to_string(msp_override_channel) + ")");
                    break;
                }
                // Track first free slot (exclude index 0)
                if (function == 0 && rc_channel == 0 && aux_index != 0 && first_free_index == -1) {
                    first_free_index = aux_index;
                }
            }
        }
    }
    
    if (msp_override_channel != -1) {
        logStatus("CLI: MSP_OVERRIDE already configured on aux index " + std::to_string(msp_override_aux_index));
        config.setMSPOverrideChannel(msp_override_channel);
        config.saveConfig();
        logStatus("Saved existing MSP override channel: aux index " + std::to_string(msp_override_aux_index) + " -> RC channel " + std::to_string(msp_override_channel + 1) + " (saved as " + std::to_string(msp_override_channel) + ")");
    } else {
        if (first_free_index == -1) {
            logError("No free AUX slot found (excluding index 0) to set MSP_OVERRIDE");
            exitCLI();
            return false;
        }
        // Set MSP_OVERRIDE on the first free slot, using rc_channel=3 (RC8) and range 1800-2100 per request
        std::string aux_cmd = "aux " + std::to_string(first_free_index) + " 50 3 1800 2100 0 0";
        logStatus("CLI: Sending '" + aux_cmd + "'");
        if (!sendCLICommand(aux_cmd, aux_cmd)) {
            logError("Failed to set MSP_OVERRIDE on aux index " + std::to_string(first_free_index));
            exitCLI();
            return false;
        }
        logStatus("CLI: ✓ Command successful");
        aux_changed = true;
        config.setMSPOverrideChannel(7); // rc_channel 3 + 4 = 7
        config.saveConfig();
        logStatus("Saved MSP override channel: aux index " + std::to_string(first_free_index) + " -> RC channel 8 (saved as 7)");
    }
    
    if (aux_changed) {
        if (!sendCLICommand("save", "save")) {
            logError("Failed to save settings after AUX change");
            exitCLI();
            return false;
        }
        logStatus("CLI: Configuration saved, drone will reboot");
    }
    
    exitCLI();
    return true;
}

bool DroneInterface::setParametersViaCLI() {
    logStatus("Setting parameters via CLI commands...");
    
    if (!enterCLI()) {
        logError("Failed to enter CLI mode");
        return false;
    }
    
    // Function to send CLI command and wait for response
    auto sendCLICommand = [this](const std::string& command, const std::string& expected_response) -> bool {
        logStatus("CLI: Sending '" + command + "'");
        
        if (!this->sendCLICommand(command, expected_response)) {
            logError("CLI: Failed to send command: " + command);
            return false;
        }
        
                    logStatus("CLI: ✓ Command successful");
                    return true;
    };
    
    // Check and set each parameter
    std::vector<std::pair<std::string, std::string>> parameters = {
        {"thrust_linear", "0"},
        {"roll_srate", "67"},
        {"pitch_srate", "67"},
        {"yaw_srate", "67"},
        {"rates_type", "ACTUAL"},
        {"roll_rc_rate", "7"},
        {"pitch_rc_rate", "7"},
        {"yaw_rc_rate", "7"},
        {"msp_override_channels_mask", "15"},
        {"msp_override_failsafe", "ON"}
    };
    
    bool needs_save = false;
    
    for (const auto& param : parameters) {
        std::string response;
        if (!sendCLICommandWithResponse("get " + param.first, response)) {
            logError("CLI: Failed to get " + param.first);
            exitCLI();
        return false;
    }
    
        // Parse current value
        std::istringstream iss(response);
        std::string line;
        std::string current_value;
        
        while (std::getline(iss, line)) {
            if (line.find(param.first + " =") != std::string::npos) {
                size_t pos = line.find("=");
                if (pos != std::string::npos) {
                    current_value = line.substr(pos + 1);
                    // Clean the string by removing all whitespace and control characters
                    std::string cleaned;
                    for (char c : current_value) {
                        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
                            cleaned += c;
                        }
                    }
                    current_value = cleaned;
                    break;
                }
            }
        }
        
        // If we couldn't parse the value, assume it's correct to avoid unnecessary changes
        if (current_value.empty()) {
            logStatus("CLI: Could not parse " + param.first + ", assuming correct");
            continue;
        }
        

        
        if (current_value != param.second) {
            logStatus("CLI: " + param.first + " mismatch: current='" + current_value + "', expected='" + param.second + "'");
            if (!sendCLICommand("set " + param.first + " = " + param.second, "set " + param.first + " = " + param.second)) {
                logError("CLI: Failed to set " + param.first);
                exitCLI();
                return false;
            }
            needs_save = true;
        } else {
            logStatus("CLI: " + param.first + " already correct: '" + current_value + "'");
        }
    }
    
    // Save if needed
    if (needs_save) {
        if (!sendCLICommand("save", "save")) {
            logError("Failed to save settings");
            exitCLI();
            return false;
        }
        logStatus("CLI: Configuration saved, drone will reboot");
    } else {
        logStatus("CLI: All settings already correct, no save needed");
    }
    
    exitCLI();
    return true;
}

bool DroneInterface::setAUXAndParametersViaCLI() {
    logStatus("Setting AUX and parameters via CLI commands...");
    
    if (!enterCLI()) {
        logError("Failed to enter CLI mode");
        return false;
    }
    
    // Check current aux configuration
    logStatus("CLI: Checking current aux configuration...");
    std::string response;
    if (!sendCLICommandWithResponse("aux", response)) {
        logError("Failed to get aux configuration");
        exitCLI();
        return false;
    }
    
    logStatus("CLI: Current aux configuration received");
    logStatus("CLI: DEBUG - Full aux response: '" + response + "'");
    logStatus("CLI: DEBUG - Full aux response: '" + response + "'");
    
    // Parse aux configuration to find MSP_OVERRIDE (function 50)
    std::istringstream iss(response);
    std::string line;
    int msp_override_channel = -1;
    int msp_override_aux_index = -1;
    int first_free_index = -1;
    bool aux_changed = false;
    
    while (std::getline(iss, line)) {
        if (line.find("aux ") == 0) {
            logStatus("CLI: DEBUG - Parsing aux line: '" + line + "'");
            // Parse aux line like "aux 2 50 3 1700 2100 0 0"
            std::istringstream line_iss(line);
            std::string aux_label;
            int aux_index;
            int function;
            int rc_channel;
            
            if (line_iss >> aux_label >> aux_index >> function >> rc_channel) {
                logStatus("CLI: DEBUG - Parsed: aux_index=" + std::to_string(aux_index) + ", function=" + std::to_string(function) + ", rc_channel=" + std::to_string(rc_channel));
                if (function == 50) { // MSP_OVERRIDE function
                    msp_override_channel = rc_channel + 4; // Convert to AUX index (value + 4)
                    msp_override_aux_index = aux_index;
                    logStatus("CLI: Found existing MSP_OVERRIDE on aux index " + std::to_string(aux_index) + " -> RC channel " + std::to_string(rc_channel) + " (saved as " + std::to_string(msp_override_channel) + ")");
                    break;
                }
                // Track first free slot (exclude index 0)
                if (function == 0 && rc_channel == 0 && aux_index != 0 && first_free_index == -1) {
                    first_free_index = aux_index;
                }
            }
        }
    }
    
    if (msp_override_channel != -1) {
        logStatus("CLI: MSP_OVERRIDE already configured on aux index " + std::to_string(msp_override_aux_index));
        config.setMSPOverrideChannel(msp_override_channel);
        config.saveConfig();
        logStatus("Saved existing MSP override channel: aux index " + std::to_string(msp_override_aux_index) + " -> RC channel " + std::to_string(msp_override_channel + 1) + " (saved as " + std::to_string(msp_override_channel) + ")");
    } else {
        if (first_free_index == -1) {
            logError("No free AUX slot found (excluding index 0) to set MSP_OVERRIDE");
            exitCLI();
            return false;
        }
        // Set MSP_OVERRIDE on the first free slot, using rc_channel=3 (RC8) and range 1800-2100 per request
        std::string aux_cmd = "aux " + std::to_string(first_free_index) + " 50 3 1800 2100 0 0";
        logStatus("CLI: Sending '" + aux_cmd + "'");
        if (!sendCLICommand(aux_cmd, aux_cmd)) {
            logError("Failed to set MSP_OVERRIDE on aux index " + std::to_string(first_free_index));
            exitCLI();
            return false;
        }
        logStatus("CLI: ✓ Command successful");
        aux_changed = true;
        config.setMSPOverrideChannel(7); // rc_channel 3 + 4 = 7
        config.saveConfig();
        logStatus("Saved MSP override channel: aux index " + std::to_string(first_free_index) + " -> RC channel 8 (saved as 7)");
    }
    
    // Check current parameter values
    logStatus("CLI: Checking current parameter values...");
    
    // Function to send CLI command and wait for response
    auto sendCLICommand = [this](const std::string& command, const std::string& expected_response) -> bool {
        logStatus("CLI: Sending '" + command + "'");
        
        if (!this->sendCLICommand(command, expected_response)) {
            logError("CLI: Failed to send command: " + command);
            return false;
        }
        
        logStatus("CLI: ✓ Command successful");
        return true;
    };
    
    // Check and set each parameter
    std::vector<std::pair<std::string, std::string>> parameters = {
        {"thrust_linear", "0"},
        {"roll_srate", "67"},
        {"pitch_srate", "67"},
        {"yaw_srate", "67"},
        {"rates_type", "ACTUAL"},
        {"roll_rc_rate", "7"},
        {"pitch_rc_rate", "7"},
        {"yaw_rc_rate", "7"},
        {"msp_override_channels_mask", "15"},
        {"msp_override_failsafe", "ON"}
    };
    
    bool needs_save = aux_changed;
    
    for (const auto& param : parameters) {
        std::string response;
        if (!sendCLICommandWithResponse("get " + param.first, response)) {
            logError("CLI: Failed to get " + param.first);
            exitCLI();
            return false;
        }
        
        // Parse current value
        std::istringstream iss(response);
        std::string line;
        std::string current_value;
        
        while (std::getline(iss, line)) {
            if (line.find(param.first + " =") != std::string::npos) {
                size_t pos = line.find("=");
                if (pos != std::string::npos) {
                    current_value = line.substr(pos + 1);
                    // Clean the string by removing all whitespace and control characters
                    std::string cleaned;
                    for (char c : current_value) {
                        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
                            cleaned += c;
                        }
                    }
                    current_value = cleaned;
                    break;
                }
            }
        }
        
        if (current_value != param.second) {
            logStatus("CLI: " + param.first + " mismatch: current='" + current_value + "', expected='" + param.second + "'");
            if (!sendCLICommand("set " + param.first + " = " + param.second, "set " + param.first + " = " + param.second)) {
                logError("CLI: Failed to set " + param.first);
                exitCLI();
                return false;
            }
            needs_save = true;
        } else {
            logStatus("CLI: " + param.first + " already correct: " + current_value);
        }
    }
    
    // Save if needed
    if (needs_save) {
        if (!sendCLICommand("save", "save")) {
            logError("Failed to save settings");
            exitCLI();
            return false;
        }
        logStatus("CLI: Configuration saved, drone will reboot");
    } else {
        logStatus("CLI: All settings already correct, no save needed");
    }
    
    // Exit CLI
    if (!sendCLICommand("exit", "exit")) {
        logError("Failed to exit CLI");
        exitCLI();
        return false;
    }
    
    logStatus("AUX and parameter configuration via CLI completed successfully");
    return true;
}

bool DroneInterface::readMSPOverrideChannelViaMSP() {
    logStatus("MSP: Reading MSP_OVERRIDE via MSP_MODE_RANGES (no CLI)...");

    MSPConnection msp("/dev/serial0");
    if (!msp.connect()) {
        logError("MSP: Failed to connect");
        config.setMSPOverrideChannel(-1);
        config.saveConfig();
        return false;
    }

    MSPMessage req(MSP_MODE_RANGES);
    MSPMessage resp;
    if (!msp.sendAndReceive(req, resp, 1000)) {
        logError("MSP: MSP_MODE_RANGES request failed or timed out");
        config.setMSPOverrideChannel(-1);
        config.saveConfig();
        msp.disconnect();
        return false;
    }

    logStatus("MSP: Received MODE_RANGES payload size=" + std::to_string((int)resp.size));

    // Payload is groups of 4 bytes: permanentId, auxChannelIndex, rangeStartStep, rangeEndStep
    const std::vector<uint8_t>& d = resp.data;
    if (d.size() % 4 != 0) {
        logError("MSP: Unexpected MODE_RANGES payload length");
        config.setMSPOverrideChannel(-1);
        config.saveConfig();
        msp.disconnect();
        return false;
    }

    int found_rc_channel = -1;
    for (size_t i = 0; i + 3 < d.size(); i += 4) {
        uint8_t permanentId = d[i + 0];
        uint8_t auxChannelIndex = d[i + 1];
        uint8_t rangeStartStep = d[i + 2];
        uint8_t rangeEndStep = d[i + 3];

        // Unused slots usually have start==end; only consider active ranges
        if (permanentId == MSP_OVERRIDE_MODE && rangeStartStep != rangeEndStep) {
            int rc_channel = static_cast<int>(auxChannelIndex) + 4; // 0-indexed RC channel
            found_rc_channel = rc_channel;
            logStatus("MSP: Found MSP_OVERRIDE in MODE_RANGES on AUX" + std::to_string((int)auxChannelIndex) +
                      " -> RC channel " + std::to_string(rc_channel));
            break;
        }
    }

    if (found_rc_channel == -1) {
        logStatus("MSP: MSP_OVERRIDE not found in MODE_RANGES");
        config.setMSPOverrideChannel(-1);
    } else {
        config.setMSPOverrideChannel(found_rc_channel);
    }
    config.saveConfig();

    msp.disconnect();
    return true;
}

void DroneInterface::logStatus(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "] " 
              << message << std::endl;
}

void DroneInterface::logError(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::cerr << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "] ERROR: " 
              << message << std::endl;
}