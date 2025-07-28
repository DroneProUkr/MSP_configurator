#pragma once

#include "msp_protocol.h"
#include <string>
#include <chrono>

class MSPConnection {
private:
    int serial_fd;
    bool connected;
    std::string device_path;
    static constexpr int TIMEOUT_MS = 1000;
    static constexpr int BAUD_RATE = 115200;
    
public:
    MSPConnection(const std::string& device = "/dev/serial0");
    ~MSPConnection();
    
    bool connect();
    void disconnect();
    bool isConnected() const { return connected; }
    
    bool sendMessage(const MSPMessage& msg);
    bool receiveMessage(MSPMessage& msg, int timeout_ms = TIMEOUT_MS);
    bool sendAndReceive(const MSPMessage& request, MSPMessage& response, int timeout_ms = TIMEOUT_MS);
    
    // Test method to check if any data is available
    int testReadByte(int timeout_ms = 1000);
    
    // Test different baud rates
    bool testBaudRate(int baud_rate);
    
private:
    bool configureSerial();
    int readBytes(uint8_t* buffer, int max_bytes, int timeout_ms);
};