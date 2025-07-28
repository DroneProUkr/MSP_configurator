#include "msp_protocol.h"
#include <iostream>
#include <cstring>

MSPMessage::MSPMessage(uint8_t command, const std::vector<uint8_t>& payload) 
    : cmd(command), data(payload) {
    size = data.size();
    checksum = MSPProtocol::calculateChecksum(*this);
}

std::vector<uint8_t> MSPProtocol::encodeMessage(const MSPMessage& msg) {
    std::vector<uint8_t> buffer;
    
    std::cout << "DEBUG: Encoding MSP message - cmd=" << (int)msg.cmd << ", size=" << (int)msg.size << std::endl;
    
    // Header
    buffer.insert(buffer.end(), msg.header, msg.header + 3);
    
    // Size and command
    buffer.push_back(msg.size);
    buffer.push_back(msg.cmd);
    
    // Data
    buffer.insert(buffer.end(), msg.data.begin(), msg.data.end());
    
    // Checksum
    buffer.push_back(msg.checksum);
    
    std::cout << "DEBUG: Encoded message size: " << buffer.size() << std::endl;
    return buffer;
}

bool MSPProtocol::decodeMessage(const std::vector<uint8_t>& buffer, MSPMessage& msg) {
    std::cout << "DEBUG: Decoding MSP message, buffer size: " << buffer.size() << std::endl;
    
    if (buffer.size() < 6) {
        std::cout << "DEBUG: Buffer too small for MSP message (need 6, got " << buffer.size() << ")" << std::endl;
        return false;
    }
    
    // Check header - accept both $M> and $M! (error responses)
    if (buffer[0] != '$' || buffer[1] != 'M' || (buffer[2] != '>' && buffer[2] != '!' && buffer[2] != '<')) {
        std::cout << "DEBUG: Invalid header - expected $M>, $M<, or $M!, got: " 
                  << (char)buffer[0] << (char)buffer[1] << (char)buffer[2] << std::endl;
        return false;
    }
    
    msg.header[0] = buffer[0];
    msg.header[1] = buffer[1];
    msg.header[2] = buffer[2];
    
    msg.size = buffer[3];
    msg.cmd = buffer[4];
    
    std::cout << "DEBUG: Header valid, size=" << (int)msg.size << ", cmd=" << (int)msg.cmd << std::endl;
    
    if (buffer.size() < static_cast<size_t>(6 + msg.size)) {
        std::cout << "DEBUG: Buffer too small for payload (need " << (6 + msg.size) 
                  << ", got " << buffer.size() << ")" << std::endl;
        return false;
    }
    
    msg.data.clear();
    msg.data.insert(msg.data.end(), buffer.begin() + 5, buffer.begin() + 5 + msg.size);
    
    msg.checksum = buffer[5 + msg.size];
    
    std::cout << "DEBUG: Extracted data size: " << msg.data.size() << ", checksum: 0x" 
              << std::hex << (int)msg.checksum << std::dec << std::endl;
    
    // Verify checksum
    uint8_t calculated_checksum = calculateChecksum(msg);
    bool checksum_valid = calculated_checksum == msg.checksum;
    
    std::cout << "DEBUG: Checksum verification - calculated: 0x" << std::hex << (int)calculated_checksum 
              << ", received: 0x" << (int)msg.checksum << std::dec 
              << ", valid: " << (checksum_valid ? "YES" : "NO") << std::endl;
    
    return checksum_valid;
}

uint8_t MSPProtocol::calculateChecksum(const MSPMessage& msg) {
    uint8_t checksum = msg.size ^ msg.cmd;
    for (uint8_t byte : msg.data) {
        checksum ^= byte;
    }
    return checksum;
}

std::vector<AuxFunction> MSPProtocol::parseAuxFunctions(const std::vector<uint8_t>& data) {
    std::vector<AuxFunction> functions;
    
    std::cout << "DEBUG: parseAuxFunctions called with " << data.size() << " bytes" << std::endl;
    
    // Each aux function is 7 bytes
    for (size_t i = 0; i + 6 < data.size(); i += 7) {
        AuxFunction func;
        func.aux_id = data[i];
        func.mode = data[i + 1];
        func.aux_channel = data[i + 2];
        func.range_start = (data[i + 4] << 8) | data[i + 3];
        func.range_end = (data[i + 6] << 8) | data[i + 5];
        func.extra1 = 0;  // Not used in basic format
        func.extra2 = 0;
        
        std::cout << "DEBUG: Parsed aux function " << (i/7) << " - "
                  << "ID:" << (int)func.aux_id << " "
                  << "Mode:" << (int)func.mode << " "
                  << "Channel:" << (int)func.aux_channel << " "
                  << "Range:" << func.range_start << "-" << func.range_end << std::endl;
        
        functions.push_back(func);
    }
    
    std::cout << "DEBUG: Total aux functions parsed: " << functions.size() << std::endl;
    return functions;
}

RCTuning MSPProtocol::parseRCTuning(const std::vector<uint8_t>& data) {
    RCTuning tuning = {};
    
    if (data.size() >= 14) {
        tuning.roll_rc_rate = data[0];
        tuning.pitch_rc_rate = data[1];
        tuning.yaw_rc_rate = data[2];
        tuning.roll_srate = data[3];
        tuning.pitch_srate = data[4];
        tuning.yaw_srate = data[5];
        tuning.thr_mid = data[6];
        tuning.thr_expo = data[7];
        tuning.tpa_rate = (data[9] << 8) | data[8];
        tuning.tpa_breakpoint = (data[11] << 8) | data[10];
        tuning.expo_8 = data[12];
        
        if (data.size() >= 17) {
            tuning.roll_rate_limit = data[13];
            tuning.pitch_rate_limit = data[14];
            tuning.yaw_rate_limit = data[15];
        }
        
        if (data.size() >= 21) {
            tuning.thrust_linear = data[16];
            tuning.rates_type = data[17];
            tuning.msp_override_channels_mask = data[18];
            tuning.msp_override_failsafe = data[19];
        }
    }
    
    return tuning;
}

std::vector<uint8_t> MSPProtocol::encodeRCTuning(const RCTuning& tuning) {
    std::vector<uint8_t> data;
    
    data.push_back(tuning.roll_rc_rate);
    data.push_back(tuning.pitch_rc_rate);
    data.push_back(tuning.yaw_rc_rate);
    data.push_back(tuning.roll_srate);
    data.push_back(tuning.pitch_srate);
    data.push_back(tuning.yaw_srate);
    data.push_back(tuning.thr_mid);
    data.push_back(tuning.thr_expo);
    data.push_back(tuning.tpa_rate & 0xFF);
    data.push_back((tuning.tpa_rate >> 8) & 0xFF);
    data.push_back(tuning.tpa_breakpoint & 0xFF);
    data.push_back((tuning.tpa_breakpoint >> 8) & 0xFF);
    data.push_back(tuning.expo_8);
    data.push_back(tuning.roll_rate_limit);
    data.push_back(tuning.pitch_rate_limit);
    data.push_back(tuning.yaw_rate_limit);
    data.push_back(tuning.thrust_linear);
    data.push_back(tuning.rates_type);
    data.push_back(tuning.msp_override_channels_mask);
    data.push_back(tuning.msp_override_failsafe);
    
    return data;
}

std::vector<uint8_t> MSPProtocol::encodeAuxFunctions(const std::vector<AuxFunction>& functions) {
    std::vector<uint8_t> data;
    
    for (const auto& func : functions) {
        data.push_back(func.aux_id);
        data.push_back(func.mode);
        data.push_back(func.aux_channel);
        data.push_back(func.range_start & 0xFF);
        data.push_back((func.range_start >> 8) & 0xFF);
        data.push_back(func.range_end & 0xFF);
        data.push_back((func.range_end >> 8) & 0xFF);
    }
    
    return data;
}

// New methods for better compatibility
MSPMessage MSPProtocol::createIdentRequest() {
    return MSPMessage(MSP_IDENT);
}

MSPMessage MSPProtocol::createStatusRequest() {
    return MSPMessage(MSP_STATUS);
}

MSPMessage MSPProtocol::createRcRequest() {
    return MSPMessage(MSP_RC);
}

MSPMessage MSPProtocol::createAttitudeRequest() {
    return MSPMessage(MSP_ATTITUDE);
}

MSPMessage MSPProtocol::createAltitudeRequest() {
    return MSPMessage(MSP_ALTITUDE);
}

MSPMessage MSPProtocol::createAnalogRequest() {
    return MSPMessage(MSP_ANALOG);
}

MSPMessage MSPProtocol::createSetRCTuningRequest(const RCTuning& tuning) {
    std::vector<uint8_t> data = encodeRCTuning(tuning);
    return MSPMessage(MSP_SET_RC_TUNING, data);
}