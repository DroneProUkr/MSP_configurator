#include "drone_interface.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream> // Added for error file writing
#include <sstream> // Added for string parsing

DroneInterface::DroneInterface(const std::string& device) 
    : connection(device), config("/tmp/config.ini") {
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
    if (!connection.connect()) {
        logError("Failed to connect to drone");
        logError("DEBUG: connection.connect() returned false");
        return false;
    }
    logStatus("DEBUG: Connection established successfully");
    
    // Give the connection a moment to stabilize
    logStatus("DEBUG: Waiting 500ms for connection to stabilize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Test if there's any data coming from the serial port
    logStatus("DEBUG: Testing if any data is available from serial port...");
    int test_read = connection.testReadByte(1000);
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
    connection.disconnect();
    config.saveConfig();
    logStatus("Drone interface shutdown complete");
    return true;
}

bool DroneInterface::getAuxChannels(std::vector<AuxFunction>& aux_functions) {
    logStatus("Requesting aux channel configuration...");
    
    // Simple test: just try MSP_IDENT first
    logStatus("DEBUG: Testing basic MSP_IDENT command...");
    MSPMessage ident_request(MSP_IDENT);
    MSPMessage ident_response;
    
    if (!connection.sendAndReceive(ident_request, ident_response, 2000)) {
        logError("DEBUG: MSP_IDENT failed - no MSP communication");
        return false;
    }
    
    logStatus("DEBUG: MSP_IDENT successful! Drone responds to MSP");
    logStatus("DEBUG: Response cmd=" + std::to_string(ident_response.cmd) + 
              ", size=" + std::to_string(ident_response.size));
    
    // Try MSP_RC command (this is what the working VOT_C app uses)
    logStatus("DEBUG: Trying MSP_RC command (cmd=105)...");
    MSPMessage rc_request(MSP_RC);
    MSPMessage rc_response;
    
    if (!connection.sendAndReceive(rc_request, rc_response, 2000)) {
        logError("DEBUG: MSP_RC failed");
        return false;
    }
    
    logStatus("DEBUG: MSP_RC successful! Response size: " + std::to_string(rc_response.data.size()));
    
    // Parse RC data to see if we can find aux channels
    if (rc_response.data.size() >= 16) {
        logStatus("DEBUG: RC data contains " + std::to_string(rc_response.data.size()) + " channels");
        
        // Print first few channels to see the data
        for (size_t i = 0; i < std::min(rc_response.data.size(), size_t(16)); i += 2) {
            if (i + 1 < rc_response.data.size()) {
                uint16_t channel_value = (rc_response.data[i+1] << 8) | rc_response.data[i];
                logStatus("DEBUG: Channel " + std::to_string(i/2) + ": " + std::to_string(channel_value));
            }
        }
        
        // For now, let's assume aux channels start from channel 4 (indices 8-9, 10-11, etc.)
        // Create dummy aux functions based on RC data
        // Extend to include AUX6 (aux_id 5) which would be RC channel 10
        for (int i = 4; i < 12; i++) {  // Extend to include more aux channels
            if (i*2 + 1 < rc_response.data.size()) {
                uint16_t channel_value = (rc_response.data[i*2+1] << 8) | rc_response.data[i*2];
                logStatus("DEBUG: Aux channel " + std::to_string(i-4) + " value: " + std::to_string(channel_value));
                
                AuxFunction func;
                func.aux_id = i - 4;
                func.aux_channel = i - 4;
                func.mode = 0; // Unknown mode
                func.range_start = 1000;
                func.range_end = 2000;
                func.extra1 = 0;
                func.extra2 = 0;
                
                aux_functions.push_back(func);
            }
        }
        
        logStatus("DEBUG: Created " + std::to_string(aux_functions.size()) + " aux functions from RC data");
        return true;
    }
    
    // Fallback: try AUX_FUNCTIONS command
    logStatus("DEBUG: Trying MSP_AUX_FUNCTIONS (cmd=" + std::to_string(MSP_AUX_FUNCTIONS) + ")");
    MSPMessage request(MSP_AUX_FUNCTIONS);
    MSPMessage response;
    
    if (!connection.sendAndReceive(request, response, 2000)) {
        logError("Failed to get aux channels");
        logError("DEBUG: MSP_AUX_FUNCTIONS sendAndReceive failed");
        return false;
    }
    
    logStatus("DEBUG: Received response - cmd=" + std::to_string(response.cmd) + 
              ", size=" + std::to_string(response.size) + 
              ", data_size=" + std::to_string(response.data.size()));
    
    if (response.cmd != MSP_AUX_FUNCTIONS) {
        logError("Received unexpected response for aux channels");
        logError("DEBUG: Expected cmd=" + std::to_string(MSP_AUX_FUNCTIONS) + 
                 ", got cmd=" + std::to_string(response.cmd));
        return false;
    }
    
    logStatus("DEBUG: Parsing aux functions from " + std::to_string(response.data.size()) + " bytes");
    
    aux_functions = MSPProtocol::parseAuxFunctions(response.data);
    logStatus("Retrieved " + std::to_string(aux_functions.size()) + " aux functions");
    
    // Debug: Print each aux function
    for (size_t i = 0; i < aux_functions.size(); i++) {
        const auto& func = aux_functions[i];
        logStatus("DEBUG: Aux[" + std::to_string(i) + "] - ID:" + std::to_string(func.aux_id) + 
                  " Mode:" + std::to_string(func.mode) + 
                  " Channel:" + std::to_string(func.aux_channel) + 
                  " Range:" + std::to_string(func.range_start) + "-" + std::to_string(func.range_end));
    }
    
    return true;
}

int DroneInterface::findMSPOverrideChannel() {
    std::vector<AuxFunction> aux_functions;
    
    if (!getAuxChannels(aux_functions)) {
        return -1;
    }
    
    // Check if we're using RC data (all modes will be 0)
    bool using_rc_data = true;
    for (const auto& func : aux_functions) {
        if (func.mode != 0) {
            using_rc_data = false;
            break;
        }
    }
    
    if (using_rc_data) {
        logStatus("DEBUG: Using RC data - mode information not available");
        logStatus("DEBUG: Looking for any AUX set to MSP_OVERRIDE");
        
        // Since we're using RC data, we can't determine which AUX is set to MSP_OVERRIDE
        // from the mode information (all modes are 0). We need to rely on CLI or other methods.
        // For now, return -1 to indicate no MSP_OVERRIDE found, so the calling code
        // will set AUX4 to MSP_OVERRIDE
        logStatus("DEBUG: Cannot determine MSP_OVERRIDE from RC data, returning -1");
        return -1;
    }
    
    // Using MSP_AUX_FUNCTIONS data - look for MSP_OVERRIDE mode
    for (const auto& func : aux_functions) {
        logStatus("Aux " + std::to_string(func.aux_id) + 
                  " mode " + std::to_string(func.mode) + 
                  " channel " + std::to_string(func.aux_channel) + 
                  " range " + std::to_string(func.range_start) + "-" + std::to_string(func.range_end));
        
        // Look for any aux channel set to MSP_OVERRIDE mode
        if (func.mode == MSP_OVERRIDE_MODE) {  // Check if this aux channel is set to MSP_OVERRIDE
            logStatus("Found MSP override on aux channel " + std::to_string(func.aux_id + 1) + " (aux_id " + std::to_string(func.aux_id) + ")");
            return func.aux_channel;
        }
    }
    
    logError("MSP_OVERRIDE mode not found in any aux channel");
    return -1;
}

bool DroneInterface::saveMSPOverrideChannel(int aux_channel_index) {
    // Save the found aux channel as the MSP override channel
    // aux_channel_index is the RC channel number for the found aux channel
    int rc_channel = aux_channel_index;
    
    config.setMSPOverrideChannel(rc_channel);
    
    if (!config.saveConfig()) {
        logError("Failed to save MSP override channel to config");
        return false;
    }
    
    logStatus("Saved MSP override channel: aux channel -> RC channel " + std::to_string(rc_channel));
    return true;
}

bool DroneInterface::getRCTuning(RCTuning& tuning) {
    MSPMessage request(MSP_RC_TUNING);
    MSPMessage response;
    
    if (!connection.sendAndReceive(request, response)) {
        logError("Failed to get RC tuning");
        return false;
    }
    
    if (response.cmd != MSP_RC_TUNING) {
        logError("Received unexpected response for RC tuning");
        return false;
    }
    
    tuning = MSPProtocol::parseRCTuning(response.data);
    return true;
}

bool DroneInterface::setRCTuning(const RCTuning& tuning) {
    std::vector<uint8_t> data = MSPProtocol::encodeRCTuning(tuning);
    MSPMessage request(MSP_SET_RC_TUNING, data);
    MSPMessage response;
    
    if (!connection.sendAndReceive(request, response)) {
        logError("Failed to set RC tuning");
        return false;
    }
    
    return true;
}

bool DroneInterface::getCurrentParameters(DroneParameters& params) {
    RCTuning tuning;
    if (!getRCTuning(tuning)) {
        return false;
    }
    
    params.roll_rc_rate = tuning.roll_rc_rate;
    params.pitch_rc_rate = tuning.pitch_rc_rate;
    params.yaw_rc_rate = tuning.yaw_rc_rate;
    params.roll_srate = tuning.roll_srate;
    params.pitch_srate = tuning.pitch_srate;
    params.yaw_srate = tuning.yaw_srate;
    
    // Note: Some parameters like thrust_linear, msp_override_channels_mask, etc.
    // would require additional MSP commands that may be flight controller specific
    // For now, we'll focus on the RC tuning parameters that are standard
    
    return true;
}

bool DroneInterface::setParameters(const DroneParameters& params) {
    RCTuning current_tuning;
    if (!getRCTuning(current_tuning)) {
        return false;
    }
    
    // Update the tuning with new parameters
    current_tuning.roll_rc_rate = params.roll_rc_rate;
    current_tuning.pitch_rc_rate = params.pitch_rc_rate;
    current_tuning.yaw_rc_rate = params.yaw_rc_rate;
    current_tuning.roll_srate = params.roll_srate;
    current_tuning.pitch_srate = params.pitch_srate;
    current_tuning.yaw_srate = params.yaw_srate;
    
    return setRCTuning(current_tuning);
}

bool DroneInterface::verifyParameters() {
    logStatus("Verifying drone parameters...");
    
    MSPMessage request(MSP_RC_TUNING);
    MSPMessage response;
    
    if (!connection.sendAndReceive(request, response, 1000)) {
        logError("Failed to get RC tuning parameters");
        return false;
    }
    
    if (response.cmd != MSP_RC_TUNING) {
        logError("Received unexpected response for RC tuning");
        return false;
    }
    
    // Debug: Show raw data bytes
    logStatus("DEBUG: RC tuning raw data (" + std::to_string(response.data.size()) + " bytes):");
    std::string hex_data = "";
    for (size_t i = 0; i < response.data.size(); i++) {
        hex_data += std::to_string(response.data[i]) + " ";
    }
    logStatus("DEBUG: " + hex_data);
    
    // Parse RC tuning parameters
    RCTuning tuning = MSPProtocol::parseRCTuning(response.data);
    
    // Debug: Show parsed values
    logStatus("DEBUG: Parsed values:");
    logStatus("  roll_rc_rate: " + std::to_string(tuning.roll_rc_rate));
    logStatus("  pitch_rc_rate: " + std::to_string(tuning.pitch_rc_rate));
    logStatus("  yaw_rc_rate: " + std::to_string(tuning.yaw_rc_rate));
    logStatus("  roll_srate: " + std::to_string(tuning.roll_srate));
    logStatus("  pitch_srate: " + std::to_string(tuning.pitch_srate));
    logStatus("  yaw_srate: " + std::to_string(tuning.yaw_srate));
    logStatus("  thrust_linear: " + std::to_string(tuning.thrust_linear));
    logStatus("  rates_type: " + std::to_string(tuning.rates_type));
    logStatus("  msp_override_channels_mask: " + std::to_string(tuning.msp_override_channels_mask));
    logStatus("  msp_override_failsafe: " + std::to_string(tuning.msp_override_failsafe));
    
    bool needs_update = false;
    
    // Check thrust_linear = 0
    if (tuning.thrust_linear != 0) {
        logStatus("thrust_linear mismatch: expected 0, got " + std::to_string(tuning.thrust_linear));
        needs_update = true;
    }
    
    // Check roll_srate = 67
    if (tuning.roll_srate != 67) {
        logStatus("roll_srate mismatch: expected 67, got " + std::to_string(tuning.roll_srate));
        needs_update = true;
    }
    
    // Check pitch_srate = 67
    if (tuning.pitch_srate != 67) {
        logStatus("pitch_srate mismatch: expected 67, got " + std::to_string(tuning.pitch_srate));
        needs_update = true;
    }
    
    // Check yaw_srate = 67
    if (tuning.yaw_srate != 67) {
        logStatus("yaw_srate mismatch: expected 67, got " + std::to_string(tuning.yaw_srate));
        needs_update = true;
    }
    
    // Check rates_type = ACTUAL (assuming ACTUAL = 0)
    if (tuning.rates_type != 0) {
        logStatus("rates_type mismatch: expected ACTUAL(0), got " + std::to_string(tuning.rates_type));
        needs_update = true;
    }
    
    // Check roll_rc_rate = 7
    if (tuning.roll_rc_rate != 7) {
        logStatus("roll_rc_rate mismatch: expected 7, got " + std::to_string(tuning.roll_rc_rate));
        needs_update = true;
    }
    
    // Check pitch_rc_rate = 7
    if (tuning.pitch_rc_rate != 7) {
        logStatus("pitch_rc_rate mismatch: expected 7, got " + std::to_string(tuning.pitch_rc_rate));
        needs_update = true;
    }
    
    // Check yaw_rc_rate = 7
    if (tuning.yaw_rc_rate != 7) {
        logStatus("yaw_rc_rate mismatch: expected 7, got " + std::to_string(tuning.yaw_rc_rate));
        needs_update = true;
    }
    
    // Check msp_override_channels_mask = 15
    if (tuning.msp_override_channels_mask != 15) {
        logStatus("msp_override_channels_mask mismatch: expected 15, got " + std::to_string(tuning.msp_override_channels_mask));
        needs_update = true;
    }
    
    // Check msp_override_failsafe = ON (assuming ON = 1)
    if (tuning.msp_override_failsafe != 1) {
        logStatus("msp_override_failsafe mismatch: expected ON(1), got " + std::to_string(tuning.msp_override_failsafe));
        needs_update = true;
    }
    
    if (needs_update) {
        logStatus("Some parameters need to be updated");
        
        // Update parameters with correct values
        tuning.thrust_linear = 0;
        tuning.roll_srate = 67;
        tuning.pitch_srate = 67;
        tuning.yaw_srate = 67;
        tuning.rates_type = 0; // ACTUAL
        tuning.roll_rc_rate = 7;
        tuning.pitch_rc_rate = 7;
        tuning.yaw_rc_rate = 7;
        tuning.msp_override_channels_mask = 15;
        tuning.msp_override_failsafe = 1; // ON
        
        // Debug: Show what we're about to send
        logStatus("DEBUG: Setting parameters to:");
        logStatus("  roll_rc_rate: " + std::to_string(tuning.roll_rc_rate));
        logStatus("  pitch_rc_rate: " + std::to_string(tuning.pitch_rc_rate));
        logStatus("  yaw_rc_rate: " + std::to_string(tuning.yaw_rc_rate));
        logStatus("  roll_srate: " + std::to_string(tuning.roll_srate));
        logStatus("  pitch_srate: " + std::to_string(tuning.pitch_srate));
        logStatus("  yaw_srate: " + std::to_string(tuning.yaw_srate));
        logStatus("  thrust_linear: " + std::to_string(tuning.thrust_linear));
        logStatus("  rates_type: " + std::to_string(tuning.rates_type));
        logStatus("  msp_override_channels_mask: " + std::to_string(tuning.msp_override_channels_mask));
        logStatus("  msp_override_failsafe: " + std::to_string(tuning.msp_override_failsafe));
        
        std::vector<uint8_t> data = MSPProtocol::encodeRCTuning(tuning);
        MSPMessage set_request(MSP_SET_RC_TUNING, data);
        MSPMessage set_response;
        
        if (!connection.sendAndReceive(set_request, set_response, 1000)) {
            logError("Failed to set RC tuning parameters");
            return false;
        }
        
        logStatus("Parameters updated successfully");
    } else {
        logStatus("All parameters are already set correctly");
    }
    
    return true;
}

bool DroneInterface::saveSettings() {
    logStatus("Saving settings to drone EEPROM...");
    
    MSPMessage request(MSP_EEPROM_WRITE);
    MSPMessage response;
    
    if (!connection.sendAndReceive(request, response, 3000)) { // Longer timeout for EEPROM write
        logError("Failed to save settings");
        return false;
    }
    
    logStatus("Settings saved successfully");
    return true;
}

bool DroneInterface::checkAndSetMSPOverrideMode() {
    logStatus("Checking MSP_OVERRIDE mode configuration...");
    
    // First try to get aux functions using MSP_AUX_FUNCTIONS
    MSPMessage aux_request(MSP_AUX_FUNCTIONS);
    MSPMessage aux_response;
    
    bool aux_functions_supported = connection.sendAndReceive(aux_request, aux_response, 2000);
    
    if (!aux_functions_supported || aux_response.data.size() == 0) {
        logStatus("DEBUG: MSP_AUX_FUNCTIONS not supported or returns no data, attempting to set AUX4 to MSP_OVERRIDE");
        
        // Even if we can't read aux functions, try to set AUX4 to MSP_OVERRIDE mode
        AuxFunction msp_override_func;
        msp_override_func.aux_id = 3;  // aux channel 4 (0-indexed)
        msp_override_func.mode = MSP_OVERRIDE_MODE;  // 50
        msp_override_func.aux_channel = 3;  // aux channel 4
        msp_override_func.range_start = 1000;
        msp_override_func.range_end = 2000;
        msp_override_func.extra1 = 0;
        msp_override_func.extra2 = 0;
        
        // Create MSP_SET_AUX_FUNCTIONS request
        std::vector<uint8_t> aux_data = MSPProtocol::encodeAuxFunctions({msp_override_func});
        
        // Debug: Show the exact data being sent
        logStatus("DEBUG: AuxFunction data to send:");
        logStatus("  aux_id: " + std::to_string(msp_override_func.aux_id));
        logStatus("  mode: " + std::to_string(msp_override_func.mode));
        logStatus("  aux_channel: " + std::to_string(msp_override_func.aux_channel));
        logStatus("  range_start: " + std::to_string(msp_override_func.range_start));
        logStatus("  range_end: " + std::to_string(msp_override_func.range_end));
        logStatus("  extra1: " + std::to_string(msp_override_func.extra1));
        logStatus("  extra2: " + std::to_string(msp_override_func.extra2));
        
        std::string hex_data = "";
        for (size_t i = 0; i < aux_data.size(); i++) {
            hex_data += std::to_string(aux_data[i]) + " ";
        }
        logStatus("DEBUG: Encoded data bytes: " + hex_data);
        
        MSPMessage set_aux_request(MSP_SET_AUX_FUNCTIONS, aux_data);
        MSPMessage set_aux_response;
        
        if (!connection.sendAndReceive(set_aux_request, set_aux_response, 2000)) {
            logError("Failed to set MSP_OVERRIDE mode on aux channel 4");
            return false;
        }
        
        logStatus("Successfully set aux channel 4 to MSP_OVERRIDE mode");
        
        // Save settings immediately after setting AUX configuration
        logStatus("Saving AUX configuration to EEPROM...");
        MSPMessage save_request(MSP_EEPROM_WRITE);
        MSPMessage save_response;
        
        if (!connection.sendAndReceive(save_request, save_response, 3000)) {
            logError("Failed to save AUX configuration to EEPROM");
            return false;
        }
        
        logStatus("AUX configuration saved to EEPROM successfully");
        
        // Verify the configuration was actually saved by reading it back
        logStatus("Verifying AUX configuration was saved correctly...");
        MSPMessage verify_request2(MSP_AUX_FUNCTIONS);
        MSPMessage verify_response2;
        
        if (connection.sendAndReceive(verify_request2, verify_response2, 2000) && verify_response2.data.size() > 0) {
            std::vector<AuxFunction> verify_functions = MSPProtocol::parseAuxFunctions(verify_response2.data);
            bool found_configured = false;
            
            for (const auto& func : verify_functions) {
                if (func.aux_id == 3 && func.mode == MSP_OVERRIDE_MODE) {
                    logStatus("VERIFIED: AUX4 is correctly configured with MSP_OVERRIDE mode");
                    found_configured = true;
                    break;
                }
            }
            
            if (!found_configured) {
                logStatus("WARNING: AUX4 MSP_OVERRIDE configuration not found in verification");
                return false;  // Return false to trigger CLI fallback
            }
        } else {
            logStatus("Could not verify AUX configuration (MSP_AUX_FUNCTIONS still not supported)");
            return false;  // Return false to trigger CLI fallback
        }
        
        return true;
    }
    
    // If we can read aux functions, check if any are set to MSP_OVERRIDE
    std::vector<AuxFunction> aux_functions = MSPProtocol::parseAuxFunctions(aux_response.data);
    
    // Check if any aux channel is set to MSP_OVERRIDE mode (50)
    bool found_msp_override = false;
    for (const auto& func : aux_functions) {
        if (func.mode == MSP_OVERRIDE_MODE) {
            logStatus("Found MSP_OVERRIDE mode on aux channel " + std::to_string(func.aux_channel));
            found_msp_override = true;
            break;
        }
    }
    
    if (!found_msp_override) {
        logStatus("No aux channel set to MSP_OVERRIDE mode, setting aux channel 4 (index 3) to MSP_OVERRIDE");
        
        // Set aux channel 4 (index 3) to MSP_OVERRIDE mode
        AuxFunction msp_override_func;
        msp_override_func.aux_id = 3;  // aux channel 4
        msp_override_func.mode = MSP_OVERRIDE_MODE;  // 50
        msp_override_func.aux_channel = 3;  // aux channel 4
        msp_override_func.range_start = 1000;
        msp_override_func.range_end = 2000;
        msp_override_func.extra1 = 0;
        msp_override_func.extra2 = 0;
        
        // Create MSP_SET_AUX_FUNCTIONS request
        std::vector<uint8_t> aux_data = MSPProtocol::encodeAuxFunctions({msp_override_func});
        
        // Debug: Show the exact data being sent
        logStatus("DEBUG: AuxFunction data to send:");
        logStatus("  aux_id: " + std::to_string(msp_override_func.aux_id));
        logStatus("  mode: " + std::to_string(msp_override_func.mode));
        logStatus("  aux_channel: " + std::to_string(msp_override_func.aux_channel));
        logStatus("  range_start: " + std::to_string(msp_override_func.range_start));
        logStatus("  range_end: " + std::to_string(msp_override_func.range_end));
        logStatus("  extra1: " + std::to_string(msp_override_func.extra1));
        logStatus("  extra2: " + std::to_string(msp_override_func.extra2));
        
        std::string hex_data = "";
        for (size_t i = 0; i < aux_data.size(); i++) {
            hex_data += std::to_string(aux_data[i]) + " ";
        }
        logStatus("DEBUG: Encoded data bytes: " + hex_data);
        
        MSPMessage set_aux_request(MSP_SET_AUX_FUNCTIONS, aux_data);
        MSPMessage set_aux_response;
        
        if (!connection.sendAndReceive(set_aux_request, set_aux_response, 2000)) {
            logError("Failed to set MSP_OVERRIDE mode on aux channel 4");
            return false;
        }
        
        logStatus("Successfully set aux channel 4 to MSP_OVERRIDE mode");
        
        // Save settings immediately after setting AUX configuration
        logStatus("Saving AUX configuration to EEPROM...");
        MSPMessage save_request2(MSP_EEPROM_WRITE);
        MSPMessage save_response2;
        
        if (!connection.sendAndReceive(save_request2, save_response2, 3000)) {
            logError("Failed to save AUX configuration to EEPROM");
            return false;
        }
        
        logStatus("AUX configuration saved to EEPROM successfully");
        
        // Verify the configuration was actually saved by reading it back
        logStatus("Verifying AUX configuration was saved correctly...");
        MSPMessage verify_request2(MSP_AUX_FUNCTIONS);
        MSPMessage verify_response2;
        
        if (connection.sendAndReceive(verify_request2, verify_response2, 2000) && verify_response2.data.size() > 0) {
            std::vector<AuxFunction> verify_functions = MSPProtocol::parseAuxFunctions(verify_response2.data);
            bool found_configured = false;
            
            for (const auto& func : verify_functions) {
                if (func.aux_id == 3 && func.mode == MSP_OVERRIDE_MODE) {
                    logStatus("VERIFIED: AUX4 is correctly configured with MSP_OVERRIDE mode");
                    found_configured = true;
                    break;
                }
            }
            
            if (!found_configured) {
                logStatus("WARNING: AUX4 MSP_OVERRIDE configuration not found in verification");
            }
        } else {
            logStatus("Could not verify AUX configuration (MSP_AUX_FUNCTIONS still not supported)");
        }
    } else {
        logStatus("MSP_OVERRIDE mode already configured on an existing aux channel");
    }
    
    return true;
}

bool DroneInterface::setAUXConfigurationViaCLI() {
    logStatus("Setting AUX4 to MSP_OVERRIDE mode via CLI commands...");
    
    // Function to send CLI command and wait for response
    auto sendCLICommand = [this](const std::string& command, const std::string& expected_response) -> bool {
        logStatus("CLI: Sending '" + command + "'");
        
        // Send command with CR
        std::string cmd_with_cr = command + "\r";
        if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(cmd_with_cr.c_str()), cmd_with_cr.length())) {
            logError("CLI: Failed to send command: " + command);
            return false;
        }
        
        // Wait for response
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Read response
        std::vector<uint8_t> response;
        if (connection.readAvailableBytes(response, 1000)) {
            std::string response_str(response.begin(), response.end());
            logStatus("CLI: Response: " + response_str);
            
            // For CLI entry, accept various responses
            if (command == "#") {
                if (response_str.find("CLI") != std::string::npos || 
                    response_str.find("Entering CLI Mode") != std::string::npos ||
                    response_str.find("#") != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            // For other commands, check for expected response
            else if (response_str.find(expected_response) != std::string::npos) {
                logStatus("CLI: ✓ Command successful");
                return true;
            } else {
                logStatus("CLI: ✗ Unexpected response");
                return false;
            }
        } else {
            logStatus("CLI: No response received");
            return false;
        }
    };
    
    // Function to exit CLI safely
    auto exitCLI = [this, &sendCLICommand]() {
        logStatus("CLI: Exiting CLI mode...");
        sendCLICommand("exit", "exit");
        // Also try sending exit command directly in case the function fails
        std::string exit_cmd = "exit\r";
        connection.sendBytes(reinterpret_cast<const uint8_t*>(exit_cmd.c_str()), exit_cmd.length());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    };
    
    // Enter CLI mode
    if (!sendCLICommand("#", "CLI")) {
        logError("Failed to enter CLI mode");
        exitCLI(); // Try to exit CLI anyway
        return false;
    }
    
    // First, get current aux configuration to find a free index
    logStatus("CLI: Checking current aux configuration...");
    std::string aux_cmd = "aux\r";
    if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(aux_cmd.c_str()), aux_cmd.length())) {
        logError("CLI: Failed to send aux command");
        exitCLI();
        return false;
    }
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Read response
    std::vector<uint8_t> response;
    int free_index = -1;
    int existing_msp_override_index = -1;
    
    if (connection.readAvailableBytes(response, 2000)) {
        std::string response_str(response.begin(), response.end());
        logStatus("CLI: Current aux configuration received");
        
        // Parse aux configuration to find free index or existing MSP_OVERRIDE
        std::istringstream iss(response_str);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("aux ") == 0) {
                int index, mode, channel;
                if (sscanf(line.c_str(), "aux %d %d %d", &index, &mode, &channel) >= 3) {
                    if (mode == 50 && channel == 3) {
                        // Found existing MSP_OVERRIDE on channel 3 (AUX4)
                        existing_msp_override_index = index;
                        logStatus("CLI: Found existing MSP_OVERRIDE on AUX4 at index " + std::to_string(index));
                        break;
                    } else if (mode == 0 && free_index == -1 && index > 0) {
                        // Found a free index (skip index 0 as it's used for arming)
                        free_index = index;
                    }
                }
            }
        }
    }
    
    // If MSP_OVERRIDE is already set on AUX4, we're done
    if (existing_msp_override_index >= 0) {
        logStatus("CLI: MSP_OVERRIDE already configured on AUX4");
        exitCLI();
        return true;
    }
    
    // Use free index if found, otherwise use index 1 (skip index 0 as it's used for arming)
    int target_index = (free_index >= 0) ? free_index : 1;
    logStatus("CLI: Using aux index " + std::to_string(target_index) + " for MSP_OVERRIDE");
    
    // Set AUX4 to MSP_OVERRIDE mode
    // Format: aux [index] [mode] [channel] [range_start] [range_end] [logic] [linked_to]
    std::string set_aux_cmd = "aux " + std::to_string(target_index) + " 50 3 1650 2100 0 0";
    logStatus("CLI: Sending '" + set_aux_cmd + "'");
    std::string set_aux_cmd_with_cr = set_aux_cmd + "\r";
    if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(set_aux_cmd_with_cr.c_str()), set_aux_cmd_with_cr.length())) {
        logError("CLI: Failed to send aux command");
        exitCLI();
        return false;
    }
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Read response
    std::vector<uint8_t> aux_response;
    if (connection.readAvailableBytes(aux_response, 1000)) {
        std::string aux_response_str(aux_response.begin(), aux_response.end());
        logStatus("CLI: Response: " + aux_response_str);
        // Accept the command echo or any response as success for aux command
        logStatus("CLI: ✓ AUX command sent");
    } else {
        logStatus("CLI: No response received for AUX command, continuing anyway");
    }
    
    // Don't save here - we'll save after setting all parameters
    // Just exit CLI for now
    
    // Exit CLI
    if (!sendCLICommand("exit", "exit")) {
        logError("Failed to exit CLI");
        exitCLI(); // Try alternative exit method
        return false;
    }
    
    logStatus("AUX4 configuration via CLI completed successfully");
    return true;
}

bool DroneInterface::setParametersViaCLI() {
    logStatus("Checking and setting parameters via CLI commands...");
    
    // Function to send CLI command and wait for response with shorter timeout
    auto sendCLICommand = [this](const std::string& command, const std::string& expected_response) -> bool {
        logStatus("CLI: Sending '" + command + "'");
        
        // Send command with CR
        std::string cmd_with_cr = command + "\r";
        if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(cmd_with_cr.c_str()), cmd_with_cr.length())) {
            logError("CLI: Failed to send command: " + command);
            return false;
        }
        
        // Wait for response (shorter timeout)
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Read response (shorter timeout)
        std::vector<uint8_t> response;
        if (connection.readAvailableBytes(response, 1000)) {
            std::string response_str(response.begin(), response.end());
            logStatus("CLI: Response: '" + response_str + "'");
            
            // For CLI entry, accept empty response or CLI prompt
            if (command == "#") {
                if (response_str.empty() || response_str.find("CLI") != std::string::npos || 
                    response_str.find("#") != std::string::npos || response_str.find("\n") != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            // For save command, accept multiple valid responses
            else if (command == "save") {
                if (response_str.find("saving") != std::string::npos || 
                    response_str.find("saved") != std::string::npos ||
                    response_str.find("Rebooting") != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            // For set commands, accept newline or the expected response
            else if (command.find("set ") == 0) {
                if (response_str == "\n" || response_str == "\r\n" || 
                    response_str.find(expected_response) != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            } else {
                // For other commands, use exact expected response
                if (response_str.find(expected_response) != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            
            logStatus("CLI: ✗ Unexpected response");
            return false;
        } else {
            // For CLI entry, accept no response as success
            if (command == "#") {
                logStatus("CLI: ✓ Command successful (no response)");
                return true;
            }
            logStatus("CLI: No response received");
            return false;
        }
    };
    
    // Function to get parameter value via CLI
    auto getCLIParameter = [this, &sendCLICommand](const std::string& param_name) -> std::string {
        std::string command = "get " + param_name;
        logStatus("CLI: Getting '" + param_name + "'");
        
        // Send command with CR
        std::string cmd_with_cr = command + "\r";
        if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(cmd_with_cr.c_str()), cmd_with_cr.length())) {
            logError("CLI: Failed to send get command: " + command);
            return "";
        }
        
        // Wait for response (longer timeout for get commands)
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Read response (longer timeout)
        std::vector<uint8_t> response;
        if (connection.readAvailableBytes(response, 1000)) {
            std::string response_str(response.begin(), response.end());
            logStatus("CLI: Get response: " + response_str);
            
            // Skip the "Entering CLI Mode" message if present
            if (response_str.find("Entering CLI Mode") != std::string::npos) {
                return "";
            }
            
            // Look for the parameter value in the response
            // Response format is usually "parameter_name = value\n#" 
            size_t pos = response_str.find(param_name + " = ");
            if (pos != std::string::npos) {
                size_t start = pos + param_name.length() + 3; // Skip "param_name = "
                size_t end = response_str.find_first_of("\r\n#", start);
                if (end != std::string::npos) {
                    std::string value = response_str.substr(start, end - start);
                    // Trim whitespace
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);
                    return value;
                }
            }
        }
        return "";
    };
    
    // Function to exit CLI safely
    auto exitCLI = [this, &sendCLICommand]() {
        logStatus("CLI: Exiting CLI mode...");
        sendCLICommand("exit", "exit");
        // Also try sending exit command directly in case the function fails
        std::string exit_cmd = "exit\r";
        connection.sendBytes(reinterpret_cast<const uint8_t*>(exit_cmd.c_str()), exit_cmd.length());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    };
    
    // Enter CLI mode
    if (!sendCLICommand("#", "CLI")) {
        logError("Failed to enter CLI mode");
        exitCLI(); // Try to exit CLI anyway
        return false;
    }
    
    // Check and set parameters only if they're incorrect
    logStatus("Checking current parameter values...");
    
    // Define required parameters and their expected values
    std::vector<std::pair<std::string, std::string>> required_params = {
        {"thrust_linear", "0"},
        {"roll_srate", "67"},
        {"pitch_srate", "67"},
        {"yaw_srate", "67"},
        {"rates_type", "ACTUAL"},  // Use ACTUAL instead of 0
        {"roll_rc_rate", "7"},
        {"pitch_rc_rate", "7"},
        {"yaw_rc_rate", "7"},
        {"msp_override_channels_mask", "15"},
        {"msp_override_failsafe", "ON"}  // Use ON instead of 1
    };
    
    bool need_save = false;
    
    for (const auto& param : required_params) {
        std::string current_value = getCLIParameter(param.first);
        if (current_value.empty()) {
            logStatus("CLI: Could not get current value for " + param.first + ", setting it");
            if (!sendCLICommand("set " + param.first + " = " + param.second, param.first + " set to " + param.second)) {
                logError("Failed to set " + param.first);
                exitCLI(); // Exit CLI before returning
                return false;
            }
            need_save = true;
        } else if (current_value != param.second) {
            logStatus("CLI: " + param.first + " mismatch: current=" + current_value + ", expected=" + param.second);
            if (!sendCLICommand("set " + param.first + " = " + param.second, param.first + " set to " + param.second)) {
                logError("Failed to set " + param.first);
                exitCLI(); // Exit CLI before returning
                return false;
            }
            need_save = true;
        } else {
            logStatus("CLI: " + param.first + " already correct: " + current_value);
        }
    }
    
    // Only save if we made changes
    if (need_save) {
        logStatus("CLI: Saving changes to EEPROM...");
        if (!sendCLICommand("save", "saving")) {
            logError("Failed to save settings");
            exitCLI(); // Exit CLI before returning
            return false;
        }
    } else {
        logStatus("CLI: All parameters already correct, no save needed");
    }
    
    // Exit CLI
    if (!sendCLICommand("exit", "exit")) {
        logError("Failed to exit CLI");
        exitCLI(); // Try alternative exit method
        return false;
    }
    
    logStatus("Parameter check and set via CLI completed successfully");
    return true;
}

bool DroneInterface::setAUXAndParametersViaCLI() {
    logStatus("Setting AUX and parameters via CLI commands...");
    
    // Function to send CLI command and wait for response
    auto sendCLICommand = [this](const std::string& command, const std::string& expected_response) -> bool {
        logStatus("CLI: Sending '" + command + "'");
        
        // Send command with CR
        std::string cmd_with_cr = command + "\r";
        if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(cmd_with_cr.c_str()), cmd_with_cr.length())) {
            logError("CLI: Failed to send command: " + command);
            return false;
        }
        
        // Wait for response
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Read response
        std::vector<uint8_t> response;
        if (connection.readAvailableBytes(response, 1000)) {
            std::string response_str(response.begin(), response.end());
            logStatus("CLI: Response: " + response_str);
            
            // For CLI entry, accept various responses
            if (command == "#") {
                if (response_str.find("CLI") != std::string::npos || 
                    response_str.find("Entering CLI Mode") != std::string::npos ||
                    response_str.find("#") != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            // For save command, accept multiple valid responses
            else if (command == "save") {
                if (response_str.find("saving") != std::string::npos || 
                    response_str.find("saved") != std::string::npos ||
                    response_str.find("Rebooting") != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            // For set commands, accept newline or the expected response
            else if (command.find("set ") == 0) {
                if (response_str == "\n" || response_str == "\r\n" || 
                    response_str.find(expected_response) != std::string::npos) {
                    logStatus("CLI: ✓ Command successful");
                    return true;
                }
            }
            // For get commands, return true if we got any response
            else if (command.find("get ") == 0) {
                return true;
            }
            // For aux command, accept echo or any response
            else if (command.find("aux ") == 0) {
                logStatus("CLI: ✓ Command successful");
                return true;
            }
            // For other commands, check for expected response
            else if (response_str.find(expected_response) != std::string::npos) {
                logStatus("CLI: ✓ Command successful");
                return true;
            } else {
                logStatus("CLI: ✗ Unexpected response");
                return false;
            }
        } else {
            logStatus("CLI: No response received");
            return false;
        }
    };
    
    // Function to get parameter value via CLI
    auto getCLIParameter = [this, &sendCLICommand](const std::string& param_name) -> std::string {
        std::string command = "get " + param_name;
        logStatus("CLI: Getting '" + param_name + "'");
        
        // Send command with CR
        std::string cmd_with_cr = command + "\r";
        if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(cmd_with_cr.c_str()), cmd_with_cr.length())) {
            logError("CLI: Failed to send get command: " + command);
            return "";
        }
        
        // Wait for response
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Read response
        std::vector<uint8_t> response;
        if (connection.readAvailableBytes(response, 1000)) {
            std::string response_str(response.begin(), response.end());
            logStatus("CLI: Get response: " + response_str);
            
            // Look for the parameter value in the response
            size_t pos = response_str.find(param_name + " = ");
            if (pos != std::string::npos) {
                size_t start = pos + param_name.length() + 3;
                size_t end = response_str.find_first_of("\r\n#", start);
                if (end != std::string::npos) {
                    std::string value = response_str.substr(start, end - start);
                    // Trim whitespace
                    value.erase(0, value.find_first_not_of(" \t"));
                    value.erase(value.find_last_not_of(" \t") + 1);
                    return value;
                }
            }
        }
        return "";
    };
    
    // Function to exit CLI safely
    auto exitCLI = [this, &sendCLICommand]() {
        logStatus("CLI: Exiting CLI mode...");
        sendCLICommand("exit", "exit");
        // Also try sending exit command directly in case the function fails
        std::string exit_cmd = "exit\r";
        connection.sendBytes(reinterpret_cast<const uint8_t*>(exit_cmd.c_str()), exit_cmd.length());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    };
    
    // Enter CLI mode
    if (!sendCLICommand("#", "CLI")) {
        logError("Failed to enter CLI mode");
        exitCLI();
        return false;
    }
    
    bool need_save = false;
    
    // STEP 1: Check and set AUX configuration
    logStatus("CLI: Checking current aux configuration...");
    std::string aux_cmd = "aux\r";
    if (!connection.sendBytes(reinterpret_cast<const uint8_t*>(aux_cmd.c_str()), aux_cmd.length())) {
        logError("CLI: Failed to send aux command");
        exitCLI();
        return false;
    }
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Read response
    std::vector<uint8_t> response;
    int free_index = -1;
    int existing_msp_override_index = -1;
    int existing_msp_override_channel = -1;  // Store the actual AUX channel number, not index
    
    if (connection.readAvailableBytes(response, 2000)) {
        std::string response_str(response.begin(), response.end());
        logStatus("CLI: Current aux configuration received");
        
        // Parse aux configuration
        std::istringstream iss(response_str);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("aux ") == 0) {
                int index, mode, channel;
                if (sscanf(line.c_str(), "aux %d %d %d", &index, &mode, &channel) >= 3) {
                    if (mode == 50) {  // Any AUX set to MSP_OVERRIDE
                        existing_msp_override_channel = channel;  // Store the actual AUX channel number
                        existing_msp_override_index = index;
                        logStatus("CLI: Found existing MSP_OVERRIDE on AUX" + std::to_string(channel) + " at index " + std::to_string(index));
                        break;
                    } else if (mode == 0 && free_index == -1 && index > 0) {
                        // Skip index 0 (AUX0) as it's used for arming, find next free index
                        free_index = index;
                    }
                }
            }
        }
    }
    
    // If MSP_OVERRIDE is not already set on AUX4, set it
    if (existing_msp_override_index < 0) {
        int target_index = (free_index >= 0) ? free_index : 1;  // Use index 1 as fallback instead of 3
        logStatus("CLI: Using aux index " + std::to_string(target_index) + " for MSP_OVERRIDE");
        
        std::string set_aux_cmd = "aux " + std::to_string(target_index) + " 50 3 1650 2100 0 0";
        if (!sendCLICommand(set_aux_cmd, set_aux_cmd)) {
            logError("Failed to set AUX configuration");
            exitCLI();
            return false;
        }
        need_save = true;
        
        // Save the channel configuration - AUX channel number to RC channel mapping
        int rc_channel = 3 + 4;  // AUX4 (channel 3) + 4 = RC channel 7
        config.setMSPOverrideChannel(rc_channel);
        if (!config.saveConfig()) {
            logError("Failed to save MSP override channel to config");
            exitCLI();
            return false;
        }
        logStatus("Saved MSP override channel: AUX4 (channel 3) -> RC channel " + std::to_string(rc_channel));
    } else {
        logStatus("CLI: MSP_OVERRIDE already configured on AUX" + std::to_string(existing_msp_override_channel));
        
        // Save the existing channel configuration - AUX channel number to RC channel mapping
        int rc_channel = existing_msp_override_channel + 4;  // AUX channel + 4 = RC channel
        config.setMSPOverrideChannel(rc_channel);
        if (!config.saveConfig()) {
            logError("Failed to save existing MSP override channel to config");
            exitCLI();
            return false;
        }
        logStatus("Saved existing MSP override channel: AUX" + std::to_string(existing_msp_override_channel) + " -> RC channel " + std::to_string(rc_channel));
    }
    
    // STEP 2: Check and set parameters
    logStatus("CLI: Checking current parameter values...");
    
    std::vector<std::pair<std::string, std::string>> required_params = {
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
    
    for (const auto& param : required_params) {
        std::string current_value = getCLIParameter(param.first);
        if (current_value.empty()) {
            logStatus("CLI: Could not get current value for " + param.first + ", setting it");
            if (!sendCLICommand("set " + param.first + " = " + param.second, param.first + " set to " + param.second)) {
                logError("Failed to set " + param.first);
                exitCLI();
                return false;
            }
            need_save = true;
        } else if (current_value != param.second) {
            logStatus("CLI: " + param.first + " mismatch: current=" + current_value + ", expected=" + param.second);
            if (!sendCLICommand("set " + param.first + " = " + param.second, param.first + " set to " + param.second)) {
                logError("Failed to set " + param.first);
                exitCLI();
                return false;
            }
            need_save = true;
        } else {
            logStatus("CLI: " + param.first + " already correct: " + current_value);
        }
    }
    
    // STEP 3: Save if we made any changes
    if (need_save) {
        logStatus("CLI: Saving all changes to EEPROM...");
        if (!sendCLICommand("save", "saving")) {
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

bool DroneInterface::checkFirmwareVersion() {
    logStatus("Checking firmware version...");
    
    MSPMessage ident_request(MSP_IDENT);
    MSPMessage ident_response;
    
    if (!connection.sendAndReceive(ident_request, ident_response, 2000)) {
        logError("Failed to get firmware identification - MSP_IDENT command not supported");
        
        // Since MSP_IDENT failed, we can't determine the firmware version
        // Don't write error file - just continue
        logStatus("DEBUG: Continuing with configuration - unable to determine firmware version");
        return true;
    }
    
    if (ident_response.data.size() == 0) {
        logStatus("DEBUG: No firmware identification data received, but drone is responding to MSP");
        logStatus("DEBUG: Continuing with configuration - drone appears to be compatible");
        
        // The drone is responding to MSP commands but not providing firmware data
        // This could be a different firmware variant that doesn't support MSP_IDENT properly
        // Let's continue with the configuration since the drone is communicating
        return true;
    }
    
    // Parse firmware identification data
    // Format: version, multitype, msp_version, capability, aux_count, aux_sequence, aux_sequence2
    if (ident_response.data.size() >= 7) {
        uint8_t version = ident_response.data[0];
        uint8_t multitype = ident_response.data[1];
        uint8_t msp_version = ident_response.data[2];
        
        logStatus("DEBUG: Firmware version: " + std::to_string(version) + 
                  ", multitype: " + std::to_string(multitype) + 
                  ", MSP version: " + std::to_string(msp_version));
        
        // Check if version is 4.5.x (version should be 45 for 4.5.x)
        if (version != 45) {
            logError("Incompatible firmware version: " + std::to_string(version) + " (expected 45 for 4.5.x)");
            
            // Write error to /tmp/error.txt only when we can determine the version and it's wrong
            std::ofstream error_file("/tmp/error.txt");
            if (error_file.is_open()) {
                error_file << "Incompatible betaflight version" << std::endl;
                error_file.close();
                logStatus("Error written to /tmp/error.txt");
            } else {
                logError("Failed to write error to /tmp/error.txt");
            }
            
            return false;
        }
        
        logStatus("Firmware version compatible: 4.5.x");
    } else {
        logStatus("DEBUG: Insufficient firmware identification data");
        
        // Don't write error file - just continue since we can't determine version
        logStatus("DEBUG: Continuing with configuration - insufficient firmware data");
        return true;
    }
    
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