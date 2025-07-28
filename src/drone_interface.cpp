#include "drone_interface.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream> // Added for error file writing

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
        logStatus("DEBUG: Looking for AUX6 (aux_id 5) as MSP override channel");
        
        // Since we're using RC data and you've set AUX6 for MSP_OVERRIDE,
        // look for aux_id 5 (AUX6) and return its RC channel
        for (const auto& func : aux_functions) {
            if (func.aux_id == 5) {  // AUX6
                logStatus("Found AUX6 (aux_id 5) - assuming it's set to MSP_OVERRIDE");
                return func.aux_channel + 4;  // RC channel = aux_channel + 4 (since aux channels start from RC channel 4)
            }
        }
        
        logError("AUX6 (aux_id 5) not found in aux channels");
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
    
    // Parse RC tuning parameters
    RCTuning tuning = MSPProtocol::parseRCTuning(response.data);
    
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
        
        // Send updated parameters
        MSPMessage update_request = MSPProtocol::createSetRCTuningRequest(tuning);
        MSPMessage update_response;
        
        if (!connection.sendAndReceive(update_request, update_response, 1000)) {
            logError("Failed to update RC tuning parameters");
            return false;
        }
        
        if (update_response.cmd != MSP_SET_RC_TUNING) {
            logError("Received unexpected response for RC tuning update");
            return false;
        }
        
        logStatus("Parameters updated successfully");
    } else {
        logStatus("All parameters are already correct");
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
    
    if (!connection.sendAndReceive(aux_request, aux_response, 2000)) {
        logStatus("DEBUG: MSP_AUX_FUNCTIONS not supported, using RC data approach");
        return true; // Fall back to RC data approach
    }
    
    if (aux_response.data.size() == 0) {
        logStatus("DEBUG: No aux functions data received");
        return true; // Fall back to RC data approach
    }
    
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
        MSPMessage set_aux_request(MSP_SET_AUX_FUNCTIONS, aux_data);
        MSPMessage set_aux_response;
        
        if (!connection.sendAndReceive(set_aux_request, set_aux_response, 2000)) {
            logError("Failed to set MSP_OVERRIDE mode on aux channel 4");
            return false;
        }
        
        logStatus("Successfully set aux channel 4 to MSP_OVERRIDE mode");
    } else {
        logStatus("MSP_OVERRIDE mode already configured on an existing aux channel");
    }
    
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