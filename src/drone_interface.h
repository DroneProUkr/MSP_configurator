#pragma once

#include "msp_connection.h"
#include "config_manager.h"
#include <vector>

struct DroneParameters {
    int thrust_linear = 0;
    int roll_srate = 67;
    int pitch_srate = 67;
    int yaw_srate = 67;
    std::string rates_type = "ACTUAL";
    int roll_rc_rate = 7;
    int pitch_rc_rate = 7;
    int yaw_rc_rate = 7;
    int msp_override_channels_mask = 15;
    std::string msp_override_failsafe = "ON";
};

class DroneInterface {
private:
    MSPConnection connection;
    ConfigManager config;
    DroneParameters expected_params;
    
public:
    DroneInterface(const std::string& device = "/dev/serial0");
    ~DroneInterface();
    
    bool initialize();
    bool shutdown();
    
    // Aux channel operations
    bool getAuxChannels(std::vector<AuxFunction>& aux_functions);
    int findMSPOverrideChannel();
    bool saveMSPOverrideChannel(int aux_channel_index);
    
    // Parameter operations
    bool verifyParameters();
    bool getCurrentParameters(DroneParameters& params);
    bool setParameters(const DroneParameters& params);
    bool saveSettings();
    bool checkAndSetMSPOverrideMode();
    bool setAUXConfigurationViaCLI();
    bool setParametersViaCLI();
    bool setAUXAndParametersViaCLI();  // New combined method
    bool checkFirmwareVersion();
    
    // Logging
    void logStatus(const std::string& message);
    void logError(const std::string& message);
    
private:
    bool getRCTuning(RCTuning& tuning);
    bool setRCTuning(const RCTuning& tuning);
};