#include "drone_interface.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [OPTIONS]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --skip-update    Skip parameter updates, only configure AUX channels" << std::endl;
    std::cout << "  -h, --help       Show this help message" << std::endl;
}

bool parseArguments(int argc, char* argv[], bool& skipUpdate) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--skip-update") {
            skipUpdate = true;
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    bool skipUpdate = false;
    
    if (!parseArguments(argc, argv, skipUpdate)) {
        return 1;
    }
    
    std::cout << "=== Drone MSP Configuration Tool ===" << std::endl;
    if (skipUpdate) {
        std::cout << "Mode: AUX channel configuration only (skipping parameter updates)" << std::endl;
    }
    std::cout << "Connecting to drone via MSP on /dev/serial0" << std::endl;
    
    // Initialize drone interface
    DroneInterface drone("/dev/serial0");
    
    if (!drone.initialize()) {
        std::cerr << "Failed to initialize drone interface" << std::endl;
        return 1;
    }
    
    // Step 0: Check firmware version
    std::cout << "\n--- Step 0: Checking Firmware Version ---" << std::endl;
    if (!drone.checkFirmwareVersion()) {
        std::cerr << "Firmware version check failed" << std::endl;
        return 1;
    }
    
    try {
        // Step 1: Find MSP Override Channel
        std::cout << "\n--- Step 1: Finding MSP Override Channel ---" << std::endl;
        int msp_override_channel = drone.findMSPOverrideChannel();
        
        if (msp_override_channel == -1) {
            std::cout << "No MSP Override channel found, setting AUX4 (channel 8) as default" << std::endl;
            msp_override_channel = 7; // AUX4 = RC channel 8 (1-indexed), so 7 (0-indexed)
        }
        
        // Save the channel configuration
        if (!drone.saveMSPOverrideChannel(msp_override_channel)) {
            std::cerr << "Failed to save MSP Override channel configuration" << std::endl;
            return 1;
        }
        
        if (skipUpdate) {
            // Only configure AUX channels, skip parameter updates
            std::cout << "\n--- Step 1.5: Configuring AUX Channels Only ---" << std::endl;
            if (!drone.setAUXConfigurationViaCLI()) {
                std::cerr << "Failed to configure AUX channels" << std::endl;
                return 1;
            }
        } else {
            // Step 1.5: Check and set MSP_OVERRIDE mode
            std::cout << "\n--- Step 1.5: Checking MSP_OVERRIDE Mode Configuration ---" << std::endl;
            bool used_cli_method = false;
            if (!drone.checkAndSetMSPOverrideMode()) {
                std::cout << "MSP commands failed to set AUX configuration, trying CLI method..." << std::endl;
                // Use combined function to set both AUX and parameters in one CLI session
                if (!drone.setAUXAndParametersViaCLI()) {
                    std::cerr << "Failed to set AUX and parameters via CLI" << std::endl;
                    return 1;
                }
                used_cli_method = true;
            } else {
                // If MSP worked for AUX, still need to set parameters via CLI
                std::cout << "\n--- Step 2: Setting Parameters via CLI ---" << std::endl;
                if (!drone.setParametersViaCLI()) {
                    std::cerr << "Failed to set parameters via CLI" << std::endl;
                    return 1;
                }
            }
            
            // Step 3: Save settings to drone (if not already saved by CLI)
            if (!used_cli_method) {
                std::cout << "\n--- Step 3: Saving Settings ---" << std::endl;
                if (!drone.saveSettings()) {
                    std::cerr << "Failed to save settings to drone" << std::endl;
                    return 1;
                }
            }
        }
        
        std::cout << "\n=== Configuration Complete ===" << std::endl;
        std::cout << "✓ MSP Override channel found and saved" << std::endl;
        if (skipUpdate) {
            std::cout << "✓ AUX channels configured (parameter updates skipped)" << std::endl;
        } else {
            std::cout << "✓ Parameters verified and updated" << std::endl;
            std::cout << "✓ Settings saved to drone EEPROM" << std::endl;
        }
        std::cout << "✓ Configuration saved to /tmp/config.ini" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}