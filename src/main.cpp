#include "drone_interface.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "=== Drone MSP Configuration Tool ===" << std::endl;
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
            std::cerr << "Could not find MSP Override channel" << std::endl;
            return 1;
        }
        
        // Save the channel configuration
        if (!drone.saveMSPOverrideChannel(msp_override_channel)) {
            std::cerr << "Failed to save MSP Override channel configuration" << std::endl;
            return 1;
        }
        
        // Step 1.5: Check and set MSP_OVERRIDE mode
        std::cout << "\n--- Step 1.5: Checking MSP_OVERRIDE Mode Configuration ---" << std::endl;
        if (!drone.checkAndSetMSPOverrideMode()) {
            std::cerr << "Failed to check/set MSP_OVERRIDE mode" << std::endl;
            return 1;
        }
        
        // Step 2: Verify and set parameters
        std::cout << "\n--- Step 2: Verifying Parameters ---" << std::endl;
        if (!drone.verifyParameters()) {
            std::cerr << "Parameter verification/setting failed" << std::endl;
            return 1;
        }
        
        // Step 3: Save settings to drone
        std::cout << "\n--- Step 3: Saving Settings ---" << std::endl;
        if (!drone.saveSettings()) {
            std::cerr << "Failed to save settings to drone" << std::endl;
            return 1;
        }
        
        std::cout << "\n=== Configuration Complete ===" << std::endl;
        std::cout << "✓ MSP Override channel found and saved" << std::endl;
        std::cout << "✓ Parameters verified and updated" << std::endl;
        std::cout << "✓ Settings saved to drone EEPROM" << std::endl;
        std::cout << "✓ Configuration saved to /tmp/config.ini" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}