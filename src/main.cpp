#include "drone_interface.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [OPTIONS]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --skip-update    MSP-only: read MSP_OVERRIDE AUX and save to config.ini (no CLI, no changes)" << std::endl;
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
    
    std::cout << "=== Drone MSP/CLI Tool ===" << std::endl;
    if (skipUpdate) {
        std::cout << "Mode: MSP-only read of MSP_OVERRIDE (no CLI, no updates)" << std::endl;
    } else {
        std::cout << "Mode: CLI configuration (AUX + parameters)" << std::endl;
    }
    
    // Initialize drone interface
    DroneInterface drone("/dev/serial0");
    
    if (!drone.initialize()) {
        std::cerr << "Failed to initialize drone interface" << std::endl;
        return 1;
    }
    
    try {
        if (skipUpdate) {
            // MSP-only: do not enter CLI or perform firmware check (to avoid reboots)
            bool msp_ok = drone.readMSPOverrideChannelViaMSP();
            (void)msp_ok; // even if false, config.ini already has -1 saved inside the function
            std::cout << "\n=== Done ===" << std::endl;
            std::cout << "✓ MSP-only read complete. Written to /tmp/config.ini (msp_override_channel or -1)" << std::endl;
        } else {
            // Step 0: Check firmware version (uses CLI)
            std::cout << "\n--- Step 0: Checking Firmware Version ---" << std::endl;
            if (!drone.checkFirmwareVersion()) {
                std::cerr << "Firmware version check failed" << std::endl;
                return 1;
            }
            
            // Use CLI method to set both AUX and parameters in one session
            std::cout << "\n--- Step 1: Setting AUX and Parameters via CLI ---" << std::endl;
            if (!drone.setAUXAndParametersViaCLI()) {
                std::cerr << "Failed to set AUX and parameters via CLI" << std::endl;
                return 1;
            }
            
            std::cout << "\n=== Configuration Complete ===" << std::endl;
            std::cout << "✓ Parameters verified and updated via CLI" << std::endl;
            std::cout << "✓ Settings saved to drone EEPROM via CLI" << std::endl;
            std::cout << "✓ Configuration saved to /tmp/config.ini" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}