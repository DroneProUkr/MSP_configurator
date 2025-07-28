#include "config_manager.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

ConfigManager::ConfigManager(const std::string& path) : config_path(path) {
}

bool ConfigManager::loadConfig() {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        std::cout << "Config file not found, will create new one" << std::endl;
        return true; // Not an error, we'll create it
    }
    
    std::string line;
    while (std::getline(file, line)) {
        parseLine(line);
    }
    
    file.close();
    std::cout << "Loaded config from " << config_path << std::endl;
    return true;
}

bool ConfigManager::saveConfig() {
    std::ofstream file(config_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file for writing: " << config_path << std::endl;
        return false;
    }
    
    file << "[drone]" << std::endl;
    for (const auto& pair : config_data) {
        file << pair.first << "=" << pair.second << std::endl;
    }
    
    file.close();
    std::cout << "Saved config to " << config_path << std::endl;
    return true;
}

void ConfigManager::setValue(const std::string& key, const std::string& value) {
    config_data[key] = value;
}

void ConfigManager::setValue(const std::string& key, int value) {
    config_data[key] = std::to_string(value);
}

std::string ConfigManager::getString(const std::string& key, const std::string& default_value) {
    auto it = config_data.find(key);
    return (it != config_data.end()) ? it->second : default_value;
}

int ConfigManager::getInt(const std::string& key, int default_value) {
    auto it = config_data.find(key);
    if (it != config_data.end()) {
        try {
            return std::stoi(it->second);
        } catch (const std::exception&) {
            return default_value;
        }
    }
    return default_value;
}

void ConfigManager::setMSPOverrideChannel(int channel) {
    setValue("msp_override_channel", channel);
}

int ConfigManager::getMSPOverrideChannel() {
    return getInt("msp_override_channel", -1);
}

void ConfigManager::parseLine(const std::string& line) {
    std::string trimmed = trim(line);
    
    // Skip empty lines and comments
    if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == '[') {
        return;
    }
    
    size_t equals_pos = trimmed.find('=');
    if (equals_pos != std::string::npos) {
        std::string key = trim(trimmed.substr(0, equals_pos));
        std::string value = trim(trimmed.substr(equals_pos + 1));
        config_data[key] = value;
    }
}

std::string ConfigManager::trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}