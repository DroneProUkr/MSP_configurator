#pragma once

#include <string>
#include <map>

class ConfigManager {
private:
    std::string config_path;
    std::map<std::string, std::string> config_data;
    
public:
    ConfigManager(const std::string& path = "/tmp/config.ini");
    
    bool loadConfig();
    bool saveConfig();
    
    void setValue(const std::string& key, const std::string& value);
    void setValue(const std::string& key, int value);
    
    std::string getString(const std::string& key, const std::string& default_value = "");
    int getInt(const std::string& key, int default_value = 0);
    
    void setMSPOverrideChannel(int channel);
    int getMSPOverrideChannel();
    
private:
    void parseLine(const std::string& line);
    std::string trim(const std::string& str);
};