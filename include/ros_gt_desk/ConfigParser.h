#pragma once
#include <cstdint>
#include <iostream>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>


bool isConvertibleToInt(const std::string& str) {
    if (str.empty()) {
        return false; // Empty string cannot be converted
    }

    // Remove leading and trailing whitespace
    std::string trimmedStr = str;
    trimmedStr.erase(0, trimmedStr.find_first_not_of(" \t\n\r\f\v"));
    trimmedStr.erase(trimmedStr.find_last_not_of(" \t\n\r\f\v") + 1);

    if (trimmedStr.empty()) {
        return false; // String containing only whitespace cannot be converted
    }

    // Check for optional sign
    size_t start = 0;
    if (trimmedStr[0] == '+' || trimmedStr[0] == '-') {
        start = 1;
    }

    // Check if the remaining characters are digits
    if (start == trimmedStr.length()) {
        return false; // Only a sign, no digits
    }

    for (size_t i = start; i < trimmedStr.length(); ++i) {
        if (!std::isdigit(trimmedStr[i])) {
            return false; // Contains non-digit characters
        }
    }

    // Check for overflow/underflow (using std::stoi and catching exceptions)
    try {
        std::stoi(trimmedStr);
        return true;
    } catch (const std::invalid_argument& e) {
        return false; // Not a valid integer format
    } catch (const std::out_of_range& e) {
        return false; // Integer is out of range
    }
}



class ConfigParser {
public:
    explicit ConfigParser(const std::string& path) {
        std::ifstream f(path);
        if(!f.is_open()) throw std::runtime_error("Cannot open config file");
        try{
            data_ = nlohmann::json::parse(f);
        }catch(const nlohmann::json::parse_error& e){
            std::cerr << "parse error:"<< e.what() << std::endl;
        }
    }

    std::string getHost() const {
        return data_["host"];
    }

    std::int16_t getPort() const {
        return data_["port"];
    }

    uint16_t getRegister(const std::string& name) const {
        return data_["registers"][name];
    }

    std::int16_t getPollingRate() const {
        return data_["polling_rate"];
    }

    /** Get json value from keys
     * 
     * @param path_list {key1, key2, ..., keyn}
     * @return json_data[key1][...][keyn]
     * */ 
    template<typename T>
    T getValueByPath(const std::vector<std::string>& path_list) const {
    
        const nlohmann::json* current = &data_;

        for (const std::string& key : path_list) {
            // ROS_INFO(current.c_str());
            
            // If Key is a int, means &(*current) is a list not map.
            // So use int index to get the value.
            if (isConvertibleToInt(key))
            {
                int key_int = std::stoi(key);
                current = &(*current)[key_int];
            }else
            {
                if (!current->is_object() || !current->contains(key)) {
                    throw std::runtime_error("Invalid JSON path "+std::string(key));
                }
                current = &(*current)[key];
            }
        }

        return current->get<T>(); 
    }

    /** Get json value from keys string
     * 
     * @param path "key1.key2.keyn"
     * @return json_data[key1][...][keyn]
     * */ 
    template<typename T>
    T getValueByPath(const std::string& path) const {
        // ROS_INFO("%s",path.c_str());
        std::vector<std::string> path_list; 
        bool hasDot = (path.find('.') != std::string::npos);
        if (hasDot)
            path_list =this->splitPath(path);
        else
            path_list = std::vector<std::string> {path};

        const nlohmann::json* current = &data_;

        for (const std::string& key : path_list) {
            // ROS_INFO(current.c_str());
            
            // If Key is a int, means &(*current) is a list not map.
            // So use int index to get the value.
            if (isConvertibleToInt(key))
            {
                int key_int = std::stoi(key);
                current = &(*current)[key_int];
            }else
            {
                if (!current->is_object() || !current->contains(key)) {
                    throw std::runtime_error("Invalid JSON path "+std::string(key));
                }
                current = &(*current)[key];
            }
        }

        return current->get<T>(); 
    }

private:
    nlohmann::json data_;
    std::vector<std::string> splitPath(const std::string& dotted) const {
        
        std::vector<std::string> result;
        std::stringstream ss(dotted);
        std::string token;
        while (std::getline(ss, token, '.')) {
            result.push_back(token);
        }
        return result;
    }
};