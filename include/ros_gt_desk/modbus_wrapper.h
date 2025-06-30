#pragma once
// #include "src/modbus_wrapper.cpp"
#include <cstdint>
#include <modbus/modbus.h>
#include <string>
// #include <stdexcept>  //捕获异常头文件

// modbus C++接口主要提供几个接口：连接、断开连接、连接状态检测、读、写寄存器

class ModbusWrapper {
public:
    ModbusWrapper(const std::string& host, int port);
    ~ModbusWrapper();

    void connect();
    void disconnect();
    bool isConnected() const;

    uint32_t readRegister(int reg_addr);
    void writeRegister(int reg_addr, uint32_t value);

private:
    modbus_t* ctx_;
    std::string host_;
    int port_;
    bool connected_;
};