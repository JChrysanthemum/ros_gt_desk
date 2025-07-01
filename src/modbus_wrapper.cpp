
#include <cstdint>
#include <iostream>
#include <ros/ros.h>
#include <sys/types.h>
// #include "ros_gt_msg/gt_motion.h"
#include "modbus_wrapper.h"
#include "modbus.h"


ModbusWrapper::ModbusWrapper(const std::string& host, int port) 
    : host_(host), port_(port), connected_(false) {
    ctx_ = modbus_new_tcp(host_.c_str(), port_); 
    if (!ctx_) {
        throw std::runtime_error("Failed to create Modbus context");
    }
    modbus_set_response_timeout(ctx_, 1, 0); // 1秒超时
}

ModbusWrapper::~ModbusWrapper() {
    disconnect();
    if (ctx_) {
        modbus_free(ctx_);
    }
}

void ModbusWrapper::connect() {
    if (modbus_connect(ctx_) == -1) {
        ROS_ERROR("Modbus connection failed: %s", modbus_strerror(errno));
        connected_ = false;
    } else {
        connected_ = true;
        ROS_INFO("Connected to Modbus device at %s:%d", host_.c_str(), port_);
    }
}

void ModbusWrapper::disconnect() {
    if (connected_) {
        modbus_close(ctx_);
        connected_ = false;
    }
}

bool ModbusWrapper::isConnected() const {
    return connected_;
}

uint32_t ModbusWrapper::readRegister(int reg_addr) {
    if (!connected_) throw std::runtime_error("Modbus not connected");

    uint16_t regs[2];
    int rc = modbus_read_registers(ctx_, reg_addr, 2, regs);
    uint32_t value;
    if(rc==2){
        // Each item was stored with TWO uint16 at PLC registers
        // Combine them into one uint32 for ture value
        value = ((uint32_t)regs[1] << 16) | regs[0];//小端存储，高高低低
    }else{
        ROS_ERROR("Failed to read register %d: %s, rc %d", reg_addr, modbus_strerror(errno),rc);
    }
    return value;
}

void ModbusWrapper::writeRegister(int reg_addr, uint32_t value) {
    if (!connected_) throw std::runtime_error("Modbus not connected");

    // int rc = modbus_write_register(ctx_, reg_addr, value);
    uint16_t reg_low = value&0xFFFF;
    uint16_t reg_high = (value>>16)&0xFFFF;
    // Each item was stored with TWO uint16 at PLC registers
    // Split uint32 into two uint16 values for writing
    uint16_t regs[2] = {reg_low,reg_high};

    // std::cout << reg_low<<std::endl;
    // std::cout << reg_high<<std::endl;
    // std::cout << reg_addr<<std::endl;

    int rc = modbus_write_registers(ctx_, reg_addr, 2,regs);//小端存储，高高低低
    if (rc != 2) {
        ROS_ERROR("Failed to write register %d: %s, rc %d", reg_addr, modbus_strerror(errno), rc);
        throw std::runtime_error(modbus_strerror(errno));
    }

}