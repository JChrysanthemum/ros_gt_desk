// 主控制节点
#include <ros/ros.h>
#include "ros/package.h"
#include "modbus_wrapper.h"
#include "configParser.h"
#include "ros_gt_desk/MotorControl.h"
#include "ros_gt_desk/MotorStatus.h"
#include "ros_gt_desk/SliderControl.h"
#include "ros_gt_desk/SliderStatus.h"

class DeskControl {

public:
    // The json file that contains host port register-settings for PLC
    const std::string cfg_path = ros::package::getPath("ros_gt_desk") + "/config/PLC_config.json";
    configParser cfg_map = configParser(cfg_path);

private:
    ros::NodeHandle nh;
    ModbusWrapper* modbus_plc;
    ros::Publisher pub_motor;
    ros::Publisher pub_slider;
    ros::Subscriber sub_motor;
    ros::Subscriber sub_slider;
    ros::Timer timer_;
    
public:
    // DeskControl() : cfg_map(cfg_path) {
    DeskControl(){

        // Connect to PLC by modbus
        try {
            // The modbus connection must have <host><port> to initialize
            modbus_plc = new ModbusWrapper(cfg_map.getValueByPath<std::string>("host"), cfg_map.getValueByPath<std::int16_t>("port"));
            modbus_plc->connect();
        } catch (const std::exception& e) {
            // C++ std error from ModbusWrapper construction may not interrupt ros at some OS
            // Manually ros shutdown and PLC connection free is needed. 
            ROS_FATAL("Modbus initialization failed: %s", e.what());
            ros::shutdown();
        }
        
        

        // ROS接口
        pub_motor = nh.advertise<ros_gt_desk::MotorControl>("/gt_desk/mower_status", 10);
        ROS_INFO("Mower status will be show at /gt_desk/mower_status");
        sub_motor = nh.subscribe("/gt_desk/mower_control", 10, &DeskControl::motorCallback, this);
        ROS_INFO("Send msg to /gt_desk/mower_control to enable and control mower");


        pub_slider = nh.advertise<ros_gt_desk::SliderStatus>("/gt_desk/slider_status", 10);
        ROS_INFO("Slider status will be show at /gt_desk/slider_status"); // Not implemented
        sub_slider = nh.subscribe("/gt_desk/slider_control", 10, &DeskControl::sliderCallback, this);
        ROS_INFO("Send msg to /gt_desk/slider_control to [stop/zero/abs_move/rel_move] sliders");
        

        // 定时状态读取
        timer_ = nh.createTimer(ros::Duration(1.0/cfg_map.getValueByPath<int>("polling_rate")),
                               &DeskControl::timerCallback, this);
    }

    ~DeskControl(){
        if (modbus_plc) delete modbus_plc;
    }


private:
    void motorCallback(const ros_gt_desk::MotorControl::ConstPtr& msg) {
        // - 启动顺序 1.使能数控电源. 2.给输出电压,1500-4800之间，使能电机，
        // - 关闭顺序 1.关闭电机，2.关闭数控电源
        try {
            if (msg->mode >= 1){
                modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.motor_power_cmd.0"), 1);
                // Wait for PLC device to enable up
                ros::Duration(0.1).sleep(); 
                int voltage = (msg->voltage >= 1500 &&  msg->voltage <= 4800) ? msg->voltage : 1500;
                modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.motor_voltage.0"), voltage);
                ros::Duration(0.1).sleep(); 
                modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.motor_enable.0"), 1);
            //     
            }else
            {
                modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.motor_enable.0"), 0);
                ros::Duration(0.1).sleep(); 
                modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.motor_voltage.0"), 0);
                ros::Duration(0.1).sleep(); 
                modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.motor_power_cmd.0"), 0);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Mower control command failed: %s", e.what());
        }
    }

    void sliderCallback(const ros_gt_desk::SliderControl::ConstPtr& msg) {
        try {
            // Stop 
            if (msg->mode ==0)
            {
                if (msg->x[0] == 1)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_stop.0"), 1);
                if (msg->y[0] == 1)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_stop.0"), 1);
            }
            // Zeroing
            else if (msg->mode ==1)
            {
                if (msg->x[0] == 1)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_zero_start.0"), 1);
                if (msg->y[0] == 1)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_zero_start.0"), 1);
            }
            // Absolute Move
            else if (msg->mode ==2)
            {
                // Loc 0 is valid as zeroing
                if (msg->x[0] >= 0) // Pos (mm)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_abs_pos.0"), msg->x[0]);
                if (msg->x[1] > 0) // SPD (mm/s)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_abs_pos_spd.0"), msg->x[1]);
                if (msg->x[2] > 0) // ACC (mm/s2)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_abs_pos_acc.0"), msg->x[2]);

                if (msg->y[0] >= 0) // Pos (mm)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_abs_pos.0"), msg->y[0]);
                if (msg->y[1] > 0) // SPD (mm/s)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_abs_pos_spd.0"), msg->y[1]);
                if (msg->y[2] > 0) // ACC (mm/s2)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_abs_pos_acc.0"), msg->y[2]);
            }
            // Relative Move
            else if (msg->mode ==3)
            {
                // Offset 0 is invalid
                if (msg->x[0] > 0) // Dis (mm)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_rel_pos.0"), msg->x[0]);
                if (msg->x[1] > 0) // SPD (mm/s)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_rel_pos_spd.0"), msg->x[1]);
                if (msg->x[2] > 0) // ACC (mm/s2)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.x_rel_pos_acc.0"), msg->x[2]);

                if (msg->y[0] >= 0) // Dis (mm)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_rel_pos.0"), msg->y[0]);
                if (msg->y[1] > 0) // SPD (mm/s)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_rel_pos_spd.0"), msg->y[1]);
                if (msg->y[2] > 0) // ACC (mm/s2)
                    modbus_plc->writeRegister(cfg_map.getValueByPath<int>("registers.y_rel_pos_acc.0"), msg->y[2]);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Control command failed: %s", e.what());
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        ros_gt_desk::MotorStatus msg_motor;
        ros_gt_desk::SliderStatus msg_slider;
        try {
            // @todo ADD slider status, PLC_Config.json Address [1100-1128] with "R" right
            // @todo ADD two axis loc check
            msg_motor.power = modbus_plc->readRegister(cfg_map.getValueByPath<int>("registers.motor_power_enable.0")) > 0;
            msg_motor.motor = modbus_plc->readRegister(cfg_map.getValueByPath<int>("registers.motor_enable.0")) > 0;
            msg_motor.voltage = modbus_plc->readRegister(cfg_map.getValueByPath<int>("registers.motor_voltage.0"));
            pub_motor.publish(msg_motor);


            msg_slider.pos_x1 = modbus_plc->readRegister(cfg_map.getValueByPath<int>("registers.x1_cur_pos.0"));
            msg_slider.pos_x2 = modbus_plc->readRegister(cfg_map.getValueByPath<int>("registers.x2_cur_pos.0"));
            msg_slider.pos_y = modbus_plc->readRegister(cfg_map.getValueByPath<int>("registers.y_cur_pos.0"));

            std::vector<std::string> err_name = {"x1_cur_status","x1_err_code","x1_servo_err_code","x2_cur_status","x2_err_code","x2_servo_err_code","y_cur_status","y_err_code","y_servo_err_code"};
            
            std::vector<std::string> err_path;
            for (const auto& name : err_name) {
                err_path.push_back("registers." + name + ".0");
            }

            std::vector<int> err_codes = {};
            for (const auto& _path : err_path) {
                err_codes.push_back(modbus_plc->readRegister(cfg_map.getValueByPath<int>(_path)));
            }

            msg_slider.err_code = err_codes;
            pub_slider.publish(msg_slider);


        } catch (const std::exception& e) {
            ROS_ERROR("Status reading failed: %s", e.what());
        }
    }
    
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_gt_desk_node");
    DeskControl controller;
    ROS_INFO("PLC Control Node Started");
    ros::spin();
    return 0;
}