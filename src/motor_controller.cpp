// 主控制节点
#include <ros/ros.h>
#include "ros/package.h"
#include "ros_gt_desk/modbus_wrapper.h"
#include "ros_gt_desk/ConfigParser.h"
#include "ros_gt_desk/operhand.h"

class MotorController {
public:
    MotorController(const std::string& config_path) : nh_("~"), modbus_(nullptr),config_(config_path) {
        // ROS_INFO("???");
        // 加载配置
        config_ = ConfigParser(config_path);

        // 初始化Modbus
        try {
            modbus_ = new ModbusWrapper(config_.getHost(), config_.getPort());
            modbus_->connect();
        } catch (const std::exception& e) {
            ROS_FATAL("Modbus initialization failed: %s", e.what());
            ros::shutdown();
        }

        // ROS接口
        pub_ = nh_.advertise<ros_gt_desk::operhand>("motor_status", 10);
        sub_ = nh_.subscribe("motor_cmd", 10, &MotorController::cmdCallback, this);
        
        // 定时状态读取
        timer_ = nh_.createTimer(ros::Duration(1.0/config_.getPollingRate()),
                               &MotorController::timerCallback, this);
    }

    ~MotorController() {
        if (modbus_) delete modbus_;
    }

private:
    void cmdCallback(const ros_gt_desk::operhand::ConstPtr& msg) {
        try {
            // 电源控制
            modbus_->writeRegister(config_.getRegister("power_enable"), msg->power_cmd ? 1 : 0);
            
            // 电机控制
            if (msg->power_cmd) {
                if (msg->voltage >= 1500 &&  msg->voltage <= 4800) {
                    modbus_->writeRegister(config_.getRegister("voltage"), msg->voltage);
                    modbus_->writeRegister(config_.getRegister("motor_enable"), msg->motor_cmd ? 1 : 0);
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Control command failed: %s", e.what());
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        ros_gt_desk::operhand msg;
        try {
            msg.power_state = modbus_->readRegister(config_.getRegister("power_enable")) > 0;
            msg.motor_state = modbus_->readRegister(config_.getRegister("motor_enable")) > 0;
            msg.position_feedback = modbus_->readRegister(config_.getRegister("voltage"));
            pub_.publish(msg);
        } catch (const std::exception& e) {
            ROS_ERROR("Status reading failed: %s", e.what());
        }
    }
    ros::NodeHandle nh_;
    ModbusWrapper* modbus_;
    ConfigParser config_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller");
    const std::string path = ros::package::getPath("ros_gt_desk") + "/config/motor_config.json";
    MotorController controller(path);
    ros::spin();
    return 0;
}