import rospy
from ros_gt_desk.msg import Status
from wrapper_modbus.PLC_ModbusClient import PLC_ModbusClient
from wrapper_modbus.listen_setting import SystemStatus


if __name__ == '__main__':
    rospy.init_node('listener_modbus')

    client = PLC_ModbusClient()
    rospy.loginfo("启动 ModBus 客户端")
    sys = SystemStatus(client)

    pub = rospy.Publisher('Listener_modbus', Status, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status = Status()
        processed_data = sys.format(client.read_status2())
        status.x1 = processed_data[0]
        status.x2 = processed_data[1]
        status.y1 = processed_data[2]

        pub.publish(status)
        rate.sleep()    



