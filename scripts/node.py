#!/usr/bin/env python3
import rospy
from ros_gt_desk.msg import Control,CtrlDetail
from ros_gt_desk.msg import CtrlNoParam
from ros_gt_desk.msg import Control,CtrlDetail
from ros_gt_desk.msg import CtrlNoParam
from wrapper_modbus.PLC_ModbusClient import PLC_ModbusClient


def callback_axis(msg):
    mode = msg.mode
    try:
        if mode == 2 or mode == 3:
            x_axis = msg.X_axis
            y_axis = msg.Y_axis
            value = [[x_axis.pos, x_axis.vel, x_axis.acc], [y_axis.pos, y_axis.vel, y_axis.acc]]
            rospy.loginfo(f"mode:{mode},value:{value}")
            client.command_anaylsis(mode, value)
        else:
            raise ValueError(f"error mode:{mode}")
    except ValueError as e:
        raise e

def callback_np(msg):
    mode = msg.mode
    try:
        if mode == '0' or mode == '1':
            strs = msg.strs
            value = [[int(strs[0])],[int(strs[1])]]
            rospy.loginfo(f"mode:{mode},value:{value}")
            client.command_anaylsis(mode, value)
        else:
            raise ValueError(f"error mode:{mode}")
    except ValueError as e:
        raise e


if __name__ == '__main__':
    rospy.init_node('node_modbus')

    client = PLC_ModbusClient()
    rospy.loginfo(" node_modbus start!")

    sub_wp = rospy.Subscriber('sub_move_wp', Control, callback_axis)
    sub_np = rospy.Subscriber('sub_move_np', CtrlNoParam, callback_np)
    rospy.spin()

