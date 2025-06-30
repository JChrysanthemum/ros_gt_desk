import rospy
from icecream import ic


class SystemStatus:
    def __init__(self,client):
        self.client = client
        rospy.set_param('move', 1)

    def check_XAxis_pol(self,x_pol_list,num):
        # 计算两个数的绝对值的差
        abs_difference = abs(x_pol_list[0] - x_pol_list[1])
        
        # 判断绝对值的差是否超过5
        if abs_difference > 5:
            if num == 4 or num == 7:
                self.client.command_anaylsis("0", [[0], [0]])


        return abs_difference
        
    def re_processdata(self,value_list):
        str1 = bin(value_list[0])[2:].zfill(16)
        str2  = bin(value_list[1])[2:].zfill(16)
        bin_str = str2 + str1   # 32位

        if bin_str[0] == '1':
            # 负数，转换为补码
            num = int(bin_str, 2)
            num -= 1 << 32
            return num
        else:
            # 正数
            return int(bin_str, 2)

    def re_process(self, axis_num, reg_tmp):
        processed_data = []
        for i in (range(axis_num)):
            l = [*map(self.re_processdata, reg_tmp[i * 5:i * 5 + 5])]
            processed_data.append(l)
        return processed_data

    def format(self, reg_tmp):
        # 数据校验: 确保reg_tmp长度至少为15
        if len(reg_tmp) < 15:
            raise ValueError("reg_tmp长度不足")

        # 使用循环和列表推导简化代码，提高代码的可维护性和性能
        axis_names = ["X axis 1", "X axis 2", "Y axis"]
        data_list = [["Axis_name", "current status", "fault code", "servo fault code", "current position", "current speed"],]
        processed_data = self.re_process(self.client.remapping.axis_num, reg_tmp)

        self.moveCtrl_param_set(processed_data[0][0])

        for i, axis_name in enumerate(axis_names):
            axis_data = [axis_name]
            data_row = axis_data + processed_data[i]
            data_list.append(data_row)

        print("-" * 50)
        pos_l = [data_list[1][4], data_list[2][4]]
        speed_l = [data_list[1][5], data_list[2][5]]
 
        # 两轴同步：
        # print(f"X axis  position_diff: {self.check_XAxis_pol(pos_l, processed_data[0][0])}")
        # print(f"X axis  speed_diff: {self.check_XAxis_pol(speed_l,processed_data[0][0])}")
 
        for row in data_list:
            print(f"{row[0]:<10} {row[1]:<15} {row[2]:<15} {row[3]:<20} {row[4]:<20} {row[5]:<15}")
        print("-" * 50)

        return processed_data
    
    def moveCtrl_param_set(self,num):
        if self.client.move:
            ic(self.client.move)
            ic(num)
            if num == 4 or num == 7:
                rospy.set_param("move",0)
                self.client.move = rospy.get_param("move")
                ic(self.client.move)
        else:
             if not num == 4 or num == 7:
                rospy.set_param("move",1)
                self.client.move = rospy.get_param("move")



