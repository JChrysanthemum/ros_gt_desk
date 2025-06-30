import rospy
from wrapper_modbus.BaseModbusClient import BaseModbusClient
from wrapper_modbus.read_mapping import ReadMapping
import os
import time
from icecream import ic


class PLC_ModbusClient:
    def __init__(self):
        """
        初始化PLC_ModbusClient类。
        """
        self.remapping = ReadMapping()
        self.client = BaseModbusClient(self.remapping.host, self.remapping.port)
        
        # 单轴是否移动控制 参数：
        self.move = 1
        self.max_wait_time = 60  # 最大等待时间，单位为秒
        self.waited_time = 0  # 已等待的时间
    
    def _data_validation(self, data, lower, upper):
        """
        验证数据是否在指定的上下限范围内。

        如果数据在范围内，则记录一条信息日志表示验证通过。
        如果数据不在范围内，则抛出一个ValueError，并记录一条错误日志表示验证失败。

        参数:
        - data: 待验证的数据。
        - lower: 数据的下限。
        - upper: 数据的上限。

        抛出:
        - ValueError: 如果数据不在指定范围内，则抛出此异常。
        """
        try:
            # 检查数据是否在指定的上下限范围内
            if lower <= data and data <= upper:
                rospy.loginfo("位置参数 验证通过")
            else:
                # 如果数据不在范围内，抛出ValueError异常
                raise ValueError(f"{data} 不在 [{lower}, {upper}] 范围内")
        except ValueError as e1:
            # 捕获ValueError异常，记录错误日志，并重新抛出异常
            rospy.logerr("位置参数 验证失败")
            raise e1

    def command_anaylsis(self, mode, value):
        """
        根据给定的模式和值进行命令分析与执行。
        
        :param mode: 指定的模式，用于确定要操作的寄存器或配置。
        :param value: 要写入寄存器或配置的值，可以是序列。
        """
        # 根据模式从映射中获取操作列表
        mode_list = self.remapping.opera["write_register"][str(mode)]

        self._data_validation(value[0][0],
                              self.remapping.itinerary_setting_x[0],
                              self.remapping.itinerary_setting_x[1])
        self._data_validation(value[1][0],
                              self.remapping.itinerary_setting_y[0],
                              self.remapping.itinerary_setting_y[1])

        value = self._norm_command(value)

        # 遍历模式列表，并对每个模式执行单次写入操作
        for i in range(len(mode_list)):
            self.single_write_operation(mode_list[i], value[i])
            rospy.loginfo("单次写入操作完成")
            # time.sleep(5)
            # self.movement_condition()
        rospy.loginfo("写入操作完成")

    def _norm_command(self, value):
        """
        标准化命令参数列表。

        该方法将输入的value列表中的每个项目转换为一个包含六个元素的新列表，
        其中项目原来的三个元素分别占据新列表的第二个、第四个和第六个位置。
        初始的前两个和后两个元素被设为0。

        参数:
        value - 一个包含命令参数的列表，每个参数自身是一个包含三个元素的列表。

        返回值:
        返回一个新列表，其中包含了经过标准化处理的命令参数。
        """
        values = []
        for item in value:
            # 例如 item 为 x 轴 的 pos v acc
            items = []
            for v in item:
                items += self._processdata(v) 
            values.append(items)
        return values

    def _processdata(self, data):
        """
        处理数据，将其转换为二进制并分割为两个部分。
        
        参数:
        - data: 需要处理的整数数据。
        
        返回值:
        - 一个包含两个整数的列表,分别是处理后的数据的前16位和后16位二进制转换后的结果。
        """

        # 判断正负
        if data >= 0:
            binary_str = bin(data)[2:].zfill(32)  # 正数直接转换为32位二进制数
        else:
            # 负数需要转换为补码形式，并确保补码长度为32位
            binary_str = bin(data & 0xffffffff)[2:].zfill(32)
        
        # 将二进制字符串分割并转换为整数
        data1 = int(binary_str[:16],2)  # 前16位
        data2 = int(binary_str[16:],2)  # 后16位
        return [data2, data1] 
    
    def single_write_operation(self, func_name, value):
        """
        根据给定的函数名称和值进行单次写入操作。
        
        :param func_name: 指定的函数名称，用于确定要写入的寄存器。
        :param value: 要写入的寄存器值，可以是序列。
        """
        address = self.remapping.write_register[func_name]["Address"]
        rospy.loginfo(f"func:{func_name}, address:{address}, value:{value}")
        
        # 可以一次设置多个 例如 传数组三个元素，就从寄存器依次设置三个
        self.client._writeRegisters(address, value)


    def single_read_operation(self, func_name,mul):
        """
        根据给定的函数名称读取寄存器。
        
        :param func_name: 指定的函数名称，用于确定要读取的寄存器。
        :return: 读取的寄存器值，可以是序列。
        """
        address = self.remapping.read_register[func_name]["Address"]
        num_registers = self.remapping.read_register[func_name]["num"] * mul
        rospy.loginfo(f"func:{func_name}, address:{address}")

        return self.client.readRegisters(address, num_registers)

    def read_status(self):
        """
        读取当前状态寄存器的值。

        该方法通过遍历重映射字典中的寄存器项，对每个寄存器执行一次单次读取操作，
        并将寄存器名称及其对应的值存储在一个列表中。最后，返回这个包含所有寄存器状态的列表。

        Returns:
            list: 包含每个寄存器名称及其当前值的列表。
        """
        # 初始化一个临时列表，用于存储寄存器名称和值的组合
        register_tmp = []
        # 遍历重映射字典中的每个寄存器项
        for key, values in self.remapping.read_register.items():
            # 对当前寄存器执行单次读取操作，并将结果存储在tmp中
            tmp = self.single_read_operation(key,1)
            # 将寄存器名称和读取的值作为一个元组添加到临时列表中
            register_tmp.append([key,tmp])
        # 返回包含所有寄存器名称和值的列表
        return register_tmp

    def read_status2(self):
        first_key, first_value = next(iter(self.remapping.read_register.items()))
        tmp = self.single_read_operation(first_key,len(self.remapping.read_register))
        
        register_tmp = [tmp[i:i+2] for i in range(0, len(tmp), 2)]        
        return register_tmp
