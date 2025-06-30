"""Read PLC register value, named by PLC_Config.json

Usage: python modbus_reader.py motor_enable
"""
import logging
import argparse  # 导入 argparse 模块
from pymodbus.client import ModbusTcpClient
import pymodbus
print(pymodbus.__version__)

import os
from os.path import join as pj
import json
from modbus_dummy import REAL_ADDR,REAL_PORT,FAKE_ADDR,FAKE_PORT

FAKE_PLC = True

current_file_path = os.path.abspath(__file__)
cfg_root = "/".join(current_file_path.split("/")[:-2])
with open(pj(cfg_root, "config", "PLC_Config.json")) as fp:
    register_addresses = json.load(fp)["registers"]


# 配置日志（可选，用于调试）
FORMAT = ('%(asctime)-15s %(threadName)-15s '
          '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT, level=logging.INFO)
log = logging.getLogger()

def read_registers_from_server(register_name):
    """
    从 Modbus 服务器读取寄存器，起始地址从字典中获取。

    Args:
        register_name (str): 寄存器名称，用于从地址字典中查找起始地址。
    """

    # 获取寄存器起始地址
    if register_name in register_addresses:
        start_address = register_addresses[register_name][0]
        print(f"读取 {register_name} 寄存器，起始地址: {start_address}")
    else:
        print(f"错误：未找到寄存器名称 '{register_name}'。")
        return  # 退出函数

    # 创建 Modbus TCP 客户端
    # client = ModbusTcpClient('localhost', port=1502)
    if FAKE_PLC:
        client = ModbusTcpClient(FAKE_ADDR, port=FAKE_PORT)
    else:
        client = ModbusTcpClient(REAL_ADDR, port=REAL_PORT)

    # 连接到服务器
    if client.connect():
        print("Connected to Modbus Server.")

        # 读取保持寄存器
        # read_holding_registers(address, count, unit=0)
        # address: 起始地址 (例如，从寄存器 0 开始)
        # count: 要读取的寄存器数量 (例如，读取 10 个)
        # unit: 从站ID (对于单个从站通常为 0 或 1)
        try:
            # 读取从起始地址开始的 2 个保持寄存器
            result = client.read_holding_registers(address=start_address, count=2, slave=1) # 添加 slave=1

            if not result.isError():
                print(f"成功读取保持寄存器: {result.registers}")
            else:
                print(f"读取寄存器失败: {result}")

        except Exception as e:
            print(f"读取寄存器时发生错误: {e}")
        finally:
            # 关闭客户端连接
            client.close()
            print("Client connection closed.")
    else:
        print("无法连接到 Modbus 服务器。请确保服务器已启动并在监听 'localhost:5020'。")

if __name__ == "__main__":
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description="从 Modbus 服务器读取寄存器。")

    # 添加命令行参数
    parser.add_argument("register_name", help="要读取的寄存器名称 (temperature, pressure, flow_rate, level)")

    # 解析命令行参数
    args = parser.parse_args()

    # 调用读取寄存器的函数
    read_registers_from_server(args.register_name)