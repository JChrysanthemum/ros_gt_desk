import rospy
from typing import List, Union
from typing import Any


try:
    from pymodbus.client import ModbusTcpClient
except ImportError as e:
    print("pymodbus 似乎未安装。\n")
    print(e)
    exit()
from .post_threading import Post
from threading import Lock


class ModbusCommunicationError(Exception):
    """自定义异常，表示 Modbus 通信错误"""

    def __init__(self, message: str, original_exception: Exception):
        super().__init__(message)
        self.original_exception = original_exception


class BaseModbusClient:
    def __init__(self, host: str, port: int = 502, rate: float = 50, reset_registers: bool = True):
        """
        初始化 Modbus 客户端。

        :param host: Modbus 服务器的 IP 地址。
        :param port: 端口号（默认：502）。
        :param rate: 每秒读取寄存器的频率（默认：50）。
        :param reset_registers: 是否在读取后将保持寄存器重置为 0（如果可写；默认：True）。
        """
        try:
            self.client = ModbusTcpClient(host, port)
            isConnect = self.client.connect()
            if isConnect:
                rospy.loginfo("已连接到 Modbus 服务器 '%s:%d'", host, port)
            else:
                rospy.logwarn("无法连接到 Modbus 服务器 '%s:%d'", host, port)
                raise 
        except Exception as e:
            raise e

        self.post = Post(self)
        self.__last_output_time = rospy.get_time()

        self.__mutex = Lock()
        rospy.on_shutdown(self._closeConnection)

    def _closeConnection(self):
        """
        关闭 Modbus 连接。
        """
        self.client.close()

    def _writeRegisters(self, address_start: int, values: Union[int, List[int]], slave: int = 0):
        """
        写入 Modbus 寄存器。

        :param address_start: 要写入的值的起始地址。
        :param values: 要写入的值（整数或整数列表）。
        :param slave: 从站 ID（默认：0）。
        """
        with self.__mutex:
            try:
                if not rospy.is_shutdown():
                    self.client.write_registers(address_start, values, slave=slave)
                    rospy.loginfo(
                        "BaseModbusClient-_writeRegisters: 写入地址: %d, 值: %s",
                        address_start,
                        values,
                    )
            except Exception as e:
                rospy.logwarn(
                    "无法将值 %s 写入地址 %d。异常：%s",
                    str(values),
                    address_start,
                    str(e),
                )
                raise ModbusCommunicationError(
                    f"向 Modbus 地址 {address_start} 写入时出错", e
                )

    def _handle_none_value(self, x: Any) -> int:
        """
        处理 None 值，将其替换为 0。

        :param x: 需要检查和可能替换的值。
        :return: 如果输入值不是 None，则返回该值，否则返回 0。
        """
        if x is None:
            return 0
        return x

    def _readRegisters(self, address_start: int, num_registers: int) -> List[int]:
        """
        读取 Modbus 寄存器。

        :param address_start: 要读取的寄存器的起始地址。
        :param num_registers: 要读取的寄存器数量。
        :return: 寄存器值列表。
        """
        with self.__mutex:
            try:
                result = self.client.read_holding_registers(address_start, num_registers)
                return result.registers
            except Exception as e:
                rospy.logwarn(
                    "在地址 %d 上读取失败。异常：%s",
                    address_start,
                    str(e),
                )
                raise ModbusCommunicationError(
                    f"从 Modbus 地址 {address_start} 读取时出错", e
                )

    def readRegisters(self, address_start: int, num_registers: int) -> List[int]:
        """
        读取 Modbus 寄存器，并处理 None 参数的错误。

        :param address_start: 要读取的寄存器的起始地址。
        :param num_registers: 要读取的寄存器数量。
        :return: 寄存器值列表。
        """
        if address_start is None or num_registers is None:
            rospy.logwarn(
                "无效参数：address_start=%s, num_registers=%s。将 None 替换为 0。",
                str(address_start),
                str(num_registers),
            )

        address_start = self._handle_none_value(address_start)
        num_registers = self._handle_none_value(num_registers)

        rospy.loginfo(
            "BaseModbusClient-readRegisters: 地址: %d, 寄存器数量: %d",
            address_start,
            num_registers,
        )
        tmp = self._readRegisters(address_start, num_registers)
        rospy.loginfo("BaseModbusClient-readRegisters: tmp: %s", tmp)
        return tmp