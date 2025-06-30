import os
import logging
# from icecream import ic

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class PathConfig:
    """
    路径配置
    """

    def __init__(self):
        self.FolderName  = "config/"
        self.WriteAddressTable = "Write_address_table.csv"
        self.ReadAddressTable  = "Read_address_table.csv"
        
        self.AddressTable      = "AddressTable.json"
        self.MappingName       = "FunctionSettingTable.json"
    
        try:
            self.p_dir = self.getScriptDirectory()
            self.write_address, self.read_address = self.PathConstruction(self.WriteAddressTable, self.ReadAddressTable)
            self.table_path, self.mapping_path = self.PathConstruction(self.AddressTable, self.MappingName)
        except Exception as e:
            logging.error(f"初始化路径配置失败: {e}")

    def getScriptDirectory(self):
        """获取当前脚本文件的所在目录及其上两级目录"""
        try:
            current_script_directory = os.path.dirname(os.path.realpath(__file__))
            # ic(current_script_directory)
            parent_directory = os.path.abspath(os.path.join(current_script_directory, os.pardir))
            # ic(parent_directory)
            return parent_directory
        except Exception as e:
            logging.error(f"获取路径时出错: {e}")
            raise

    def PathConstruction(self, file_name1, file_name2):
        """构造写地址和读地址文件的路径"""
        try:
            name1 = os.path.join(self.p_dir, self.FolderName + file_name1)
            name2 = os.path.join(self.p_dir, self.FolderName + file_name2)
            return name1, name2
        except Exception as e:
            logging.error(f"构造地址路径时出错: {e}")
            raise
