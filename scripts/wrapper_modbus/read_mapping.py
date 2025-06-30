import json
from tools.path_configuration import PathConfig
# from icecream import ic



class ReadMapping:
    def __init__(self):
        self._initialize_properties()

    def _initialize_properties(self):
        """初始化类的属性"""

        self.path_config = PathConfig()

        self.tablepath = self.path_config.table_path        # AddressTable.json
        self.mappingpath = self.path_config.mapping_path    # FunctionSettingTable.json

        try:
            opera_content = self._read_file(self.mappingpath)  # 操作内容
            json_mapping = self._read_file(self.tablepath)   # 具体的地址表参数
        except Exception as e:
            # 日志记录或其他错误处理机制
            print(f"初始化失败: {e}")
            raise
        
        self.opera = opera_content["mapping"]
        self.json_mapping = json_mapping

        self.host = json_mapping.get("host")
        self.port = int(json_mapping.get("port"))
        self.axis_num = json_mapping.get("axis_num")

        # X、Y轴 输入范围的限制：X:-1000～1000, Y: 0～1000
        self.itinerary_setting_x = json_mapping.get("Itinerary Setting",{}).get("X_axis")
        self.itinerary_setting_y = json_mapping.get("Itinerary Setting",{}).get("Y_axis")

        self.write_register = json_mapping.get("mapping", {}).get("write_register")
        self.read_register = json_mapping.get("mapping", {}).get("read_register")

    def _read_file(self, file_path)-> object:
        """读取并返回文件内容"""
        with open(file_path, "r") as j:
            data = json.load(j)
        return data



