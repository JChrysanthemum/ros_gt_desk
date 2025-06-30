#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import csv
import json
from path_configuration import PathConfig

class CsvToJson:
    def __init__(self):
        self._initialize_properties()

    def _initialize_properties(self):
        self.path_config = PathConfig()

        mapping_w = self._read_csv(self.path_config.write_address)
        mapping_r = self._read_csv(self.path_config.read_address)

        self._json_setting(mapping_w, mapping_r)

    def _read_csv(self, file_name):
        with open(file_name, 'r') as csv_file:
            csv_data = csv.reader(csv_file, delimiter=',')
            csv_data_filtered = [row[1:3] + [row[7]] for row in csv_data][2:]
        mapping = self._process_csv_data(csv_data_filtered)
        return mapping

    def _process_csv_data(self, data):
        register = {}
        for item in data:
            register[item[2]] = {
                "Address": item[0][1:],
                "Type": item[1][1:],
                "num": 2  # 静态值，如果需要可以根据数据动态设置
            }
            register[item[2]]["Address"] = self.safe_type_conversion(
                register[item[2]]["Address"], register[item[2]]["Type"])

        return register

    def _json_setting(self, data1, data2):
        plc_json = {
            "host": "192.168.2.152",
            "port": 502,
            "axis_num": 3,
            
            "Itinerary Setting":{
                "X_axis":[-1000,1000],
                "Y_axis":[0,1000]
            },
            "mapping": {"write_register": data1, "read_register": data2}
        }
        json_file_path = self.path_config.table_path

        # 将字典转换为 JSON 格式的字符串并写入文件
        with open(json_file_path, 'w') as json_file:
            json.dump(plc_json, json_file, indent=4)

        print(f"The JSON file has been saved to the {json_file_path}")

    def safe_type_conversion(self,data, target_type):
        """
        安全地将数据转换为指定类型。
        
        :param data: 需要转换的数据
        :param target_type: 目标数据类型（字符串形式）
        :return: 转换后的数据，如果无法转换则返回 None
        """
        # 定义允许的类型转换字典
        allowed_types = {
            "INT": int,
            "FLOAT": float,
            "BOOL": bool,
            "STR": str
        }

        # 检查目标类型是否在允许的类型列表中
        if target_type not in allowed_types:
            print(f"Unsupported type conversion: {target_type}")
            return None

        # 尝试根据目标类型进行数据转换
        try:
            return allowed_types[target_type](data)
        except ValueError:
            print(f"Value '{data}' cannot be converted to type {target_type}")
            return None
        except TypeError:
            print(f"Value '{data}' cannot be converted to type {target_type}")
            return None


c = CsvToJson()



