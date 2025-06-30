""" Virtual PLC Server for debugging

Auto overwrite addr/port in config json, add write them back
on exit.
"""
import logging
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
# Modbus 3.6.9

import os
from os.path import join as pj
import json
import math
from typing import Any, Iterable

# Config json file Address and port replace
REAL_ADDR = "192.168.2.152"
REAL_PORT = 502

FAKE_ADDR = "127.0.0.1"
FAKE_PORT = 1502

current_file_path = os.path.abspath(__file__)
cfg_root = "/".join(current_file_path.split("/")[:-2])

_fs = ["PLC_Config"]

def _rep_inf(_addr, _port):
    """Replace 'host' and 'port' value in json file
    """
  
    for f in _fs:
        _pth = pj(cfg_root, "config", f"{f}.json")
        if not os.path.exists(_pth):
            print(_pth, "Not exists")
            continue
        with open(_pth,"r") as fp:
            js = json.load(fp)
        js["host"] = _addr
        js["port"] = _port
        with open(_pth,"w") as fp:
           json.dump(js,fp,indent=4, separators=(',', ': '))
        print(f"Change host to {_addr} and port to {_port}, at {_pth}")
    

FORMAT = ('%(asctime)-15s %(threadName)-15s '
          '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT, level=logging.INFO)
log = logging.getLogger()

def run_server():
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [0]*10),  # Discrete Inputs
        co=ModbusSequentialDataBlock(0, [0]*10),  # Coils
        # Data to write or read at our PLC device. Start at 1000
        hr=ModbusSequentialDataBlock(1000, [0]*200), # Holding Registers
        ir=ModbusSequentialDataBlock(0, [0]*10))  # Input Registers

    context = ModbusServerContext(slaves=store, single=True)

    print(f"Virtual Modbus Server started on {FAKE_ADDR}:{FAKE_PORT}")
    StartTcpServer(context=context, identity=None, address=(FAKE_ADDR, FAKE_PORT))

if __name__ == "__main__":
    try:
        _rep_inf(FAKE_ADDR,FAKE_PORT)
        run_server()
    except KeyboardInterrupt as e:
        print("")
        _rep_inf(REAL_ADDR,REAL_PORT)
        
    # pass
        