# ros_gt_desk: GT-02 Tools platform control

![Show](misc/ros_gt_desk.jpg)

Communication with PLC device via modbus to control electronic tools on the lifting platforms.

Currently, includes `gantry sliders` (two x-axis sliders and one y slider) and a `mower`.


The original version (`CtrNoParam.msg` for zeroing, `node.py` for sending, `operhand.msg` for mower control, [@吕锐](https://github.com/ouranar), [@张培](https://github.com/ZP1931301733), [@赵盼盼](https://github.com/lzuzhaopp) ) was restored at [15ec71](https://github.com/JChrysanthemum/ros_gt_desk/tree/15ec71f8cbe6e292ee5fd2820e5937d4a6de1643).

------

# 2. Structure

```
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config           # for configuration of PLC or other devices
├── doc              # markdown doc for this repo
├── include          # cpp head files
├── launch           # ros launch file 
├── misc             # tiny images or other temp files
├── msg              # ros message
├── package.xml
├── scripts          # ros python scripts
└── src              # ros cpp source file
```


File|Comment
-----|-----
[desk_control.launch](launch/desk_control.launch)| Start ros node to process control msg and send to PLC; Also recieve msg from PLC periodically. Launch this **FIRST** before sending or recieving anything.
[PLC_Config.json](config/PLC_Config.json)| The host address, port, polling rate and register detail were restore in this json file.
[configParser.h](include/ros_gt_desk/configParser.h)|Use nlohmann to read json file. Call `getValueByPath<T>()` to get corresponding value in config json. List index is supported.
[modbus_wrapper.cpp](src/modbus_wrapper.cpp)| Connect to PLC device and process input value from a int32 into 2 int16 that could be restored at PLC holding registers.
[ros_gt_desk.cpp](src/ros_gt_desk.cpp)| Control node for all devices. You can find topic and corresponding msg here.
[modbus_dummy.py](scripts/modbus_dummy.py)| For **DEBUGING**, create a virtual PLC server at localhost, that you can test your new code locally.
[modbus_reader.py](scripts/modbus_reader.py) | For **DEBUGING**, check the name of corresponding register in PLC. Check name in [PLC_Config.json](config/PLC_Config.json).



Topic|Device|Message
-----|-----|-----
/gt_desk/mower_control | Mower Control| [MotorControl.msg](msg/MotorControl.msg) # Mode for on/off and Voltage for speed
/gt_desk/mower_status | Mower Surveillance | [MotorStatus.msg](msg/MotorStatus.msg)  # Status of DCDC power enable and voltage, motor enable
/gt_desk/slider_control | Sliders Control| [SliderControl.msg](msg/SliderControl.msg) # x,y is a list has three value, so x [1,2,3]
/gt_desk/slider_status | Sliders Surveillance| [SliderStatus.msg](msg/SliderStatus.msg) # Abs loc and debugging codes 


# 3. Enviroment setup

Modbus communication node need `libmodbus` and `nlohmann`. Virtual PLC server and PLC server check script need `pymodbus`.

```bash
sudo apt update
sudo apt install libmodbus-dev nlohmann-json3-dev
python3 -m pip install pymodbus
```

If libmodbus can not be installed by system package installer, use source code to install it.

```bash
# Only if libmodbus-dev can not be installed
cd ~
git clone https://github.com/stephane/libmodbus.git
cd libmodbus
./autogen.sh
./configure
make
sudo make install
sudo ldconfig
```

