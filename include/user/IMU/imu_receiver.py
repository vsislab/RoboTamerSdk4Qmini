import argparse
import sys
import serial  # 导入模块
import serial.tools.list_ports
import threading
import struct
import time
import platform
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

# 宏定义参数
PI = 3.1415926
FRAME_HEAD = str('fc')
FRAME_END = str('fd')
TYPE_IMU = str('40')
TYPE_AHRS = str('41')
TYPE_INSGPS = str('42')
TYPE_GEODETIC_POS = str('5c')
TYPE_GROUND = str('f0')
TYPE_SYS_STATE = str('50')
TYPE_BODY_ACCELERATION = str('62')
TYPE_ACCELERATION = str('61')
TYPE_MSG_BODY_VEL = str('60')
IMU_LEN = str('38')  # //56
AHRS_LEN = str('30')  # //48
INSGPS_LEN = str('48')  # //72
GEODETIC_POS_LEN = str('20')  # //32
SYS_STATE_LEN = str('64')  # // 100
BODY_ACCELERATION_LEN = str('10') #// 16
ACCELERATION_LEN = str('0c')  # 12
PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295
isrun = True


# 获取命令行输入参数
def parse_opt(known=False):
    parser = argparse.ArgumentParser()
    # parser.add_argument('--debugs', type=bool, default=False, help='if debug info output in terminal ')
    parser.add_argument('--port', type=str, default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0', help='the models serial port receive data; example: '
                                                                 '    Windows: COM3'
                                                                 '    Linux: /dev/ttyUSB0')

    parser.add_argument('--bps', type=int, default=921600, help='the models baud rate set; default: 921600')
    parser.add_argument('--timeout', type=int, default=20, help='set the serial port timeout; default: 20')
    # parser.add_argument('--device_type', type=int, default=0, help='0: origin_data, 1: for single imu or ucar in ROS')

    receive_params = parser.parse_known_args()[0] if known else parser.parse_args()
    return receive_params


def read_imu_data(port="/dev/ttyUSB0", baudrate=921600, timeout=1):
    try:
        serial_ = serial.Serial(port=port, baudrate=baudrate, bytesize=EIGHTBITS, parity=PARITY_NONE,
                                stopbits=STOPBITS_ONE,
                                timeout=timeout)
        print("baud rates = " + str(serial_.baudrate))
    except:
        print("error:  unable to open port .")
        exit(1)


    temp1=False
    temp2=False
    
    result = {
        "Accelerometer_X": 0,
        "Accelerometer_Y": 0,
        "Accelerometer_Z": 0,
        "RollSpeed": 0,
        "PitchSpeed": 0,
        "HeadingSpeed": 0,
        "Roll": 0,
        "Pitch": 0,
        "Heading": 0,
        "qw": 0,
        "qx": 0,
        "qy": 0,
        "qz":0,
    }

    
    while serial_.isOpen():
        check_head = serial_.read().hex()
        # 校验帧头
        if check_head != FRAME_HEAD:
            continue
        head_type = serial_.read().hex()
        # 校验数据类型
        if (head_type != TYPE_IMU and head_type != TYPE_AHRS and head_type != TYPE_INSGPS and
                head_type != TYPE_GEODETIC_POS and head_type != 0x50 and head_type != TYPE_GROUND and 
                head_type != TYPE_SYS_STATE and  head_type!=TYPE_MSG_BODY_VEL and head_type!=TYPE_BODY_ACCELERATION and head_type!=TYPE_ACCELERATION):
            continue
        check_len = serial_.read().hex()
        # 校验数据类型的长度
        if head_type == TYPE_IMU and check_len != IMU_LEN:
            continue
        elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
            continue
        elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
            continue
        elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
            continue
        elif head_type == TYPE_SYS_STATE and check_len != SYS_STATE_LEN:
            continue
        elif head_type == TYPE_GROUND or head_type == 0x50:
            continue
        elif head_type == TYPE_MSG_BODY_VEL and check_len != ACCELERATION_LEN:
            print("check head type "+str(TYPE_MSG_BODY_VEL)+" failed;"+" check_LEN:"+str(check_len))
            continue
        elif head_type == TYPE_BODY_ACCELERATION and check_len != BODY_ACCELERATION_LEN:
            print("check head type "+str(TYPE_BODY_ACCELERATION)+" failed;"+" check_LEN:"+str(check_len))
            continue
        elif head_type == TYPE_ACCELERATION and check_len != ACCELERATION_LEN:
            print("check head type "+str(TYPE_ACCELERATION)+" failed;"+" ckeck_LEN:"+str(check_len))
            continue
        check_sn = serial_.read().hex()
        head_crc8 = serial_.read().hex()
        crc16_H_s = serial_.read().hex()
        crc16_L_s = serial_.read().hex()


        # 读取并解析IMU数据
        if head_type == TYPE_IMU:
            data_s = serial_.read(int(IMU_LEN, 16))
            IMU_DATA = struct.unpack('12f ii',data_s[0:56])
            #print("Accelerometer_X(m/s^2) : " + str(IMU_DATA[3]))
            #print("Accelerometer_Y(m/s^2) : " + str(IMU_DATA[4]))
            #print("Accelerometer_Z(m/s^2) : " + str(IMU_DATA[5]))
            result["Accelerometer_X"]=IMU_DATA[3]
            result["Accelerometer_Y"]=IMU_DATA[4]
            result["Accelerometer_Z"]=IMU_DATA[5]
            temp1=True

        # 读取并解析AHRS数据
        elif head_type == TYPE_AHRS:
            data_s = serial_.read(int(AHRS_LEN, 16))
            AHRS_DATA = struct.unpack('10f ii',data_s[0:48])
            #print("RollSpeed(rad/s): " + str(AHRS_DATA[0]))
            #print("PitchSpeed(rad/s) : " + str(AHRS_DATA[1]))
            #print("HeadingSpeed(rad) : " + str(AHRS_DATA[2]))
            #print("Roll(rad) : " + str(AHRS_DATA[3]))
            #print("Pitch(rad) : " + str(AHRS_DATA[4]))
            #print("Heading(rad) : " + str(AHRS_DATA[5]))
            #print("qw : " + str(AHRS_DATA[6]))
            #print("qx : " + str(AHRS_DATA[7]))
            #print("qy : " + str(AHRS_DATA[8]))
            # print("qz : " + str(AHRS_DATA[9]))

            result["RollSpeed"]= AHRS_DATA[0]
            result["PitchSpeed"]=AHRS_DATA[1]
            result["HeadingSpeed"]=AHRS_DATA[2]

            result["Roll"]=AHRS_DATA[3]
            result["Pitch"]=AHRS_DATA[4]
            result["Heading"]=AHRS_DATA[5]

            result["qw"]=AHRS_DATA[6]
            result["qx"]=AHRS_DATA[7]
            result["qy"]=AHRS_DATA[8]
            result["qz"]=AHRS_DATA[9]
            temp2=True

        # result = {
        #     "Accelerometer_X": IMU_DATA[3],
        #     "Accelerometer_Y": IMU_DATA[4],
        #     "Accelerometer_Z": IMU_DATA[5],
        #     "RollSpeed": AHRS_DATA[0],
        #     "PitchSpeed": AHRS_DATA[1],
        #     "HeadingSpeed": AHRS_DATA[2],
        #     "Roll": AHRS_DATA[3],
        #     "Pitch": AHRS_DATA[4],
        #     "Heading": AHRS_DATA[5],
        #     "qw": AHRS_DATA[6],
        #     "qx": AHRS_DATA[7],
        #     "qy": AHRS_DATA[8],
        #     "qz": AHRS_DATA[9],
        # }
        
        if temp1 is True and temp2 is True:
          print(result)
          temp2=False
          temp1=False
          return result
        

    return result
# read_imu_data(port="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0", baudrate=921600, timeout=1)
