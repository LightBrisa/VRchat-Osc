import argparse
import config
from Preprocess.preprocess import load_data
from Socket.UDP import send_data_to_unity, DataProcessServer
import torch
import time
import numpy as np
from articulate.math import quaternion_to_rotation_matrix
from threading import Thread, Event
from bleak import BleakClient, BleakScanner
from pygame.time import Clock
import asyncio
import onnxruntime as rt
import serial
import serial.tools.list_ports
import struct
from collections import namedtuple
from Socket.UDP import *
import config
from pythonosc import udp_client
import socket
from Math.angular import axis_angle_to_rotation_matrix, rotation_matrix_to_euler_angle
import json

# 全局变量
global BUFFER_SIZE
global data_buffer
global CALIBRATION_DONE
global my_server
global i_session
global left_euler, right_euler
global data_ready

CALIBRATION_DONE = False
BUFFER_SIZE = 500
data_buffer = []
data_buffer_angle = []
clock = Clock()
data_ready = Event()  # 初始化 data_ready 事件

def unity_args_setting():
    parser = argparse.ArgumentParser(description='Visual System')

    parser.add_argument('--mode', default='single', type=str, choices=['single', 'twins'], help='single for a model, twins for two models at the same time')
    parser.add_argument('--skeleton', default='smpl', type=str, choices=['smpl', 'h36m'], help='The type of skeleton used by your data')
    parser.add_argument('--rotation_type', default='AXIS_ANGLE', type=str, choices=['AXIS_ANGLE', 'DCM', 'QUATERNION', 'R6D', 'EULER_ANGLE'], help='Rotation representations. Quaternions are in wxyz. Euler angles are in local XYZ.')
    parser.add_argument('--part', default='body', type=str, choices=['body', 'upper_body', 'lower_body', 'head', 'spine', 'left_hand', 'right_hand', 'left_leg', 'right_leg', 'hands'], help='You can choose the part of visualization')
    parser.add_argument('--fps', default=60, type=int, help='The frame rate at which the animation is played')

    args = parser.parse_args()

    return args

args = unity_args_setting()

providers = ['CPUExecutionProvider']
i_session = rt.InferenceSession('.\LIP_full_body.onnx', providers=providers)
device_config = config.device_config.jacket_17

my_server = DataProcessServer(rotation_type=args.rotation_type, part=args.part, config=device_config, use_trans=False)
clock = Clock()

# 数据传输的顺序若变化, 请根据情况重新设置
# 0: left
# 1: right
# 2: back
# 3: root
imu_data_order = device_config['imu_order']
acc_scale = 1

def data_receive():
    MTI_MODULE_NUM = 4
    ONE_AXIS_MODULE_NUM = 2
    BT_FRAME_HEADER_LEN = 2
    QUATERNION_DATA_LEN = 16
    SAMPLETIMEFINE_DATA_LEN = 4
    ACCELERATION_DATA_LEN = 12
    PACKETCOUNTER_DATA_LEN = 2
    MtiData = namedtuple('MtiData', ['packet_counter', 'sample_time_fine', 'acceleration', 'quaternion'])
    OneAxisData = namedtuple('OneAxisData', ['angle'])
    BtFrame = namedtuple('BtFrame', ['header', 'len', 'mti_data', 'one_axis_data', 'check_sum'])

    def parse_bt_frame(data):
        index = 0
        header = data[index:index + BT_FRAME_HEADER_LEN]
        header = [byte for byte in header]
        index += BT_FRAME_HEADER_LEN

        length = struct.unpack('<H', data[index:index + 2])[0]
        index += 2

        mti_data_list = []
        for _ in range(MTI_MODULE_NUM):
            packet_counter = data[index:index + PACKETCOUNTER_DATA_LEN]
            packet_counter = [byte for byte in packet_counter]
            index += PACKETCOUNTER_DATA_LEN

            sample_time_fine = data[index:index + SAMPLETIMEFINE_DATA_LEN]
            sample_time_fine = [byte for byte in sample_time_fine]
            index += SAMPLETIMEFINE_DATA_LEN

            acceleration = data[index:index + ACCELERATION_DATA_LEN]
            acceleration = struct.unpack(">3f", acceleration)
            acceleration = [round(value, 10) for value in acceleration]
            index += ACCELERATION_DATA_LEN

            quaternion = data[index:index + QUATERNION_DATA_LEN]
            quaternion = struct.unpack(">4f", quaternion)
            quaternion = [round(value, 10) for value in quaternion]
            index += QUATERNION_DATA_LEN

            mti_data_list.append(MtiData(packet_counter, sample_time_fine, acceleration, quaternion))

        one_axis_data_list = []
        for _ in range(ONE_AXIS_MODULE_NUM):
            angle = struct.unpack('<f', data[index:index + 4])[0]
            angle = round(angle, 6)
            index += 4
            one_axis_data_list.append(OneAxisData(angle))

        check_sum = data[index]
        return BtFrame(header, length, mti_data_list, one_axis_data_list, check_sum)

    def find_receiver():
        port_list = list(serial.tools.list_ports.comports())
        for port in port_list:
            if port.description.find('CH340') > -1:
                print(f'接收器已连接! 设备名: {port.description}')
                return port.name
        print('未找到接收器!')
        return None

    try:
        global data_buffer
        global data_buffer_angle
        port = find_receiver()
        bps = 115200
        timeout = 5
        uart = serial.Serial(port, bps, timeout=timeout)

        while True:
            str_1 = uart.read(1).hex()
            if str_1 == 'ff':
                str_2 = uart.read(1).hex()
                if str_2 == 'fe':
                    clock.tick()
                    hex_data = str_1 + ' ' + str_2
                    for i in range(147):
                        hex_data += f' {uart.read(1).hex()}'
                    if hex_data[5:].find('ff fe') > -1:
                        print('err data')
                        continue
                    byte_data = bytes.fromhex(hex_data)
                    frame = parse_bt_frame(byte_data)

                    data = []
                    for i in range(4):
                        data += frame.mti_data[i].acceleration
                        data += frame.mti_data[i].quaternion

                    elbow_angle = np.array([frame.one_axis_data[0].angle, frame.one_axis_data[1].angle])
                    elbow_angle = -elbow_angle
                    elbow_angle = np.clip(elbow_angle, 0, 170)

                    elbow_angle = elbow_angle.tolist()
                    data_buffer.append(np.array(data))
                    data_buffer_angle.append((elbow_angle))
        uart.close()

    except Exception as result:
        print("******error******：", result)

def data_transmit(root_fix, fps=30):
    global CALIBRATION_DONE
    global data_buffer
    global i_session
    global left_euler, right_euler
    global data_ready
    while True:
        time.sleep(1 / fps)
        if len(data_buffer) == 0:
            print('\r', '等待接收数据...', end='')
            continue

        if CALIBRATION_DONE == False:
            input('数据接收成功, 请按任意键开始姿态校准')
            print('请保持T-Pose')
            for i in range(5):
                time.sleep(1)
                print(5 - i)

            print('校准数据采集完成! 上传中...')
            tpose_data = np.array(data_buffer[-60:]).reshape(-1, 4, 7).mean(axis=0)
            tpose_acc = torch.tensor(tpose_data[:, 0:3]) * acc_scale
            tpose_q = torch.tensor(tpose_data[:, 3:7])
            tpose_oris = quaternion_to_rotation_matrix(tpose_q)

            tpose_data_angle = np.array(data_buffer_angle[-60:]).reshape(-1, 2).mean(axis=0).tolist()
            tpose_acc = tpose_acc[imu_data_order].view(-1)
            tpose_oris = tpose_oris[imu_data_order].view(-1)

            tpose_data = np.array(torch.cat([tpose_acc, tpose_oris], dim=0)).tolist()
            my_server.set_calibrate_data(tpose_data)
            CALIBRATION_DONE = True
        else:
            data = data_buffer[-1].reshape(4, 7)
            accs = torch.tensor(data[:, 0:3]) * acc_scale
            q = torch.tensor(data[:, 3:7])
            oris = quaternion_to_rotation_matrix(q)
            accs = accs[imu_data_order].view(-1)
            oris = oris[imu_data_order].view(-1)
            data = np.array(torch.cat([accs, oris], dim=0)).tolist()
            data = my_server.calibrate(data)
            my_server.operator(data)
            data_feed = my_server.to_predict_data()
            result = i_session.run(output_names=None, input_feed=data_feed)
            result = my_server.predict_result(result, root_fix=root_fix)
            my_server.update_data(result)

            left_elbow_data = result[17]
            right_elbow_data = result[18]

            left_elbow_matrix = axis_angle_to_rotation_matrix(left_elbow_data)
            right_elbow_matrix = axis_angle_to_rotation_matrix(right_elbow_data)

            left_euler = rotation_matrix_to_euler_angle(left_elbow_matrix)
            right_euler = rotation_matrix_to_euler_angle(right_elbow_matrix)
            data_ready.set()

def start_tcp_server(ip, port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((ip, port))
    server_socket.listen(5)
    print(f"TCP Server listening on {ip}:{port}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connected by {addr}")
        handle_client(client_socket, addr)

def handle_client(client_socket, addr):
    global left_euler, right_euler
    global data_ready
    try:
        while True:
            data_ready.wait()
            if left_euler is not None and right_euler is not None:
                data_to_send = json.dumps({
                    "left_elbow": left_euler.tolist(),
                    "right_elbow": right_euler.tolist()
                })
                client_socket.sendall(data_to_send.encode('utf-8'))
                time.sleep(1 / 30)
            else:
                client_socket.sendall(b'Waiting for data initialization...')
    except Exception as e:
        print(f"Error handling client {addr}: {e}")
    finally:
        client_socket.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Clothes Live Demo')
    parser.add_argument('--root_fix', default="False", type=str, choices=["True", ' False'], help='fix root rotation')
    parser.add_argument('--fps', default=32, type=int, help='fps')
    args = parser.parse_args()

    if args.root_fix == 'False':
        args.root_fix = False
    else:
        args.root_fix = True

    t_pool = []
    t_pool.append(Thread(target=data_receive))
    t_pool.append(Thread(target=data_transmit, kwargs={'root_fix': bool(args.root_fix), 'fps': args.fps}))
    t_pool.append(Thread(target=start_tcp_server, args=('127.0.0.1', 8888)))  # 启动 TCP 服务器线程
    for t in t_pool:
        t.start()
