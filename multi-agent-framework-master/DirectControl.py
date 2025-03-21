import socket
import numpy as np
import time


car_communication = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip_1 = '192.168.1.207'
port_1 = int(12345)

ip_2 = '192.168.1.205'
port_2 = int(12345)

ip_3 = '192.168.1.201'
port_3 = int(12345)

ip_4 = '192.168.1.208'
port_4 = int(12345)

def cvt_ctrl_to_car_ctrl(speed, angle):

    """
    将控制输入转换为小车控制输入
    :param speed: 径向速度
    :param angle: 角速度
    :return: 小车控制输入
    """
    buffer = np.zeros(4)
    if angle != 0:
        speed = int(speed * (100 - abs(angle)) / 100)
    buffer[0] = max(-100, min(100, speed - angle))
    buffer[1] = max(-100, min(100, speed + angle))
    buffer[2] = max(-100, min(100, speed - angle))
    buffer[3] = max(-100, min(100, speed + angle))
    return buffer

def send_ctrl(speed, angle,ip,port):
    """
    发送控制指令
    :param speed: 径向速度
    :param angle: 角速度
    :return:
    """
    buffer = cvt_ctrl_to_car_ctrl(speed, angle)  # 将控制输入转换为小车控制输入
    command = "<%d,%d,%d,%d>" % (int(buffer[0]), int(buffer[1]), int(buffer[2]), int(buffer[3]))
    car_communication.sendto(command.encode(), (ip, port))  # 发送控制指令

while True:  # 更新PID控制器
     # 等待下一次控制
    user_input = input("请输入一个字符").strip().lower()

    if user_input == "w":
        send_ctrl(70, -7.5,ip_1,port_1)
        time.sleep(1)

    elif user_input == "a":
        send_ctrl(0, 42,ip_1,port_1)
        time.sleep(0.2)

    elif user_input == "d":
        send_ctrl(0, -40,ip_1,port_1)
        time.sleep(0.2)

    elif user_input == "s":
        send_ctrl(-70, 7,ip_1,port_1)
        time.sleep(1)

    elif user_input == "x":
        send_ctrl(0, 0, ip_1, port_1)


