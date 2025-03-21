import os
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf import transformations
import json


class State:
    pos: np.zeros(3)  # 位置(x, y, z)
    ang: np.zeros(3)  # 角度(x, y, z)
    vel: np.zeros(3)  # 速度(x, y, z)
    acc: np.zeros(3)  # 加速度(x, y, z)

    def __init__(self, pos=np.zeros(3), ang=np.zeros(3), vel=np.zeros(3), acc=np.zeros(3)):
        """
        初始化节点状态
        :param pos: 位置
        :param ang: 角度
        :param vel: 速度
        :param acc: 加速度
        """
        self.pos = pos
        self.ang = ang
        self.vel = vel
        self.acc = acc

    def __str__(self):
        return f"State(pos={self.pos}, ang={self.ang}, vel={self.vel}, acc={self.acc})"


def state_encode(state, frame_id="world"):
    """
    将状态转换为PoseStamped和TwistStamped
    :param state: 节点状态
    :param frame_id: 坐标系
    :return: 包含位置、角度、速度的PoseStamped和TwistStamped
    """
    # 获取当前时间戳
    time_now = rospy.Time.now()
    # 位置和角度转换为pose
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = time_now
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z = state.pos
    pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, \
        pose_stamped.pose.orientation.w = transformations.quaternion_from_euler(*state.ang)
    # 速度转换为twist
    twist_stamped = TwistStamped()
    twist_stamped.header.stamp = time_now
    twist_stamped.header.frame_id = frame_id
    twist_stamped.twist.linear.x, twist_stamped.twist.linear.y, twist_stamped.twist.linear.z = state.vel
    return pose_stamped, twist_stamped


def pose_decode(pose_stamped):
    """
    将PoseStamped和TwistStamped转换为状态
    :param pose_stamped: pose_stamped信息
    :return: 位置和角度
    """
    pos = (pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    ang = transformations.euler_from_quaternion([
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w,
    ])
    return np.array(pos), np.array(ang)


def twist_decode(twist_stamped):
    """
    将TwistStamped转换为速度
    :param twist_stamped: twist_stamped信息
    :return: 速度
    """
    vel = (twist_stamped.twist.linear.x, twist_stamped.twist.linear.y, twist_stamped.twist.linear.z)
    return np.array(vel)

def generate_command_line(interpreter, args_dict):
    """
    根据参数字典生成命令行调用代码。
    :param interpreter: 解释器路径（如 Python 解释器路径）
    :param args_dict: 包含参数名和值的字典
    :return: 生成的命令行列表，适合 subprocess.Popen 使用
    """
    # 获取 node.py 的路径
    node_file_path = os.path.join(os.path.dirname(__file__), "class_def", "node.py")

    # 初始化命令列表，第一个元素是解释器路径，第二个元素是脚本路径
    command = [interpreter, node_file_path]

    # 遍历参数字典，将参数添加到命令列表中
    for key, value in args_dict.items():
        # 如果是字典或者列表，转换为 JSON 字符串
        if isinstance(value, (dict, list)):
            value = json.dumps(value)
            command.extend([f'--{key}', value])
        else:
            # 添加参数名和值到命令列表
            command.extend([f'--{key}', str(value)])
    return command


def quiver_data_to_segments(x, y, z, u, v, w, length=1):
    """
    将quiver数据转换为segments
    :param x: 坐标x
    :param y: 坐标y
    :param z: 坐标z
    :param u: 速度u
    :param v: 速度v
    :param w: 速度w
    :param length: 箭头长度
    :return:
    """
    # 创建包含起始和终止点的段
    start_points = np.array([x, y, z]).T
    end_points = start_points + np.array([u, v, w]).T * length
    # 合并起始点和终止点
    segments = np.stack([start_points, end_points], axis=1)
    return segments.tolist()


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


def get_pid_state_from_node_state(state, target_vel):
    """
    从节点状态中获取PID控制器的状态
    :param state: 节点状态
    :param target_vel: 目标速度
    :return: PID控制器的状态
    """
    speed = np.linalg.norm(state.vel)  # 当前速率
    speed_target = np.linalg.norm(target_vel)  # 目标速率
    angle = state.ang[2]  # 当前朝向角度
    angle_target = - np.arctan2(target_vel[0],  # 目标朝向角度
                                target_vel[1])
    error = angle_target - angle
    if error > np.pi:  # 按照最短路径调整目标朝向角度
        angle_target = angle_target - 2 * np.pi
    elif error < -np.pi:
        angle_target = angle_target + 2 * np.pi
    return speed, speed_target, angle, angle_target