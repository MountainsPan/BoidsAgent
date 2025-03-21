from geometry_msgs.msg import PoseStamped, TwistStamped
from utils import State
import numpy as np
from tf import transformations
import rospy


class MotionCapture:
    def __init__(self, name):
        # 暂存的节点状态
        self.state = State()
        self.name = name
        self.subscriber_pose = None
        self.subscriber_twist = None

    # 启动定位
    def run(self):
        # 启动ros
        pose_topic = "/vrpn_client_node/{}/pose".format(self.name)  # 订阅vrpn_PoseStamped话题
        self.subscriber_pose = rospy.Subscriber(pose_topic, PoseStamped, self.update_pos_ang)
        twist_topic = "/vrpn_client_node/{}/twist".format(self.name)  # 订阅vrpn_TwistStamped话题
        self.subscriber_twist = rospy.Subscriber(twist_topic, TwistStamped, self.update_vel)

    # 关闭定位
    def stop(self):
        self.subscriber_pose.unregister()
        self.subscriber_twist.unregister()

    # 读出节点状态
    def __call__(self, *args, **kwargs):
        return self.state

    def update_pos_ang(self, pose_stamped):
        """
        更新位置、角度
        :param pose_stamped: 位置、角度数据
        :return:
        """
        # 位置和角度，动捕中顺序为[z, x, y]
        pos = (pose_stamped.pose.position.z, pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        ang = transformations.euler_from_quaternion([
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.w,
        ])
        self.state.pos = np.array(pos)
        self.state.ang = np.array(ang)

    # 更新速度、加速度
    def update_vel(self, twist_stamped):
        """
        更新速度
        :param twist_stamped: 速度数据
        :return:
        """
        # 速度，动捕中顺序为[z, x, y]
        vel = (twist_stamped.twist.linear.z, twist_stamped.twist.linear.x, twist_stamped.twist.linear.y)
        self.state.vel = np.array(vel)
