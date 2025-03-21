import numpy as np


def find_neighbors(pos_ls, idx, visible_range):
    """
    :param pos_ls: 位置列表
    :param idx: 当前节点的索引
    :param visible_range: 判断为邻居的距离范围
    :return: 邻居列表，平方距离列表
    """
    # 计算当前个体到所有其他个体的平方距离
    distances_squared = np.sum((pos_ls - pos_ls[idx, :]) ** 2, axis=1)
    # 确定在可见范围内的邻居，排除当前个体自身
    is_neighbor = (distances_squared > 0) & (distances_squared < visible_range ** 2)
    neighbors = np.where(is_neighbor)[0]
    return neighbors, distances_squared

#隧道行进算法
class Queue:
    def __init__(self,
                 n_line=1,
                 start_point=np.array([25,25,25]),
                 end_point=np.array([75,75,75]),
                 point_radius = 3,
                 line_distance_to_center = 0,
                 max_vel=9,
                 min_vel=3):

        self.n_line = n_line
        self.start_point = start_point
        self.end_point = end_point
        self.point_radius = point_radius
        self.line_distance_to_center = line_distance_to_center
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.is_running = False

    # def towards_queue(self,pos_ls,idx):

    def start_queue(self, pos_ls, idx):
        return np.linalg.norm(pos_ls[idx, :] - self.start_point) <= self.point_radius

    def __call__(self,pos_ls,vel_ls,idx):
        if self.n_line == 1:
            if self.start_queue(pos_ls, idx):
                self.is_running = True
            else:
                pass

            if self.is_running:
                new_vel = self.end_point - self.start_point

            else:
                new_vel = self.start_point - pos_ls[idx,:]

            speed = np.linalg.norm(new_vel)
            if speed > self.max_vel:
                new_vel = new_vel / speed * self.max_vel
            elif speed < self.min_vel:
                new_vel = new_vel / speed * self.min_vel

            return new_vel


        # elif self.n_line == 2:






# Boids集群算法
class Boids:
    def __init__(self,
                 visible_range=5,
                 separation_range=10,
                 border_distance=30,
                 center=np.array([50, 50, 50]),
                 alignment_factor=0.05,
                 cohesion_factor=0.01,
                 separation_factor=0.1,
                 center_following_factor=0.1,
                 max_vel=9,
                 min_vel=3,
                 boarder_buffer=50):
        """
        :param visible_range: alignment和cohesion的邻居搜索范围
        :param separation_range: separation的邻居搜索范围
        :param border_distance: 判断为离中心太远的距离
        :param center: 集群中心
        :param alignment_factor: alignment调整因子
        :param cohesion_factor: cohesion调整因子
        :param separation_factor: separation调整因子
        :param center_following_factor: center_following调整因子
        :param max_vel: 最大速度
        :param min_vel: 最小速度
        :param boarder_buffer: 中心到边界距离
        """
        self.visible_range = visible_range
        self.separation_distance = separation_range
        self.border_distance = border_distance
        self.center = center
        self.alignment_factor = alignment_factor
        self.cohesion_factor = cohesion_factor
        self.separation_factor = separation_factor
        self.center_following_factor = center_following_factor
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.boarder_min = center + np.array([-boarder_buffer, -boarder_buffer, -boarder_buffer])
        self.boarder_max = center + np.array([boarder_buffer, boarder_buffer, boarder_buffer])

    def alignment(self, vel_ls, idx, neighbors):
        """
        :param vel_ls: 速度列表
        :param idx: 当前节点的索引
        :param neighbors: 邻居列表
        :return: 对齐速度向量
        """
        if len(neighbors) == 0:
            v_alignment = np.array([0, 0, 0])
        else:
            # 计算邻居的平均速度
            avg_vel = np.mean(vel_ls[neighbors, :], axis=0)
            # 调整速度朝向平均速度
            v_alignment = (avg_vel - vel_ls[idx, :]) * self.alignment_factor  # 调整因子
        return v_alignment

    def cohesion(self, pos_ls, idx, neighbors):
        """
        :param pos_ls: 位置列表
        :param idx: 当前节点的索引
        :param neighbors: 邻居列表
        :return: 聚合速度向量
        """
        if len(neighbors) == 0:
            v_cohesion = np.array([0, 0, 0])
        else:
            # 计算邻居的质心
            center_of_mass = np.mean(pos_ls[neighbors, :], axis=0)
            # 朝向质心
            v_cohesion = (center_of_mass - pos_ls[idx, :]) * self.cohesion_factor  # 调整因子
        return v_cohesion

    def separation(self, pos_ls, idx):
        """
        :param pos_ls: 位置列表
        :param idx: 当前节点的索引
        :return: 分离速度向量
        """
        # 根据分离距离找到邻居
        neighbors, distances_squared = find_neighbors(pos_ls, idx, self.separation_distance)

        if len(neighbors) == 0:
            return np.array([0, 0, 0])

        # 计算距离小于分离距离的邻居的位移向量
        distances = np.sqrt(distances_squared[neighbors])
        displacements = pos_ls[idx, :] - pos_ls[neighbors, :]
        normalized_displacements = displacements / distances[:, np.newaxis]

        # 累加所有的分离向量
        v_separation = np.sum(normalized_displacements, axis=0) * self.separation_factor  # 分离强度的调整因子

        return v_separation

    def center_following(self, pos_ls, idx):
        """
        :param pos_ls: 位置列表
        :param idx: 当前节点的索引
        :return: 向心速度向量
        """
        # 计算个体到中心的距离
        distance_to_center = np.linalg.norm(pos_ls[idx, :] - self.center)
        # 判断个体是否离边界太远
        if distance_to_center > self.border_distance:
            # 计算一个引导个体朝向边界的向量
            v_border_following = (self.center - pos_ls[idx, :]) / np.linalg.norm(
                self.center - pos_ls[idx, :]) * self.center_following_factor
        else:
            # 如果个体离边界足够远，则不需要动作
            v_border_following = np.array([0, 0, 0])
        return v_border_following

    def __call__(self, pos_ls, vel_ls, idx):
        """
        :param pos_ls: 所有节点的位置列表
        :param vel_ls: 所有节点的速度列表
        :param idx: 当前节点序号
        :return: 更新后的速度列表
        """
        # 计算邻居
        neighbors, _ = find_neighbors(pos_ls, idx, self.visible_range)
        # 计算各个速度分量
        v_alignment = self.alignment(vel_ls, idx, neighbors)
        v_cohesion = self.cohesion(pos_ls, idx, neighbors)
        v_separation = self.separation(pos_ls, idx)
        v_center_following = self.center_following(pos_ls, idx)
        # 更新速度
        new_vel = vel_ls[idx, :] + (v_alignment + v_cohesion + v_separation + v_center_following)
        # 限制速度，保留正负号
        speed = np.linalg.norm(new_vel)
        if speed > self.max_vel:
            new_vel = new_vel / speed * self.max_vel
        elif speed < self.min_vel:
            new_vel = new_vel / speed * self.min_vel

        # 到达边界区时反向
        pos_this = pos_ls[idx, :]
        new_vel = np.where(pos_this < self.boarder_min, np.abs(new_vel), new_vel)
        new_vel = np.where(pos_this > self.boarder_max, -np.abs(new_vel), new_vel)
        return new_vel