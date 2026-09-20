#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
质点四旋翼的位置控制器

控制律（把无人机视为受外力控制的质点）：
    F = m * ( Kp * (p_des - p) + Kd * (v_des - v) + g_vec )

其中 g_vec = (0, 0, g)，用于抵消重力，使飞行器能够悬停。
Kp 决定回位速度，Kd 提供阻尼以抑制振荡。

订阅：
    /ground_truth/odom    nav_msgs/Odometry      当前位姿与速度
    /uav/goal             geometry_msgs/Point    目标点（可随时修改）
发布：
    /quadrotor/wrench     geometry_msgs/Wrench   施加在机体上的外力
    /uav/status           std_msgs/String        运行状态

运行：
    roslaunch octree_uav_3d_pathfinding main.launch
在另一个终端下发新的目标点，可观察飞行器受控飞行：
    rostopic pub -1 /uav/goal geometry_msgs/Point "{x: 8.0, y: 2.0, z: 3.0}"
"""

import math

import rospy
from geometry_msgs.msg import Point, Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class UavController(object):
    """把四旋翼当作质点，用 PD + 重力补偿计算所需外力。"""

    def __init__(self):
        # 物理与控制参数
        self.mass = rospy.get_param('~mass', 1.0)
        self.kp = rospy.get_param('~kp', 4.0)
        self.kd = rospy.get_param('~kd', 4.0)
        self.gravity = rospy.get_param('~gravity', 9.81)
        self.max_force = rospy.get_param('~max_force', 40.0)
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        self.status_period = rospy.get_param('~status_period', 1.0)
        self.tolerance = rospy.get_param('~reach_tolerance', 0.15)

        # 初始目标点（默认即起始位置，表现为稳定悬停）
        self.goal = Point(
            rospy.get_param('~goal_x', 0.0),
            rospy.get_param('~goal_y', 0.0),
            rospy.get_param('~goal_z', 2.0))

        self.odom = None
        self.reached = False
        self.last_status = rospy.get_time()

        self.wrench_pub = rospy.Publisher(
            '/quadrotor/wrench', Wrench, queue_size=1)
        self.status_pub = rospy.Publisher(
            '/uav/status', String, queue_size=10)

        rospy.Subscriber('/ground_truth/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/uav/goal', Point, self.goal_cb, queue_size=1)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_cb)

        rospy.loginfo('位置控制器已启动：质量 %.2f kg，Kp=%.2f，Kd=%.2f，控制频率 %.0f Hz',
                      self.mass, self.kp, self.kd, self.control_rate)
        rospy.loginfo('初始目标点 (%.2f, %.2f, %.2f)',
                      self.goal.x, self.goal.y, self.goal.z)

    def odom_cb(self, msg):
        self.odom = msg

    def goal_cb(self, msg):
        self.goal = msg
        self.reached = False
        rospy.loginfo('收到新目标点 (%.2f, %.2f, %.2f)', msg.x, msg.y, msg.z)

    def control_cb(self, _event):
        # 尚未收到位姿反馈时不施力，避免盲目控制
        if self.odom is None:
            return

        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear

        ex = self.goal.x - p.x
        ey = self.goal.y - p.y
        ez = self.goal.z - p.z

        # 期望加速度 = 比例项 + 阻尼项；z 方向额外加 g 抵消重力
        ax = self.kp * ex - self.kd * v.x
        ay = self.kp * ey - self.kd * v.y
        az = self.kp * ez - self.kd * v.z + self.gravity

        fx = self.mass * ax
        fy = self.mass * ay
        fz = self.mass * az

        # 限幅，防止控制量过大导致仿真发散
        norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        if norm > self.max_force:
            scale = self.max_force / norm
            fx, fy, fz = fx * scale, fy * scale, fz * scale

        wrench = Wrench()
        wrench.force.x = fx
        wrench.force.y = fy
        wrench.force.z = fz
        self.wrench_pub.publish(wrench)

        # 到达判定
        dist = math.sqrt(ex * ex + ey * ey + ez * ez)
        if dist < self.tolerance and not self.reached:
            self.reached = True
            rospy.loginfo('已到达目标点，当前位置 (%.2f, %.2f, %.2f)',
                          p.x, p.y, p.z)

        # 周期性输出状态
        now = rospy.get_time()
        if now - self.last_status > self.status_period:
            self.last_status = now
            text = '位置 (%.2f, %.2f, %.2f)  目标 (%.2f, %.2f, %.2f)  误差 %.3f m' % (
                p.x, p.y, p.z, self.goal.x, self.goal.y, self.goal.z, dist)
            self.status_pub.publish(String(data=text))
            rospy.loginfo(text)


def main():
    rospy.init_node('uav_controller')
    UavController()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
