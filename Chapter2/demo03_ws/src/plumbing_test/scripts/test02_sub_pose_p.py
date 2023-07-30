#! /usr/bin/env python
import rospy
from turtlesim.msg import Pose

"""
    需求：订阅并输出乌龟位姿信息
    准备工作：
        1. 获取话题名称 /turtle1/pose
        2. 获取消息类型 turtlesim/Pose
            float32 x
            float32 y
            float32 theta
            float32 linear_velocity
            float32 angular_velocity
        3. 运行前启动 turtlesim_node 与 turtle_teleop_key 节点
    
    1.导包；
    2.初始化ROS节点；
    3.创建订阅对象；
    4.使用回调函数处理订阅到的信息；
    5.spin()
"""

def doPose(pose):
    rospy.loginfo("乌龟坐标：x=%.2f, y=%.2f, theta=%.2f, vl=%.2f, ve=%.2f", pose.x, pose.y, pose.theta, pose.linear_velocity, pose.angular_velocity)


if __name__ == "__main__":
    # 2.初始化ROS节点；
    rospy.init_node("sub_pose_customer_p")

    # 3.创建订阅对象；
    sub = rospy.Subscriber("/turtle1/pose", Pose, doPose, queue_size=1000)

    # 4.使用回调函数处理订阅到的信息；
    # 5.spin()
    rospy.spin()