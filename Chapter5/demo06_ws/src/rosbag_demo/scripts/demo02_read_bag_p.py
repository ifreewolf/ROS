#! /usr/bin/env python

import rospy
import rosbag

"""
    需求：读取磁盘上 bag 文件
    流程：
        1.导包；
        2.初始化；
        3.创建rosbag 对象并且以读的方式打开文件流；
        4.读数据；
        5.关闭流。
"""

if __name__ == "__main__":
    # 2.初始化；
    rospy.init_node("read_bag_p")

    # 3.创建rosbag 对象并且以读的方式打开文件流；
    bag = rosbag.Bag("hello_p.bag", "r")

    # 4.读数据；
    msgs = bag.read_messages("/liaoTian")
    for topic, msg, t in msgs:
        rospy.loginfo("话题：%s, 消息：%s, 时间：%s", topic, msg.data, t)
        print(topic, msg, t)
    
    # 5.关闭流。
    bag.close()