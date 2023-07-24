#! /usr/bin/env python
# 1. 导包；
import rospy
from plumbing_pub_sub.msg import Person

"""
    发布方：发布人的消息
        1. 导包；
        2. 初始化ROS节点；
        3. 创建发布者对相关；
        4. 组织发布逻辑并发布数据。
"""

if __name__ == "__main__":
    # 2. 初始化ROS节点；
    rospy.init_node("talker_python")

    # 3. 创建发布者对象；
    pub = rospy.Publisher("person", Person, queue_size=1000)

    # 4. 组织发布逻辑并发布数据。
    # 创建数据
    person = Person()
    person.name = "奥特曼"
    person.age = 8
    person.height = 1.85

    # 创建 Rate 对象
    rate = rospy.Rate(1)

    # 发布者注册后，休眠3s，让订阅者完成订阅
    rospy.sleep(3)

    # 循环发布数据
    while not rospy.is_shutdown():
        pub.publish(person)
        rospy.loginfo("发布的消息：%s, %d, %.2f", person.name, person.age, person.height)
        rate.sleep()
        person.age += 1