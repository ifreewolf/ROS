#! /usr/bin/env python

import rospy
from plumbing_pub_sub.msg import Person

def doPerson(person):
    rospy.loginfo("我订阅的数据：%s, %d, %.2f", person.name, person.age, person.height)


if __name__ == "__main__":
    # 1.初始化节点
    rospy.init_node("listener_python")

    # 2.创建订阅者对象
    sub = rospy.Subscriber("person", Person, doPerson, queue_size=1000)

    # 3.调用回调函数
    rospy.spin()

    