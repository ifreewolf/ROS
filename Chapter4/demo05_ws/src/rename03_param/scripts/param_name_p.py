#! /usr/bin/env python

import rospy

if __name__ == "__main__":

    rospy.init_node("hello")

    """
        设置不同类型的参数
    """
    # 1. 全局
    rospy.set_param("/radiusA", 100)

    # 2. 相对
    rospy.set_param("radiusB", 100)

    # 3. 私有
    rospy.set_param("~radiusC", 100)

    while not rospy.is_shutdown():
        pass