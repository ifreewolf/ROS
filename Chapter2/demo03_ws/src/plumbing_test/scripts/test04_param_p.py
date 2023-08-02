#! /usr/bin/env python

import rospy

"""
    需求：修改小乌龟GUI的背景色

    1.初始化 ros 节点
    2.设置参数
"""

if __name__ == "__main__":
    rospy.init_node("change_bgColor_p")

    # 修改背景色
    # 方式一(通用)：
    # rospy.set_param("/turtlesim/background_r", 50)
    # rospy.set_param("/turtlesim/background_g", 50)
    # rospy.set_param("/turtlesim/background_b", 50)

    # 方式二(先不用，后面运行管理课程会解释)：
    rospy.set_param("background_r", 200)
    rospy.set_param("background_g", 200)
    rospy.set_param("background_b", 200)