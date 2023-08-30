#! /usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs 


"""
    订阅方：订阅坐标变换消息，传入被转换的坐标点，调用转换算法

    流程：
        1.导包；
        2.初始化；
        3.创建订阅对象；
        4.组织被转换的坐标点；
        5.转换逻辑实现，调用tf封装的算法；
        6.输出结果；
        7.spin() | spinOnce();
"""

if __name__ == "__main__":
    # 2.初始化；
    rospy.init_node("dynamic_sub_p")

    # 3.创建订阅对象；
    # 3-1 创建缓存对象
    buffer = tf2_ros.Buffer()
    # 3-2 创建订阅对象（将缓存传入）
    sub = tf2_ros.TransformListener(buffer)

    # 4.组织被转换的坐标点；
    # 4-1 创建一个坐标点
    ps = tf2_geometry_msgs.PointStamped()
    ps.header.frame_id = "turtle1"
    # 时间戳 -- 0
    ps.header.stamp = rospy.Time(0)
    ps.point.x = 2.0
    ps.point.y = 3.0
    ps.point.z = 5.0

    # 5.转换逻辑实现，调用tf封装的算法；
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            # 转换实现
            """
                参数1：被转换的坐标点
                参数2：目标坐标系
                返回值：转换后的坐标点

                PS：
                问题：抛出异常 base_link 不存在
                原因：转换函数调用时，可能还没有订阅到坐标系的相对信息
                解决：try 捕获异常，并处理
            """
            ps_out = buffer.transform(ps, "world")
            rospy.loginfo("转换后的坐标：(%.2f, %.2f, %.2f), 参考的坐标系：%s",
                          ps_out.point.x,
                          ps_out.point.y,
                          ps_out.point.z,
                          ps_out.header.frame_id)
        except Exception as e:
            rospy.loginfo("坐标转换失败！")
            rospy.logwarn("错误提示：" + str(e))
        
        rate.sleep()

    # 6.输出结果；
    # 7.spin() | spinOnce();