#! /usr/bin/env python
# 1. 导包；
import rospy
from std_msgs.msg import String # 发布的消息的类型

"""
    使用 python 实现消息发布：
        1. 导包；
        2. 初始化 ROS 节点；
        3. 创建发布者对象；
        4. 编写发布逻辑并发布数据。
"""

if __name__ == "__main__":
    # 2. 初始化 ROS 节点；
    rospy.init_node("sanDai") # 传入节点名称
    # 3. 创建发布者对象；
    pub = rospy.Publisher("che", String, queue_size=10)
    # 4. 编写发布逻辑并发布数据。
    # 创建数据
    msg = String()
    # 指定发布频率
    rate = rospy.Rate(1)
    # 设置计数器
    count = 0
    # 发布者注册后，休眠3s，让订阅者完成订阅
    rospy.sleep(3)
    # 使用循环发布数据
    while not rospy.is_shutdown(): # 节点是否关闭
        count += 1
        msg.data = "hello" + str(count)
        # 发布数据
        pub.publish(msg)
        # 日志输出
        rospy.loginfo("发布的数据：%s", msg.data)
        # 休眠 1/hz 时间
        rate.sleep()