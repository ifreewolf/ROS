import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Twist
import math


if __name__ == "__main__":
    rospy.init_node("sub_tf_p")

    buffer = tf2_ros.Buffer()

    sub = tf2_ros.TransformListener(buffer)

    # 创建速度消息发布对象
    pub = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)
    
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        try:
            ts = buffer.lookup_transform("turtle2", "turtle1", rospy.Time())
            # rospy.loginfo("父级坐标系：%s，子级坐标系：%s，偏移量(%.2f, %.2f, %.2f)",
            #               ts.header.frame_id, ts.child_frame_id,
            #               ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z)

            # 组织 Twist 消息
            twist = Twist()
            # 线速度和角速度
            twist.linear.x = 0.5 * math.sqrt(ts.transform.translation.x ** 2 + ts.transform.translation.y ** 2)
            twist.angular.z = 0.7 * math.atan2(ts.transform.translation.y, ts.transform.translation.x)

            # 发布
            pub.publish(twist)
        except Exception as e:
            rospy.logerr(e)
        
        rate.sleep()