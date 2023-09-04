import rospy
from turtlesim.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf
import sys

# 接收乌龟名称的变量
turtle_name = ""

def doPose(pose):
    pub = tf2_ros.TransformBroadcaster()
    ts = TransformStamped()
    ts.header.frame_id = "world"
    ts.header.stamp = rospy.Time.now()

    ts.child_frame_id = turtle_name

    ts.transform.translation.x = pose.x
    ts.transform.translation.y = pose.y
    ts.transform.translation.z = 0.0

    qtn = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
    ts.transform.rotation.x = qtn[0]    
    ts.transform.rotation.y = qtn[1]
    ts.transform.rotation.z = qtn[2]
    ts.transform.rotation.w = qtn[3]

    pub.sendTransform(ts)

if __name__ == "__main__":
    rospy.init_node("dynamic_pub_p")

    # 解析传入的参数(现在传入几个参数？ 文件全路径 + 传入的参数 + 节点名称 + 日志文件路径)
    if len(sys.argv) != 4:
        rospy.logerr("参数个数不对")
        sys.exit(1)
    else:
        turtle_name = sys.argv[1]
    
    sub = rospy.Subscriber(turtle_name + "/pose", Pose, doPose, queue_size=100)

    rospy.spin()
