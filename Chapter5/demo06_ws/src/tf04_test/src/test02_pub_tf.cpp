#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "turtlesim/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


// 声明变量接收传递的参数
std::string turtle_name;

void doPose(const turtlesim::Pose::ConstPtr& pose)
{
    // 获取位姿信息，转换成坐标系相对关系(核心)，并发布
    // a.创建TF发布对象；
    static tf2_ros::TransformBroadcaster pub;
    // b.组织被发布的数据；
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.header.stamp = ros::Time::now();
    // 关键点2：turtle1 或 turtle2 都是动态传入的
    ts.child_frame_id = turtle_name;
    // 坐标系偏移量设置
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0.0;
    // 坐标系四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, pose->theta);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    // 发布数据
    pub.sendTransform(ts);

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "turtle_pub");
    ros::NodeHandle nh;

    /*
        解析 launch 文件通过 args 传入的参数
    */
    if (argc != 2) {
        ROS_ERROR("请传入一个参数");
        return 1;
    } else {
        turtle_name = argv[1]; 
    }

    // 3.创建订阅对象，订阅 /turtle1/pose
    ros::Subscriber sub = nh.subscribe(turtle_name + "/pose", 100, doPose);

    // 4.回调函数处理订阅的消息：将位姿信息转换成坐标相对关系并发布(关注)

    ros::spin();

    return 0;
}
