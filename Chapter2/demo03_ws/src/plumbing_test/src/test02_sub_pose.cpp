#include "ros/ros.h"
#include "turtlesim/Pose.h"

/*
    需求：订阅乌龟的位姿，并输出到控制台
    准备工作：
        1. 获取话题名称 /turtle1/pose
        2. 获取消息类型 turtlesim/Pose
            float32 x
            float32 y
            float32 theta
            float32 linear_velocity
            float32 angular_velocity
        3. 运行前启动 turtlesim_node 与 turtle_teleop_key 节点


    1. 包含头文件；
    2. 初始化 ROS 节点；
    3. 创建节点句柄；
    4. 创建订阅对象；
    5. 处理订阅到的数据（回调函数）；
    6. spin()
*/

void doPose(const turtlesim::Pose::ConstPtr& pose) {
    ROS_INFO("乌龟位姿信息：x=%.2f, y=%.2f, theta=%.2f, lv=%.2f, av=%.2f",
        pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    // 2. 初始化 ROS 节点；
    ros::init(argc, argv, "sub_pose_customer");

    // 3. 创建节点句柄；
    ros::NodeHandle nh;

    // 4. 创建订阅对象；
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, doPose);

    // 5. 处理订阅到的数据（回调函数）；
    // 6. spin()

    ros::spin();


    return 0;
}
