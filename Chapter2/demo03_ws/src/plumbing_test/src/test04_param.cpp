#include "ros/ros.h"

/*
    需求：修改参数服务器中 turtlesim 背景色相关的参数

    1. 初始化ROS节点；
    2. 不一定需要创建节点句柄(和后续API有关)
    3. 修改参数
*/

int main(int argc, char *argv[])
{
    // 1. 初始化ROS节点；
    ros::init(argc, argv, "change_bgColor");

    // 2. 不一定需要创建节点句柄(和后续API有关)
    // // 方式二：
    // ros::NodeHandle nh("turtlesim");
    // nh.setParam("background_r", 255);
    // nh.setParam("background_g", 255);
    // nh.setParam("background_b", 255);

    // 方式三：
    ros::NodeHandle nh;
    nh.setParam("/turtlesim/background_r", 125);
    nh.setParam("/turtlesim/background_g", 125);
    nh.setParam("/turtlesim/background_b", 125);


    // 3. 修改参数
    // 方式一：
    // ros::param::set("/turtlesim/background_b", 0);
    // ros::param::set("/turtlesim/background_g", 0);
    // ros::param::set("/turtlesim/background_r", 0);

    return 0;
}

