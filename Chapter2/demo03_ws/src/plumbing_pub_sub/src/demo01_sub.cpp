
#include "ros/ros.h"
#include "std_msgs/String.h"

/*
    订阅方实现：
        1.包含头文件；
            ROS中文本类型 ---> std_msgs/String.h
        2.初始化 ROS 节点；
        3.创建节点句柄；
        4.创建订阅者对象；
        5.处理订阅到的数据;
        6.spin()函数。
*/


void doMsg(const std_msgs::String::ConstPtr &msg) {
    // 通过msg获取并操作订阅到的数据
    ROS_INFO("翠花订阅到的数据：%s", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点；
    ros::init(argc, argv, "cuiHua");
    // 3.创建节点句柄；
    ros::NodeHandle nh;
    // 4.创建订阅者对象；
    ros::Subscriber sub = nh.subscribe("fang", 10, doMsg);  // doMsg 是回调函数
    // 5.处理订阅到的数据；
    // 处理订阅数据使用回调函数，回调函数的启用使用ros::spin()启动。

    ros::spin();    // ros::spin()是一个用于处理ROS消息和回调函数的函数，它是一个阻塞函数，会一直运行直到节点被关闭。
    // 当调用ros::spin()时，ROS节点将开始接收和处理来自其他节点的消息，并调用相应的回调函数来处理这些消息。它会持续监听消息队列，并在有新消息到达时触发回调函数。
    // 通常，在ROS节点的主循环中，我们会调用ros::spin()来保持节点的活动状态，以便能够及时处理传入的消息。如果不调用ros::spin()，节点将无法接收和处理消息。
    // 调用 ros::spin() 来启动节点的消息处理循环。
    return 0;
}