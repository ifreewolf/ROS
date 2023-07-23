// 1. 包含头文件；
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/*
    发布方实现：
        1. 包含头文件；
            ROS中文本类型 ---> std_msgs/String.h
        2. 初始化 ROS 节点；
        3. 创建节点句柄；
        4. 创建发布者对象；
        5. 编写发布逻辑并发布数据。
*/

int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");

    // 2. 初始化 ROS 节点；
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc, argv, "erGouZi"); // erGouZi是节点名称，需要唯一

    // 3. 创建节点句柄；
    ros::NodeHandle nh; // 该类封装了 ROS 中的一些常用功能

    // 4. 创建发布者对象；
    // 泛型：发布的消息类型
    // 参数1：要发布到的话题
    // 参数2：队列中最大保存的消息数，超出此阈值时，先进的先销毁
    ros::Publisher pub = nh.advertise<std_msgs::String>("fang", 10); // "fang"是话题名，10是消息队列的长度，超过10条消息，就把之前的消息清除。

    // 5. 编写发布逻辑并发布数据。
    // 要求以 10 Hz 的频率发布数据，并且文本后添加编号
    // 先创建被发布的消息
    std_msgs::String msg;
    // 发布频率
    ros::Rate rate(10); // 1秒10次
    // 设置编号
    int msg_count = 0;
    // 发布者注册后，休眠3s，让订阅者完成订阅
    ros::Duration(3).sleep();
    // 编写循环，循环中发布数据
    while (ros::ok())   // 只要节点还活着，条件就成立
    {
        msg_count++;
        // 实现字符串拼接数字
        std::stringstream ss;
        ss << "hello --->" << msg_count;
        // msg.data = "hello --- " + std::to_string(msg_count)
        msg.data = ss.str();

        pub.publish(msg);
        // 添加日志：
        ROS_INFO("发布的数据是：%s", ss.str().c_str());
        rate.sleep();

        // ros::spinOnce(); // 官方建议，处理回调函数，但是因为这里没有回调函数，所以也可以不需要这句。
    }
    return 0;
}
