#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "pub_api");

    ros::NodeHandle nh;

    /**
     * 作用：创建发布者对象  
     * 
     * 模板：被发布的消息的类型
     * 
     * 参数：
     *      1.话题名称
     *      2.队列长度
     *      3.latch(可选) 如果设置为true，会保存发布方的最后一条消息，并且新的订阅对象连接到发布方时，发布方会将这条消息发送给订阅者。
     * 
     * 使用：
     *      latch 设置为true的作用？
     *      以静态地图发布为例，方案1：可以使用固定频率发送地图数据，但是效率低；方案2：可以将地图发布对象的latch设置为true，并且发布方只发送一次数据，每当订阅者连接时，将地图数据发送给订阅者(只发送一次)，这样提高了数据的发送效率。
     * 
     * 函数原型：
     *  template<class M>
     *  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch=false);
    */
    ros::Publisher pub = nh.advertise<std_msgs::String>("test_pub_api", 10, true);

    std_msgs::String msg;

    ros::Rate rate(1);

    int count = 0;

    while (ros::ok())
    {
        std::stringstream ss;

        ss << "hello ----> " << count;
        msg.data = ss.str();

        if (count < 10) {
            pub.publish(msg);
            ROS_INFO("发布的数据是：%s", ss.str().c_str());
        }

        rate.sleep(); 

        ros::spinOnce();

        count++;    
    }

    return 0;
}
