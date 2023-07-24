#include "ros/ros.h"
// 1. 包含头文件
#include "plumbing_pub_sub/Person.h"

/*
    发布方：发布人的消息
        1. 包含头文件
            #include "plumbing_pub_sub/Person.h"
        2. 初始化ROS节点
        3. 创建节点句柄
        4. 创建发布者对象
        5. 编写发布逻辑，发布数据
*/
int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    // 2. 初始化ROS节点
    ros::init(argc, argv, "talk_person");
    // 3. 创建节点句柄
    ros::NodeHandle nh;
    // 4. 创建发布者对象
    ros::Publisher pub = nh.advertise<plumbing_pub_sub::Person>("liaotian", 1000);
    // 5. 编写发布逻辑，发布数据
    //      5.1 创建被发布的数据
    plumbing_pub_sub::Person p;
    p.name = "sunwukong";
    p.age = 2000;
    p.height = 1.73;

    //      5.2 创建发布频率
    ros::Rate rate(1);

    ros::Duration(3.0).sleep();

   //      5.3 循环发布数据
    while (ros::ok()) {
        // 核心：发布数据
        pub.publish(p);
        p.age += 1;
        ROS_INFO("我叫%s, 今年%d岁, 高%.2f米", p.name.c_str(), p.age, p.height);
        
        // 休眠
        rate.sleep();
        // 建议
        ros::spinOnce();
    }

    return 0;
}