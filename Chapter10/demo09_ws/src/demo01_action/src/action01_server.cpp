#include "ris/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "demo01_action/AddIntsAction.h"

/***
    需求：
        创建两个ROS节点，服务器和客户端，
        客户端可以向服务端发送目标数据N(一个整型数据)
        服务器会计算1到N之间所有整数的和，这是一个循环累加的过程，返回给客户端，这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用action实现。
    流程：
        1. 包含头文件;
        2. 初始化ROS节点;
        3. 创建NodeHandle;
        4. 创建 action 服务对象；
        5. 请求处理(a.解析提交的目标值；b.产生连续反馈；c.最终结果响应) --- 回调函数；
        6. spin()回旋。
*/

typedef actionlib::SimpleActionSever<demo01_action::AddIntsAction> Server;

// 5. 请求处理(a.解析提交的目标值；b.产生连续反馈；c.最终结果响应) --- 回调函数；
void cb(const demo01_action::AddIntsGoalConstPtr &goalPtr, Server* server) {
    // a. 解析提交的目标值；
    int goal_num = goalPtr->num;
    ROS_INFO("客户端提交的目标值是：%d", goal_num);

    // b. 产生连续反馈;
    ros::Rate rate(10); // 10 Hz
    int result = 0;
    // c. 最终结果响应
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    // 2. 初始化ROS节点;
    ros::init(argc, argv, "addInts_server");

    // 3. 创建NodeHandle;
    ros::NodeHandle nh;

    // 4. 创建 action 服务对象；
    /*
        SimpleActionServer(ros::NodeHandle n,
                    std::string name,
                    boost::function<void (const demo01_action::AddIntsGoalConstPtr &)> execute_callback，// 回调函数
                    bool auto_start);
        参数1：NodeHandle
        参数2：话题名称
        参数3：回调函数
        参数4：是否自动启动
    */
    Server server(nh, "addInts", boost::bind(&cb, _1, &server), false);
    server.start(); // 如果 auto_start 为 false，那么需要手动调用该函数启动服务

    // 6. spin()回旋。
    ros::spin();

    return 0;
}