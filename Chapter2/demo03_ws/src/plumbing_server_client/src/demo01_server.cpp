
// 1. 包含头文件；
#include "ros/ros.h"
#include "plumbing_server_client/AddInts.h"

/*
    服务端实现：解析客户端提交的数据，并运算再产生响应
        1. 包含头文件；
        2. 初始化ROS节点；
        3. 创建节点句柄；
        4. 创建一个服务对象；
        5. 处理请求并产生响应；
        6. spin()
*/

// 两个参数，一个请求，一个响应
bool doNums(plumbing_server_client::AddInts::Request &request, plumbing_server_client::AddInts::Response &response) { // 服务请求的回调函数是一个返回bool值的回调函数，因为需要反馈服务结果
    // 1. 处理请求
    int num1 = request.num1;
    int num2 = request.num2;
    ROS_INFO("收到的请求数据：num1 = %d, num2 = %d", num1, num2);

    // 2. 组织响应
    int sum = num1 + num2;
    response.sum = sum;
    ROS_INFO("求和结果： sum = %d", sum);

    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2. 初始化ROS节点；
    ros::init(argc, argv, "heishui");

    ROS_INFO("服务器端启动！");
    
    // 3. 创建节点句柄；
    ros::NodeHandle nh;

    // 4. 创建一个服务对象；
    ros::ServiceServer server = nh.advertiseService("addInts", doNums); // doNums是回调函数，因为有回调函数，所以不需要定义泛型;advertiseService是泛型函数

    // 5. 处理请求并产生响应；
    // 6. spin()
    ros::spin();
    return 0;
}
