#include "ros/ros.h"

/*
    演示参数的查询
    实现：
        ros::NodeHandle ----------------------
            param(键， 默认值)
                存在，返回对应结果，否则返回默认值
            
            getParam(键， 存储结果的变量)
                存在，返回 true， 且将值赋值给参数2
                若键不存在，那么返回false，且不为参数2赋值
            
            getParamCached(键，存储结果的变量)----提高变量获取效率
                存在，返回 true， 且将值赋值给参数2
                若键不存在，那么返回值为 false， 且不为参数2赋值
            
            getParamNames(std::vector<std::string>)
                获取所有的键，并存储在参数 vector 中
            
            hasParam(键)
                是否包含某个键，存在返回 true， 否则返回 false
            
            searchParam（参数1， 参数2）
                搜索键，参数1是被搜索的键，参数2存储搜索结果的变量
        

        ros::param ---------------------------------


    需求：首先设置机器人的共享参数，
*/

int main(int argc, char *argv[]) {
    // 设置编码
    setlocale(LC_ALL, "");

    // 初始化参数节点
    ros::init(argc, argv, "get_param_c");

    // 创建节点句柄
    ros::NodeHandle nh;

    // ros::NodeHandle -------------------
    // 1. param(键， 默认值)
    double radius = nh.param("radius", 0.5); // 查找键为radius的值，如果没有radius这个键，则返回结果0.5
    ROS_INFO("radius = %.2f", radius);

    // 2. getParam(键， 存储结果的变量)
    double radius2 = 0.0f;
    bool result = nh.getParam("radius", radius2);
    if (result) {
        ROS_INFO("获得的半径是：%.2f", radius2);
    } else {
        ROS_INFO("查询的结果不存在！");
    }

    // 3.getParamCached(键，存储结果的变量)----提高变量获取效率, 以往的获取方式，是经过RPC远程连接获取，该函数从缓存里面获取数据，如果之前获取过该key，则直接从缓存中返回结果，否则通过RPC远程获取。
    // getParamCached 与 getParam 类似，只是性能上的提升。
    result = nh.getParamCached("radius", radius2);
    if (result) {
        ROS_INFO("获得的半径是：%.2f", radius2);
    } else {
        ROS_INFO("查询的结果不存在！");
    }

    // 4. getParamNames(std::vector<std::string>)
    std::vector<std::string> names;
    nh.getParamNames(names);
    for_each(names.begin(), names.end(), [](std::string name) {
        ROS_INFO("遍历的元素：%s", name.c_str());
    });

    // 5. hasParam(键) 判断某个键是否存在
    bool flag1 = nh.hasParam("radius");
    bool flag2 = nh.hasParam("radiusxxx");

    ROS_INFO("radius 存在吗？ %d", flag1);
    ROS_INFO("radiusxxx 存在吗？ %d", flag2);

    // 6. searchParam（参数1， 参数2）
    std::string key;
    nh.searchParam("type", key);
    ROS_INFO("搜索结果：%s", key.c_str());

    ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++");
    // ros::param -------------------------------------------
    // 1. param()函数
    double radius_param = ros::param::param("radius_param", 100.5);   // 不存在就返回默认值100.5
    double radius_param2 = ros::param::param("radiusxx", 100.5);   // 不存在就返回默认值100.5
    
    ROS_INFO("radius_param = %.2f", radius_param);
    ROS_INFO("radius_param2 = %.2f", radius_param2);

    // 2. getParam()函数
    bool flag = ros::param::get("radius_param", radius_param);
    if (flag) {
        ROS_INFO("radius_param = %.2f", radius_param);
    } else {
        ROS_INFO("未获取到参数！");
    }
    

    // 3. getParamCached()函数
    flag = ros::param::getCached("radius_param", radius_param);
    if (flag) {
        ROS_INFO("radius_param = %.2f", radius_param);
    } else {
        ROS_INFO("未获取到参数！");
    }


    // 4. getParamNames()函数
    std::vector<std::string> names_param;
    ros::param::getParamNames(names_param);
    for_each(names_param.begin(), names_param.end(), [](std::string name){
        ROS_INFO("键：%s", name.c_str());
    });

    // 5. has()判断某个键是否存在
    flag = ros::param::has("type_param");
    if (flag) {
        ROS_INFO("存在参数type_param!");
    } else {
        ROS_ERROR("不存在参数type_param!");
    }

    // 6.searchParam(参数1，参数2)
    ros::param::search("type_param", key);
    ROS_INFO("搜索键：%s", key.c_str());

    return 0;
}