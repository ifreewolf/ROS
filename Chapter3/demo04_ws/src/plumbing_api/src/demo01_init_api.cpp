#include "ros/ros.h"

/**
 * 作用：ROS初始化函数
 * 
 * 参数：
 *  1. argc     ---- 参数个数（n+1）
 *  2. argv     ---- 封装参数的数组
 *  3. name     ---- 为节点命名(唯一性)
 *  4. options
 * 
 * 使用：
 *  1. argc 与 argv 的使用
 *      如果按照ROS中的特定格式传入实参，那么ROS可以加以使用，比如用来设置全局参数、给节点重命名...
 *  2. options 的使用
 *      节点名称需要保证唯一，会导致一个问题：同一个节点不能重复启动。
 *      结果：ROS 中当有重名的节点启动时，之前的节点会被关闭。
 *      需求：特定场景下，需要一个节点多次启动且能正常运行，怎么办？
 *      解决：设置启动项 ros::init_options::AnonymousName
 *          当创建ROS节点时，会在用户自定义的节点名称后缀随机数，从而避免重名问题。
 *      
 * 
 * 函数原型：void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
*/

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "init_api", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    int count = 0;
    ros::Rate rate(1);
    while (ros::ok())
    {
        ROS_INFO("%s node is running ---- %d", ros::this_node::getName().c_str(), count);
        count++;
        rate.sleep();
    }    
    
    return 0;
}
