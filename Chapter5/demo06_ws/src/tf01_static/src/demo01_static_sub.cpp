#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"

/**
 *  订阅方：订阅发布的坐标系相对关系，传入一个坐标点，调用 tf 实现转换
 * 
 *  流程：
 *      1.包含头文件；
 *      2.编码、初始化、NodeHandle（必须的）
 *      3.创建订阅对象； ---> 订阅坐标系相对关系
 *      4.组织一个坐标点数据；
 *      5.转换算法，需要调用TF内置实现；
 *      6.最后输出。
*/

int main(int argc, char *argv[])
{
    // 2.编码、初始化、NodeHandle（必须的）
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;

    // 3.创建订阅对象； ---> 订阅坐标系相对关系
    // 3.1 创建一个 buffer 缓存
    tf2_ros::Buffer buffer;

    // 3.2 再创建监听对象(监听对象可以将订阅的数据存入buffer)
    tf2_ros::TransformListener listener(buffer);

    // 4.组织一个坐标点数据；
    geometry
    // 5.转换算法，需要调用TF内置实现；
    // 6.最后输出。
    
    return 0;
}

