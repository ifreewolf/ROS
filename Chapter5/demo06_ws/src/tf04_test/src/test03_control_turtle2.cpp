#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

/**
 *  需求1：换算出 turtle1 相对于 turtle2 的关系
 *  需求2：计算角速度和线速度并发布
 * 
*/


int main(int argc, char *argv[])
{
    // 2.编码、初始化、NodeHandle；
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_sub");
    ros::NodeHandle nh;

    // 3.创建订阅对象；
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);

    // A.创建发布对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);

    // 4.编写解析逻辑；
    ros::Rate rate(10.0);
    while (ros::ok()) {
        // 核心
        try {
            geometry_msgs::TransformStamped turtle1ToTurtle2 = buffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
            // ROS_INFO("turtle1 相对于 turtle2 的信息{父级:%s，子级：%s；偏移量(%2f, %.2f, %.2f)}",
            //         turtle1ToTurtle2.header.frame_id.c_str(),
            //         turtle1ToTurtle2.child_frame_id.c_str(),
            //         turtle1ToTurtle2.transform.translation.x,
            //         turtle1ToTurtle2.transform.translation.y,
            //         turtle1ToTurtle2.transform.translation.z);

            // B.根据相对计算并组织速度消息
            geometry_msgs::Twist twist;
            /*
                组织速度，只需要设置线速度的 x 与 角速度的 z
                x = 系数 * sqrt(y^2 + x^2)
                y = 系数 * arctan(y, x)
            */
            twist.linear.x = 0.5 * sqrt(pow(turtle1ToTurtle2.transform.translation.y, 2) + pow(turtle1ToTurtle2.transform.translation.x, 2));
            twist.angular.z = 0.5 * atan2(turtle1ToTurtle2.transform.translation.y, turtle1ToTurtle2.transform.translation.x);

            // C.发布
            pub.publish(twist);

        } catch(const std::exception& e) {
            ROS_INFO("错误提示：%s", e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }

    // 5.spinOnce()。
    return 0;
}
