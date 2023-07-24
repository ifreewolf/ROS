#include "ros/ros.h"

#include "plumbing_pub_sub/Person.h"


void doPerson(const plumbing_pub_sub::Person::ConstPtr &person) {
    ROS_INFO("姓名：%s, 年龄：%d, 身高：%.2f", person-> name.c_str(), person->age, person->height);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("liaotian", 10, doPerson);

    ros::spin();

    return 0;
}