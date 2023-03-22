/**
 * 本例订阅/person_info话题，
 * 自定义消息类型为learning_topic::Person
 */

#include <ros/ros.h>
#include "learning_topic/Person.h"

void personInfoCallback(const learning_topic::Person::ConstPtr &msg)
{
    ROS_INFO("Subscribe Person Info:    name:%s    ,age:%d     ,gender:%d",msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_subsciber");

    //创建节点句柄
    ros::NodeHandle n;

    //创建一个Subscriber,订阅名为/person_info的topic,注册回调函数person_Callback
    ros::Subscriber person_sub = n.subscribe("/person_info", 10, personInfoCallback);

    //循环等待回调函数
    ros::spin();

    return 0;
}