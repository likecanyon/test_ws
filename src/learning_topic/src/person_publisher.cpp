/**
 * 本例程发布/person_info话题，
 * 自定义消息类型learning_topic::Person
 */
#include <ros/ros.h>
#include "learning_topic/Person.h"

int main(int argc, char **argv)
{
    // Ros节点初始化
    ros::init(argc, argv, "person_publisher");

    //创建句柄节点
    ros::NodeHandle n;

    //创建一个发布者，发布名为/person_info的topic，消息类型为learning_topic::Person,队列长度为10
    ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);

    //设置循环频率
    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())
    {
        //初始化learning_topic::Person类型的消息
        learning_topic::Person person_msg;
        person_msg.name = "Tom";
        person_msg.age = 18;
        person_msg.sex = learning_topic::Person::male;

        //发布消息
        person_info_pub.publish(person_msg);
        ROS_INFO("Publish Person Info:name:%s   age:%d  sex:%d",
                 person_msg.name.c_str(), person_msg.age, person_msg.sex);

        //按照循环频率延时
        loop_rate.sleep();
    }
    return 0;
}