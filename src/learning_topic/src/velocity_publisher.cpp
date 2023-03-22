/*
**本历程将发布turtle1/cmd_vel话题
**消息类型为geometry_msgs::Twist
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "velocity_publisher");

    //创建节点句柄
    ros::NodeHandle n;

    //创建一个publisher,发布名为/turtle1/cmd_vel的话题，
    //消息类型为geometry::msgs::Twist 队列长度为10
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    //设置循环频率
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        //初始化geometry_msgs::Twist类型的消息
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 1.0;
        vel_msg.angular.z = 0.6;

        //发布消息
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("publish turtle velocity command[%0.2f m/s,%0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);

        //按照循环频率延时
        loop_rate.sleep();
    }
    return 0;
}