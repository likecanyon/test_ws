/*
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-06-29 21:56:51
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-07-16 16:27:16
 * @FilePath: /test_ws/src/ur_test_ros/src/Wrench_pub.cpp
 * @Description: 读取UR机器人力传感器的数据并并进行滤波
 */

#include <iostream>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Wrench.h>
#include "ur_test_ros/Force.h"
#include "iir_filters/Iir.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");
    ur_rtde::RTDEIOInterface rtde_io("192.168.3.101");
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "Wrench_of_UR");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher FT_pub = n.advertise<geometry_msgs::Wrench>("WrenchData", 1000);
    ros::Publisher Wrench_filtered_pub = n.advertise<geometry_msgs::Wrench>("WrenchDataFiltered", 1000);

    ros::Rate loop_rate(1000);

    std::vector<double> InitQ{0, -1.57, -1.57, -1.57, 1.57, 0};
    // home:0,-90,0,-90,0,00.0, -1.57, -1.57, -1.57, 1.57, 0

    rtde_control.moveJ(InitQ);
   // rtde_control.zeroFtSensor();
    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    // init filter
    Iir::Butterworth::LowPass<2> f1, f2, f3, f4, f5, f6; // NOTE： here order should replaced by a int number!
    const float samplingrate = 1000;                     // Hz
    const float cutoff_frequency = 1;                    // Hz
    f1.setup(2, samplingrate, cutoff_frequency);         // NOTE： here order should replaced by a int number!
    f2.setup(2, samplingrate, cutoff_frequency);
    f3.setup(2, samplingrate, cutoff_frequency);
    f4.setup(2, samplingrate, cutoff_frequency);
    f5.setup(2, samplingrate, cutoff_frequency);
    f6.setup(2, samplingrate, cutoff_frequency);
    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std::vector<double> TCPForce{6, 0.0};
        TCPForce = rtde_receive.getActualTCPForce();
        geometry_msgs::Wrench Wrench_msg;

        Wrench_msg.force.x = TCPForce[0];
        Wrench_msg.force.y = TCPForce[1];
        Wrench_msg.force.z = TCPForce[2];
        Wrench_msg.torque.x = TCPForce[3];
        Wrench_msg.torque.y = TCPForce[4];
        Wrench_msg.torque.z = TCPForce[5];
        FT_pub.publish(Wrench_msg);

        /*****filter*******/
        geometry_msgs::Wrench Wrench_msg_filtered;

        /******filter done*****/
        Wrench_msg_filtered.force.x = f1.filter(TCPForce[0]);
        Wrench_msg_filtered.force.y = f2.filter(TCPForce[1]);
        Wrench_msg_filtered.force.z = f3.filter(TCPForce[2]);
        Wrench_msg_filtered.torque.x = f4.filter(TCPForce[3]);
        Wrench_msg_filtered.torque.y = f5.filter(TCPForce[4]);
        Wrench_msg_filtered.torque.z = f6.filter(TCPForce[5]);
        Wrench_filtered_pub.publish(Wrench_msg_filtered);
        // ROS_INFO("%s", FT_msg.Fx);

        // ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}