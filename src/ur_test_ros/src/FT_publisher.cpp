#include <iostream>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ur_test_ros/Force.h"

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
    ros::init(argc, argv, "FT_publisher");

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
    ros::Publisher FT_pub = n.advertise<ur_test_ros::Force>("FTdata", 1000);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        ur_test_ros::Force FT_msg;
        std::vector<double> TCPForce{6, 0.0};
        TCPForce = rtde_receive.getActualTCPForce();
        //std::cout<<TCPForce[0]<<std::endl;
        FT_msg.Fx = TCPForce[0];
        FT_msg.Fy = TCPForce[1];
        FT_msg.Fz = TCPForce[2];
        FT_msg.Tx = TCPForce[3];
        FT_msg.Ty = TCPForce[4];
        FT_msg.Tz = TCPForce[5];

        //ROS_INFO("%s", FT_msg.Fx);

        // ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        FT_pub.publish(FT_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}