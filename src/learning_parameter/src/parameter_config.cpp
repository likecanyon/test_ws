#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
    int red, green, blue;

    // ROS节点初始化
    ros::init(argc, argv, "parameter_config");

    //创建节点句柄
    ros::NodeHandle n;

    //读取背景颜色参数
    ros::param::get("/turtlesim/background_r", red);
    ros::param::get("/turtlesim/background_g", green);
    ros::param::get("/turtlesim/background_b", blue);

    //设置背景颜色参数
    ros::param::set("/turtlesim/background_r", 255);
    ros::param::set("/turtlesim/background_g", 255);
    ros::param::set("/turtlesim/background_b", 255);

    ROS_INFO("Set Background color[255, 255, 255]");

    //读取背景颜色参数
    ros::param::get("/turtlesim/background_r", red);
    ros::param::get("/turtlesim/background_g", green);
    ros::param::get("/turtlesim/background_b", blue);

    ROS_INFO("Re-get Background Color[%d,%d,%d]", red, green, blue);

    //调用服务，刷新背景颜色
    ros::service::waitForService("/clear");
    ros::ServiceClient clear_background = n.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clear_background.call(srv);

    sleep(1);
    return 0;
}