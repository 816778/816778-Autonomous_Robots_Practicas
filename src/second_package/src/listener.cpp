#include <ros/ros.h>
#include <std_msgs/String.h>

void messageCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received: %s", msg->data.c_str());
    // Reply to the message
    ROS_INFO("Hello, I am a student!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh; // Default node handle

    ros::Subscriber sub = nh.subscribe("greetings", 1000, messageCallback); // Subscribe to the "greetings" topic

    ros::spin(); // Keep the node running
    return 0;
}
