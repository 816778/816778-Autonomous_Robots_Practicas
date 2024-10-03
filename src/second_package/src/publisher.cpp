#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh; // Default node handle

    ros::Publisher pub = nh.advertise<std_msgs::String>("greetings", 1000); // Topic name "greetings"
    ros::Rate loopRate(10); // 1 Hz

    unsigned int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Autonomous Robot Course " + std::to_string(count);
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        pub.publish(msg); // Publish the message

        ros::spinOnce();
        loopRate.sleep();
        count++;
    }
    return 0;
}
