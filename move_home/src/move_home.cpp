#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <locale.h>

#define PI (4*atan(1))   

int main(int argc, char** argv){
    setlocale(LC_ALL, "");
    ros::init(argc, argv,"def_pos_node");
    ros::NodeHandle node_obj;

    float posV [6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    ros::Rate loop_rate(1);
    for (int i = 1; i <= 6; i++){
        std::string topic_name = "/m1n6s300/joint_" + std::to_string(i) + "_position_controller/command";
        ros::Publisher joint_publisher = node_obj.advertise<std_msgs::Float64>(topic_name, 10);
        std_msgs::Float64 joint_msg;
        joint_msg.data = posV[i-1];
        loop_rate.sleep();
        joint_publisher.publish(joint_msg);
        ROS_INFO("Articulaci%cn %d lista", 243, i);
        ros::spinOnce(); 
        loop_rate.sleep();    
    }

    
    return 0;
}