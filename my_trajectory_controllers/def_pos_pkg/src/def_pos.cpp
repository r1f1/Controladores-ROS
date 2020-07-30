#include <ros/ros.h>
#include <iostream>
#include <armadillo>
#include "def_pos_pkg/Trayectoria2.hpp"
#include <traj_msgs/command_msg.h>

#define PI (4*atan(1))   

int main(int argc, char** argv){

    ros::init(argc, argv,"def_pos_node");
    ros::NodeHandle node_obj;
    ros::Publisher point_publisher = node_obj.advertise<traj_msgs::command_msg>("/m1n6s300/joint_2_position_controller/command",10);
    ros::Rate loop_rate(200);

    Tray move;
    arma::mat resultado = move.jtraj(1, PI/2, 1, 200);

    for (float i = 0; i < 200; i++){
        traj_msgs::command_msg msg;
        msg.pos = resultado(i, 0);
        msg.vel = resultado(i, 1);
        ROS_INFO("Posición: %f velocidad %f", msg.pos, msg.vel);
        point_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}