#include "../include/kinetic.h"
#include <iostream>
#include <vector>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    std::vector<double> a = {9.55, -104, -96.4, 0, 0, 0};
    std::vector<double> d = {0, 0, 0, 0, 80.66, 27.5};
    std::vector<double> alpha = {PI/2, 0, 0, PI/2, PI/2, 0};
    Kinetic ur3(a, d, alpha);
    double angle0[6] = {0, -PI/2, 0, -PI/2, 0, 0};
    ur3.init_Serial();
    ur3.moveJ(angle0);
    // ros::Duration(0.5).sleep();
    // ur3.moveJ(100, -50, 200, 0, 0, 0);
    ros::Duration(0.5).sleep();
    ros::Rate loop_rate(5);
    while(ros::ok()){
        ur3.publishMsg();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
