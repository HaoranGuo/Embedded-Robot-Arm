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
    ros::Duration(1).sleep();
    while(ros::ok()){
        ros::spin();
    }
    return 0;
}
