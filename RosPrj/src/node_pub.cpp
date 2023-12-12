#include <ros/ros.h>
#include <iostream>
#include <string>
#include <embedded_robot_arm/cmd.h>
#include <vector>

#define PI (3.14159265358979324f)

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_node_pub");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<embedded_robot_arm::cmd>("/robot_cmd", 1000);
    ros::Rate loop_rate(2);
    embedded_robot_arm::cmd cmd;
    cmd.cmd = "moveJ";
    cmd.mode = 1;
    cmd.data.resize(6);
    cmd.data[0] = 0;
    cmd.data[1] = -PI/2;
    cmd.data[2] = 0;
    cmd.data[3] = -PI/2;
    cmd.data[4] = 0;
    cmd.data[5] = 0;
    while(ros::ok()){
        cmd.data[2] -= 0.1;
        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}