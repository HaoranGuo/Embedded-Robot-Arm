#include "../include/serial_node.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    SerialNode serial_node;

    serial_node.init_Serial();

    ros::Rate loop_rate(2);

    int servo_pulse[USED_SERVO_NUM] = {90, 90, 90, 90, 90, 90};

    serial_node.servos_move(servo_pulse);

    // while(ros::ok()){
        
    //     serial_node.led_control(1, 1);
    //     loop_rate.sleep();
    // }

    while(ros::ok()){

    }

    return 0;

}