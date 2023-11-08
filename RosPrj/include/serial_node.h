#ifndef __SERIAL_NODE_H__
#define __SERIAL_NODE_H__

#include <iostream>
#include <string>
#include <sstream>

#include "std_msgs/String.h"
#include "serial/serial.h"

#define USED_SERVO_NUM 6

class SerialNode
{
    private:
        serial::Serial ser;

    public:
        ~SerialNode();

        void init_Serial();
        void send(unsigned char* s, int len);

        // led_num: 0, 1
        // led_status: 0->off, 1->on
        void led_control(int led_num, int led_status);

        // servo_num: 0, 1, 2, 3, 4, 5, 6(option), 7(option)
        // servo_angle: 0~270 (double)
        void servo_move(int servo_num, double servo_angle);

        // angle: 0~270 (double)
        bool servos_move(int pulse[USED_SERVO_NUM]);
};

#endif // __SERIAL_NODE_H__