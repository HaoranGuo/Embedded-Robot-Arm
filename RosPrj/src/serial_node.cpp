#include "../include/serial_node.h"


SerialNode::~SerialNode()
{
    ser.close();
}

void SerialNode::init_Serial(){
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.setBytesize(serial::eightbits);
        ser.setParity(serial::parity_none);
        ser.setStopbits(serial::stopbits_one);
        ser.open();
    }
    catch(serial::IOException &e)
    {
        std::cout << "Failed to open port" << std::endl;
    }
    std::cout << "Succeed to open port" << std::endl;
}

void SerialNode::send(unsigned char* s, int len){
    ser.write(s, len);
}

void SerialNode::led_control(int led_num, int led_status){
    unsigned char s[] = "+led,1,1,\r\n";
    s[5] = led_num + '0';
    s[7] = led_status + '0';
    ser.write(s, sizeof(s));
    std::cout << "LED: " << led_num << " is now on status: " << led_status << std::endl;
}

bool SerialNode::servos_move(int pulse[USED_SERVO_NUM]){
    // 将角度转换为字符串，格式为：+servos,angle1,angle2,angle3,angle4,angle5,angle6,\r\n
    for (int i = 0; i < USED_SERVO_NUM; i++){
        if(pulse[i] < 500 || pulse[i] > 2500){
            return false;
        }
    }
    std::string s = "+servos,";
    for(int i = 0; i < USED_SERVO_NUM; i++){
        std::stringstream ss;
        ss << pulse[i];
        s += ss.str();
        s += ",";
    }
    s += "\r\n";
    
    // 将字符串转换为unsigned char数组
    unsigned char s_buffer[s.size()];
    for(int i = 0; i < s.size(); i++){
        s_buffer[i] = s[i];
    }
    ser.write(s_buffer, sizeof(s_buffer));
    // std::cout << "Servos are now at: ";
    // for(int i = 0; i < USED_SERVO_NUM; i++){
    //     std::cout << angle[i] << " ";
    // }
    return true;
}