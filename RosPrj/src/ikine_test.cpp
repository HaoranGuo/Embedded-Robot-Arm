#include "../include/kinetic.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "ikine_test");
    ros::NodeHandle nh;


    /*
    改进DH表    a(i-1)     alpha(i-1)    di
    Arm1  =   [0,          0,        80  ];
    Arm2  =   [-9.55,      pi/2,    0  ];
    Arm3  =   [104,        0,        0  ];
    Arm4  =   [96.4,       0,        0  ];
    Arm5  =   [0,          pi/2,     80.66];
    Arm6  =   [0,          -pi/2,     27.5 ];
    */

    /*
    这里用的应该是标准DH表来计算机器人，参考UR3的标准DH进行修改
    标准DH表    d(i)    a(i)    alpha(i)
    1           0       9.55    PI/2
    2           0       -104    0
    3           0       -96.4   0
    4           0       0       PI/2
    5           80.66   0       PI/2
    6           27.5    0       0
    */
    std::vector<double> a = {9.55, -104, -96.4, 0, 0, 0};
    std::vector<double> d = {0, 0, 0, 0, 80.66, 27.5};
    std::vector<double> alpha = {PI/2, 0, 0, PI/2, PI/2, 0};

    Kinetic ur3(a, d, alpha);

    // ur3.printDH();

     Eigen::Matrix4d T;
     // T << 1, 0, 0, 50,
     //      0, 0, -1, 30,
     //      0, 1, 0, 20,
     //      0, 0, 0, 1;
    //  T = ur3.pose2T(50, 30, 50, 0, 0, 0);

    //  ur3.setT(T);
    //  ur3.printT();

//     std::vector<std::vector<double>> result;
//     ur3.get_possible_angle();
//     ur3.printResult();
//     ur3.printPossibleResult();

    double angle0[6] = {0, -PI/2, 0, -PI/2, 0, 0};
    // double angle1[6] = {0, -PI/3 * 2, PI/2, -PI/2, 0, 0};

    ur3.init_Serial();

    ur3.moveJ(angle0);

    ros::Duration(1).sleep();

    ur3.moveJ(200, 0, 60, PI, 0, 0);

    ros::Duration(1).sleep();

    while(ros::ok()){
        ur3.draw_word("/home/haoran/catkin_ws/src/embeddedrobotarm/data/hust.txt", 50, 120, 0);
        ros::spin();
    }

    // while(ros::ok()){
    //     // ur3.moveJ(T);
    //     ur3.moveJ(250, 0, 0, PI, 0, 0);
    //     ur3.printResult();
    //     ur3.printPossibleResult();

    //     ros::Duration(2).sleep();

    //     ur3.moveL(200, 0, 0, PI, 0, 0);

    //     ros::Duration(2).sleep();

    //     ur3.moveL(200, 50, 0, PI, 0, 0);

    //     // ur3.moveJ(-50, 0, 100, PI, 0, 0);
    //     // ur3.printResult();
    //     // ur3.printPossibleResult();

    //     // 延时2s
    //     // ros::Duration(2).sleep();

    //     // ur3.moveL(150, 0, 80, 0, 0, 0);

    //     // ur3.moveJ(100, 0, 80, PI, 0, -PI/2);

    //     // ur3.printResult();
    //     // ur3.printPossibleResult();
    //     // ur3.printBestResult();

    //     // ros::Duration(2).sleep();

    //     // ur3.moveJ(160, -50, 100, 0, 0, 0);
    //     // ur3.printResult();
    //     // ur3.printPossibleResult();
    //     // ur3.printBestResult();

    //     ros::spin();
    // }
    return 0;
}