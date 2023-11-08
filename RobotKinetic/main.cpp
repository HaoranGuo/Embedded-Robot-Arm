#include "kinetic.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main(int argc, char** argv){

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
    1           0       -9.55    PI/2
    2           0       -104    0
    3           0       -96.4   0
    4           0       0       PI/2
    5           80.66   0       -PI/2
    6           27.5    0       0
    */
    std::vector<double> a = {-9.55, -104, -96.4, 0, 0, 0};
    std::vector<double> d = {0, 0, 0, 0, 80.66, 27.5};
    std::vector<double> alpha = {PI/2, 0, 0, PI/2, -PI/2, 0};

//    std::vector<double> a = {0, -425, -392, 0, 0, 0};
//    std::vector<double> d = {89.2, 0, 0, 109.3, 94.75, 82.5};
//    std::vector<double> alpha = {PI/2, 0, 0, PI/2, -PI/2, 0};

    Kinetic ur3(a, d, alpha);

    // ur3.printDH();

    Eigen::Matrix4d T;
//     T << 1, 0, 0, 50,
//          0, 1, 0, 40,
//          0, 0, 1, 50,
//          0, 0, 0, 1;
    T = ur3.pose2T(180, 40, 50, PI/4, -PI/6, PI/5);
    //T = ur3.pose2T(50, 30, 50, 0, 0, 0);

    ur3.setT(T);
    ur3.printT();

    std::vector<std::vector<double>> result;
    ur3.inverse_kine(result);
    ur3.get_possible_angle();
    ur3.printResult();
    ur3.printPossibleResult();

    Eigen::Matrix4d res;

    for (int i = 0; i < ur3._result.size(); ++i){
        if (ur3.forward_Kine(ur3._result[i], res)){
            std::cout << i << " " << "res:" << std::endl;
            std::cout << res << std::endl;
        }
    }

    return 0;
}