#ifndef __KINETIC_EMBEDDED_ROBOT_ARM_H__
#define __KINETIC_EMBEDDED_ROBOT_ARM_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>
#include <thread>
#include "serial/serial.h"

#define PI (3.14159265358979324f)

#define SERVO_RANGE (4.71238898038469f)

#define SERVO0_MID_PULSE 1440
#define SERVO1_MID_PULSE 1500
#define SERVO2_MID_PULSE 1470
#define SERVO3_MID_PULSE 1480
#define SERVO4_MID_PULSE 1500
#define SERVO5_MID_PULSE 1500


class Kinetic{
    private:
        std::vector<double> _a;
        std::vector<double> _d;
        std::vector<double> _alpha;
        Eigen::Matrix4d _T;
        std::vector<std::vector<double>> _result;
        std::vector<std::vector<double>> _possible_result;
        std::vector<double> _last_angle;
        std::vector<double> _best_angle;

        serial::Serial ser;

        double nx, ox, ax, px;
        double ny, oy, ay, py;
        double nz, oz, az, pz;

        std::vector<double> getTheta3(double theta1, double theta6);
        double getTheta2(double theta1,double theta3,double theta6);
        double getTheta4(double theta1,double theta2,double theta3,double theta6);
        double normalize_angle(double angle, int flag);

    public:
        Kinetic(std::vector<double> a, std::vector<double> d, std::vector<double> alpha);
        ~Kinetic();
        bool inverse_kine(Eigen::Matrix4d T, std::vector<std::vector<double>> &result);
        bool forward_Kine(std::vector<double> theta, Eigen::Matrix4d &T);
        bool inverse_kine(std::vector<std::vector<double>> &result);
        bool inverse_kine();

        bool get_possible_angle();
        bool get_best_angle();
        bool get_best_angle(std::vector<double> &best_angle);

        Eigen::Matrix<double, 4, 4> pose2T(Eigen::Matrix<double, 6, 1> pose);
        Eigen::Matrix<double, 4, 4> pose2T(double x, double y, double z, double rx, double ry, double rz);
        Eigen::Matrix<double, 6, 1> T2pose(Eigen::Matrix<double, 4, 4> T);

        Eigen::Matrix<double, 6, 1> get_angle();
        Eigen::Matrix<double, 6, 1> get_pose();

        Eigen::Matrix4d getTx(double a,double alpha,double d,double theta);

        void printDH();
        void printT();
        void printResult();
        void printPossibleResult();
        void printBestResult();
        void setT(Eigen::Matrix4d T);
        void serialMove(std::vector<double> cal_angle);

        bool moveJ(double angle[6]);
        bool moveJ(double x, double y, double z, double rx, double ry, double rz);
        bool moveJ(Eigen::Matrix<double, 4, 4> T);

        bool moveL(double angle[6]);
        bool moveL(double x, double y, double z, double rx, double ry, double rz);
        bool moveL(Eigen::Matrix<double, 4, 4> T);

        bool servos_move(int pulse[6]);
        void init_Serial();

};

#endif