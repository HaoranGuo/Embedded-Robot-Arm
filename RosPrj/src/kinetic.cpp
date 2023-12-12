#include "../include/kinetic.h"

Kinetic::Kinetic(std::vector<double> a, std::vector<double> d, std::vector<double> alpha){
    /*
    %% DH表    a(i-1)     alpha(i-1)    di
    Arm1  =   [0,          0,        80  ];
    Arm2  =   [9.55,       -pi/2,    0  ];
    Arm3  =   [104,        0,        0  ];
    Arm4  =   [96.4,       0,        0  ];
    Arm5  =   [0,          -pi/2,     80.66];
    Arm6  =   [0,          pi/2,     27.5 ];
    */

    if (a.size() != d.size() || a.size() != alpha.size()){
        std::cout << "DH parameter error!" << std::endl;
        return;
    }

    _a = a;
    _d = d;
    _alpha = alpha;

    _T = Eigen::Matrix4d::Identity();

    cmd_sub = nh.subscribe("/robot_cmd", 1000, &Kinetic::cmdCallback, this);
}

Kinetic::~Kinetic(){
    ser.close();
}

void Kinetic::cmdCallback(const embedded_robot_arm::cmd::ConstPtr cmd){
    std::cout << "cmd received" << std::endl;
    // 获取cmd中的string数据，并储存
    std::string cmd_str = cmd->cmd;
    int mode = cmd->mode;
    std::vector<double> data = cmd->data;

    // 判断cmd_str的值，执行相应的操作
    if (cmd_str == "moveJ"){
        if (mode == 1){
            if (data.size() != 6){
                std::cout << "data size error!" << std::endl;
                return;
            }
            double angle[6] = {0};
            for (int i = 0; i < 6; ++i){
                angle[i] = data[i];
            }
            moveJ(angle);
        }
        else{
            if (data.size() != 6){
                std::cout << "data size error!" << std::endl;
                return;
            }
            moveJ(data[0], data[1], data[2], data[3], data[4], data[5]);
        }
    }
    else if (cmd_str == "moveL"){
        if (mode == 1){
            if (data.size() != 6){
                std::cout << "data size error!" << std::endl;
                return;
            }
            double angle[6] = {0};
            for (int i = 0; i < 6; ++i){
                angle[i] = data[i];
            }
            moveL(angle, 20);
        }
        else{
            if (data.size() != 6){
                std::cout << "data size error!" << std::endl;
                return;
            }
            moveL(data[0], data[1], data[2], data[3], data[4], data[5], 20);
        }
    }
    else if (cmd_str == "test"){
        std::cout << "Received test cmd" << std::endl;
        std::cout << "mode: " << mode << std::endl;
        for (int i = 0; i < data.size(); i++){
            std::cout << data[i] << " ";
        }
        std::cout << std::endl; 
    }
}

void Kinetic::printDH() {
    std::cout << "DH parameter: " << std::endl;
    std::cout << "a: ";
    for (int i = 0; i < _a.size(); ++i){
        std::cout << _a[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "d: ";
    for (int i = 0; i < _d.size(); ++i){
        std::cout << _d[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "alpha: ";
    for (int i = 0; i < _alpha.size(); ++i){
        std::cout << _alpha[i] << " ";
    }
    std::cout << std::endl;
}

Eigen::Matrix4d Kinetic::getTx(double a,double alpha,double d,double theta){
    Eigen::Matrix4d Tx;
    double T11 = cos(theta);
    double T12 = -1*sin(theta);
    double T14 = a;
    double T21 = sin(theta)*cos(alpha);
    double T22 = cos(theta)*cos(alpha);
    double T23 = -1*sin(alpha);
    double T24 = -1*d*sin(alpha);
    double T31 = sin(theta)*sin(alpha);
    double T32 = cos(theta)*sin(alpha);
    double T33 = cos(alpha);
    double T34 = d*cos(alpha);
    Tx << T11  ,T12  ,0    ,T14  ,
            T21  ,T22  ,T23  ,T24  ,
            T31  ,T32  ,T33  ,T34  ,
            0  ,  0  ,  0  ,  1;
    return Tx;
}

void Kinetic::setT(Eigen::Matrix4d T){
    _T = T;
    nx = _T(0,0);ox = _T(0,1);ax = _T(0,2);px = _T(0,3);
    ny = _T(1,0);oy = _T(1,1);ay = _T(1,2);py = _T(1,3);
    nz = _T(2,0);oz = _T(2,1);az = _T(2,2);pz = _T(2,3);
}

double Kinetic::normalize_angle(double angle, int flag) {
    if (flag == 1){
        if ((angle > 0 && angle < 1e-4) || (angle < 0 && angle > -1e-4)){
            angle = 0;
        }
        while (angle > PI) {
            angle -= 2 * PI;
        }
        while (angle < -PI) {
            angle += 2 * PI;
        }
        return angle;
    }
    else if (flag == 2){
        double temp = angle + PI/2;
        while (temp > PI) {
            temp -= 2 * PI;
        }
        while (temp < -PI) {
            temp += 2 * PI;
        }
        return temp - PI/2;
    }
    else if (flag == 3){
        if ((angle > 0 && angle < 1e-4)){
            angle = 0;
        }
        while (angle < 0){
            angle += PI;
        }
        while (angle > PI){
            angle -= PI;
        }
        return angle;
    }
    else{
        return angle;
    }
}

Eigen::Matrix<double, 4, 4> Kinetic::pose2T(Eigen::Matrix<double, 6, 1> pose){
    // 将pose的后三个元素转换为旋转矩阵
    Eigen::Matrix<double, 3, 3> R;
    R = Eigen::AngleAxisd(pose(3), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pose(4), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(pose(5), Eigen::Vector3d::UnitZ());
    
    // 将pose的前三个元素转换为平移矩阵
    Eigen::Matrix<double, 3, 1> t;
    t << pose(0), pose(1), pose(2);

    // 将R和t转换为T
    Eigen::Matrix<double, 4, 4> T;
    T << R, t,
         0, 0, 0, 1;

    return T;
}

Eigen::Matrix<double, 4, 4> Kinetic::pose2T(double x, double y, double z, double rx, double ry, double rz){
    Eigen::Matrix<double, 6, 1> pose;
    pose << x, y, z, rx, ry, rz;
    return pose2T(pose);
}

Eigen::Matrix<double, 6, 1> Kinetic::T2pose(Eigen::Matrix<double, 4, 4> T){
    Eigen::Matrix<double, 6, 1> pose;
    pose << T(0, 3), T(1, 3), T(2, 3), 
            atan2(T(2, 1), T(2, 2)), 
            atan2(-T(2, 0), sqrt(T(2, 1)*T(2, 1) + T(2, 2)*T(2, 2))), 
            atan2(T(1, 0), T(0, 0));
    return pose;
}

bool Kinetic::forward_Kine(std::vector<double> theta, Eigen::Matrix4d &T){
    try {
        Eigen::Matrix4d T10 = getTx(_a[0], _alpha[0], _d[0], theta[0]);
        Eigen::Matrix4d T21 = getTx(_a[1], _alpha[1], _d[1], theta[1]);
        Eigen::Matrix4d T32 = getTx(_a[2], _alpha[2], _d[2], theta[2]);
        Eigen::Matrix4d T43 = getTx(_a[3], _alpha[3], _d[3], theta[3]);
        Eigen::Matrix4d T54 = getTx(_a[4], _alpha[4], _d[4], theta[4]);
        Eigen::Matrix4d T65 = getTx(_a[5], _alpha[5], _d[5], theta[5]);
        _T = T10 * T21 * T32 * T43 * T54 * T65;
        T = _T;
        nx = _T(0, 0);
        ox = _T(0, 1);
        ax = _T(0, 2);
        px = _T(0, 3);
        ny = _T(1, 0);
        oy = _T(1, 1);
        ay = _T(1, 2);
        py = _T(1, 3);
        nz = _T(2, 0);
        oz = _T(2, 1);
        az = _T(2, 2);
        pz = _T(2, 3);
        return true;
    }
    catch (std::exception& e) {
        std::cout << "forward_Kine error: " << e.what() << std::endl;
        return false;
    }
}

std::vector<double> Kinetic::getTheta3(double theta1, double theta6){
    std::vector<double> theta3;
    // 这里确实没问题
    double a = _d[4]*(sin(theta6)*(cos(theta1)*nx+sin(theta1)*ny)+cos(theta6)*(cos(theta1)*ox+sin(theta1)*oy))-_d[5]*(cos(theta1)*ax+sin(theta1)*ay)+cos(theta1)*px+sin(theta1)*py-_a[0];
    double b = _d[4]*(sin(theta6)*nz+cos(theta6)*oz)-_d[5]*az+pz-_d[0];

    double m = (a*a+b*b-_a[1]*_a[1]-_a[2]*_a[2]);
    double n = (2*_a[2]*_a[1]);

//    std::cout << "m: " << m << std::endl;
//    std::cout << "n: " << n << std::endl;

    double theta31 = std::acos(m/n);
    double theta32 = -theta31;
    theta3.clear();
    theta3.push_back(theta31);
    theta3.push_back(theta32);
    return theta3;
}


double Kinetic::getTheta2(double theta1,double theta3,double theta6){
    double a = _d[4]*(sin(theta6)*(cos(theta1)*nx+sin(theta1)*ny)+cos(theta6)*(cos(theta1)*ox+sin(theta1)*oy))-_d[5]*(cos(theta1)*ax+sin(theta1)*ay)+cos(theta1)*px+sin(theta1)*py-_a[0];
    double b = _d[4]*(sin(theta6)*nz+cos(theta6)*oz)-_d[5]*az+pz-_d[0];
    double m = (b*(_a[2]*cos(theta3)+_a[1])-_a[2]*sin(theta3)*a);
    double n = (a*(_a[2]*cos(theta3)+_a[1])+_a[2]*sin(theta3)*b);
    double theta2 = std::atan2(m,n);
    return theta2;
}

double Kinetic::getTheta4(double theta1,double theta2,double theta3,double theta6){
    double m = -1*sin(theta6)*(cos(theta1)*nx+sin(theta1)*ny)-cos(theta6)*(cos(theta1)*ox+sin(theta1)*oy);
    double n = nz*sin(theta6)+oz*cos(theta6);
    //因为atan(x)所求的结果落在-PI/2到PI/2上,theta2+theta3+theta4显然不一定落在此区域上
    double theta234;
    if ((m>0 && n<0)){
        theta234 = PI+atan(m/n);
    }
    else if (m<0 && n<0){
        theta234 = atan(m/n)-PI;
    }
    else{
        theta234 = atan(m/n);
    }
    //因为theta2+theta3+theta4的取值范围可以到[-540,540] (该范围可以在后期传入范围参数确定)
    double theta4;
    //尝试覆盖[-540,540]所有theta2+theta3+theta4的可能性
    if (2*PI+theta234-theta2-theta3<PI && 2*PI+theta234>-1*PI){
        theta4 = theta234+2*PI-theta2-theta3;
    }
    else if (-2*PI+theta234-theta2-theta3<PI && -2*PI+theta234>-1*PI){
        theta4 = theta234-2*PI-theta2-theta3;
    }
    else
        theta4 = theta234-theta2-theta3;
    return theta4;
}

bool Kinetic::inverse_kine(){
    try{
        // theta1
        // std::cout << "theta1: " << std::endl;

        double m = py-_d[5]*ay;
        double n = _d[5]*ax-px;
        double phi = std::atan2(m,n);
        double theta11 = std::atan2(-1*_d[3],sqrt(m*m+n*n-_d[3]*_d[3]))-phi;
        double theta12 = std::atan2(-1*_d[3],-1*sqrt(m*m+n*n-_d[3]*_d[3]))-phi;

        theta11 = normalize_angle(theta11, 1);
        theta12 = normalize_angle(theta12, 1);

        // theta5
        // std::cout << "theta5: " << std::endl;

        double theta51 = acos(sin(theta11)*ax-cos(theta11)*ay);
        double theta52 = -1*acos(sin(theta11)*ax-cos(theta11)*ay);
        double theta53 = acos(sin(theta12)*ax-cos(theta12)*ay);
        double theta54 = -1*acos(sin(theta12)*ax-cos(theta12)*ay);

        theta51 = normalize_angle(theta51, 1);
        theta52 = normalize_angle(theta52, 1);
        theta53 = normalize_angle(theta53, 1);
        theta54 = normalize_angle(theta54, 1);

        // theta6
        // std::cout << "theta6: " << std::endl;

        double theta61,theta62,theta63,theta64;

        double fenzi1 = (-1 * sin(theta11) * ox + cos(theta11) * oy);
        double fenmu1 = (sin(theta11) * nx - cos(theta11) * ny);
//        theta63 = std::atan2(fenzi1, fenmu1);
        theta61 = std::atan2(fenzi1, fenmu1);

        double fenzi2 = (-1 * sin(theta11) * ox + cos(theta11) * oy);
        double fenmu2 = (sin(theta11) * nx - cos(theta11) * ny);
        theta64 = std::atan2(fenzi2, fenmu2);
//        theta62 = std::atan2(fenzi2, fenmu2);

        double fenzi3 = (-1 * sin(theta12) * ox + cos(theta12) * oy);
        double fenmu3 = (sin(theta12) * nx - cos(theta12) * ny);
//        theta61 = std::atan2(fenzi3, fenmu3);
        theta63 = std::atan2(fenzi3, fenmu3);

        double fenzi4 = (-1 * sin(theta12) * ox + cos(theta12) * oy);
        double fenmu4 = (sin(theta12) * nx - cos(theta12) * ny);
        theta62 = std::atan2(fenzi4, fenmu4);
//        theta64 = std::atan2(fenzi4, fenmu4);

        // theta3
        // std::cout << "theta3: " << std::endl;

        std::vector<double> theta31_t = getTheta3(theta11, theta61);
        std::vector<double> theta32_t = getTheta3(theta11, theta62);
        std::vector<double> theta33_t = getTheta3(theta12, theta63);
        std::vector<double> theta34_t = getTheta3(theta12, theta64);

        double theta31, theta32, theta33, theta34, theta35, theta36, theta37, theta38;

        theta31 = normalize_angle(theta31_t[0], 1);
        theta32 = normalize_angle(theta31_t[1], 1);
        theta33 = normalize_angle(theta32_t[0], 1);
        theta34 = normalize_angle(theta32_t[1], 1);
        theta35 = normalize_angle(theta33_t[0], 1);
        theta36 = normalize_angle(theta33_t[1], 1);
        theta37 = normalize_angle(theta34_t[0], 1);
        theta38 = normalize_angle(theta34_t[1], 1);

        // theta2
        // std::cout << "theta2: " << std::endl;

        double theta21 = getTheta2(theta11,theta31,theta61);
        double theta22 = getTheta2(theta11,theta32,theta61);
        double theta23 = getTheta2(theta11,theta33,theta62);
        double theta24 = getTheta2(theta11,theta34,theta62);
        double theta25 = getTheta2(theta12,theta35,theta63);
        double theta26 = getTheta2(theta12,theta36,theta63);
        double theta27 = getTheta2(theta12,theta37,theta64);
        double theta28 = getTheta2(theta12,theta38,theta64);

        // theta4
        // std::cout << "theta4: " << std::endl;

        double theta41 = getTheta4(theta11,theta21,theta31,theta61);
        double theta42 = getTheta4(theta11,theta22,theta32,theta61);
        double theta43 = getTheta4(theta11,theta23,theta33,theta62);
        double theta44 = getTheta4(theta11,theta24,theta34,theta62);
        double theta45 = getTheta4(theta12,theta25,theta35,theta63);
        double theta46 = getTheta4(theta12,theta26,theta36,theta63);
        double theta47 = getTheta4(theta12,theta27,theta37,theta64);
        double theta48 = getTheta4(theta12,theta28,theta38,theta64);

        theta41 = normalize_angle(theta41, 2);
        theta42 = normalize_angle(theta42, 2);
        theta43 = normalize_angle(theta43, 2);
        theta44 = normalize_angle(theta44, 2);
        theta45 = normalize_angle(theta45, 2);
        theta46 = normalize_angle(theta46, 2);
        theta47 = normalize_angle(theta47, 2);
        theta48 = normalize_angle(theta48, 2);

        theta21 = normalize_angle(theta21, 2);
        theta22 = normalize_angle(theta22, 2);
        theta23 = normalize_angle(theta23, 2);
        theta24 = normalize_angle(theta24, 2);
        theta25 = normalize_angle(theta25, 2);
        theta26 = normalize_angle(theta26, 2);
        theta27 = normalize_angle(theta27, 2);
        theta28 = normalize_angle(theta28, 2);

        theta61 = normalize_angle(theta61, 3);
        theta62 = normalize_angle(theta62, 3);
        theta63 = normalize_angle(theta63, 3);
        theta64 = normalize_angle(theta64, 3);

        // Get the result
        _result.clear();

        std::vector<double> middle_result;
        middle_result.push_back(theta11);
        middle_result.push_back(theta21);
        middle_result.push_back(theta31);
        middle_result.push_back(theta41);
        middle_result.push_back(theta51);
        middle_result.push_back(theta61);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta11);
        middle_result.push_back(theta22);
        middle_result.push_back(theta32);
        middle_result.push_back(theta42);
        middle_result.push_back(theta51);
        middle_result.push_back(theta61);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta11);
        middle_result.push_back(theta23);
        middle_result.push_back(theta33);
        middle_result.push_back(theta43);
        middle_result.push_back(theta52);
        middle_result.push_back(theta62);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta11);
        middle_result.push_back(theta24);
        middle_result.push_back(theta34);
        middle_result.push_back(theta44);
        middle_result.push_back(theta52);
        middle_result.push_back(theta62);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta12);
        middle_result.push_back(theta25);
        middle_result.push_back(theta35);
        middle_result.push_back(theta45);
        middle_result.push_back(theta53);
        middle_result.push_back(theta63);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta12);
        middle_result.push_back(theta26);
        middle_result.push_back(theta36);
        middle_result.push_back(theta46);
        middle_result.push_back(theta53);
        middle_result.push_back(theta63);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta12);
        middle_result.push_back(theta27);
        middle_result.push_back(theta37);
        middle_result.push_back(theta47);
        middle_result.push_back(theta54);
        middle_result.push_back(theta64);
        _result.push_back(middle_result);

        middle_result.clear();
        middle_result.push_back(theta12);
        middle_result.push_back(theta28);
        middle_result.push_back(theta38);
        middle_result.push_back(theta48);
        middle_result.push_back(theta54);
        middle_result.push_back(theta64);
        _result.push_back(middle_result);

        return true;
    }
    catch (std::exception& e) {
        std::cout << "inverse_kine error: " << e.what() << std::endl;
        return false;
    }
}

bool Kinetic::inverse_kine(Eigen::Matrix4d T, std::vector<std::vector<double>> &result){
    try{
        setT(T);
        return inverse_kine(result);
    }
    catch (std::exception& e) {
        std::cout << "inverse_kine error: " << e.what() << std::endl;
        return false;
    }
}

bool Kinetic::inverse_kine(std::vector<std::vector<double>> &result){
    try{
        inverse_kine();
        result = _result;
        return true;
    }
    catch (std::exception& e) {
        std::cout << "inverse_kine error: " << e.what() << std::endl;
        return false;
    }
}

void Kinetic::printResult() {
    std::cout << "Result: " << std::endl;
    for (int i = 0; i < _result.size(); ++i){
        for (int j = 0; j < _result[i].size(); ++j){
            std::cout << _result[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Kinetic::printT() {
    std::cout << "T: " << std::endl;
    std::cout << _T << std::endl;
}

bool Kinetic::get_possible_angle() {
    _possible_result.clear();
    if(inverse_kine()){
        _possible_result.clear();
        for (int i = 0; i < _result.size(); ++i){
            if (_result[i].size() != 6){
                continue;
            }
            int cpulse = SERVO0_MID_PULSE + 2000 / SERVO_RANGE * _result[i][0];
            if (cpulse < 500 || cpulse > 2500){
                continue;
            }
            if (_result[i][1] < - PI + PI/18 || _result[i][1] > - PI/6){
                continue;
            }
            cpulse = SERVO2_MID_PULSE - 2000 / SERVO_RANGE * _result[i][2];
            if (cpulse < 500 || cpulse > 2500){
                continue;
            }
            cpulse = SERVO3_MID_PULSE - 2000 / SERVO_RANGE * (_result[i][3] + PI/2);
            if (cpulse < 500 || cpulse > 2500){
                continue;
            }
            cpulse = SERVO4_MID_PULSE + 2000 / SERVO_RANGE * _result[i][4];
            if (cpulse < 500 || cpulse > 2500){
                continue;
            }
            cpulse = 500 + 2000 / SERVO_RANGE * _result[i][5];
            if (cpulse < 500 || cpulse > 2500){
                continue;
            }
            _possible_result.push_back(_result[i]);
        }

        if (_possible_result.size() > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

bool Kinetic::get_best_angle() {
    _best_angle.clear();
    if (get_possible_angle()){
        _best_angle.clear();
        double min_distance = 1000000;
        int min_index = 0;
        for (int i = 0; i < _possible_result.size(); ++i){
            double distance = 0;
            for (int j = 0; j < _possible_result[i].size(); ++j){
                distance += (_possible_result[i][j] - _last_angle[j]) * (_possible_result[i][j] - _last_angle[j]);
            }
            if (distance < min_distance){
                min_distance = distance;
                min_index = i;
            }
        }
        _best_angle = _possible_result[min_index];
        _last_angle = _best_angle;
        return true;
    }
    else{
        return false;
    }
}

bool Kinetic::get_best_angle(std::vector<double> &best_angle) {
    if (get_best_angle()){
        best_angle = _best_angle;
        return true;
    }
    else{
        return false;
    }
}

void Kinetic::serialMove(std::vector<double> cal_angle) {
    if(cal_angle.size() != 6){
        std::cout << "cal_angle size error!" << std::endl;
        return;
    }
    int pulse[6] = {0};
    pulse[0] = SERVO0_MID_PULSE + 2000 / SERVO_RANGE * cal_angle[0];
    pulse[1] = SERVO1_MID_PULSE + 2000 / SERVO_RANGE * (cal_angle[1] + PI/2);
    pulse[2] = SERVO2_MID_PULSE - 2000 / SERVO_RANGE * cal_angle[2];
    pulse[3] = SERVO3_MID_PULSE - 2000 / SERVO_RANGE * (cal_angle[3] + PI/2);
    pulse[4] = SERVO4_MID_PULSE + 2000 / SERVO_RANGE * cal_angle[4];
    pulse[5] = 500 + 2000 / SERVO_RANGE * cal_angle[5];
    for (int i = 0; i < 6; ++i){
        if (pulse[i] < 500){
            return;
        }
        else if (pulse[i] > 2500){
            return;
        }
    }
    servos_move(pulse);
}

bool Kinetic::moveJ(double angle[6]){
    std::vector<double> angle_t;
    for (int i = 0; i < 6; ++i){
        angle_t.push_back(angle[i]);
    }
    serialMove(angle_t);
    _last_angle = angle_t;
    forward_Kine(angle_t, _T);
    Eigen::Matrix<double, 6, 1> pose;
    pose = T2pose(_T);
    for (int i = 0; i < 6; ++i){
        _last_pose[i] = pose[i];
    }
    return true;
}

bool Kinetic::moveJ(double x, double y, double z, double rx, double ry, double rz) {
    Eigen::Matrix<double, 4, 4> T = pose2T(x, y, z, rx, ry, rz);
    setT(T);
    if (get_best_angle()){
        serialMove(_best_angle);
        _last_angle = _best_angle;
        _last_pose[0] = x;
        _last_pose[1] = y;
        _last_pose[2] = z;
        _last_pose[3] = rx;
        _last_pose[4] = ry;
        _last_pose[5] = rz;
        return true;
    }
    else{
        return false;
    }
}

bool Kinetic::moveJ(Eigen::Matrix<double, 4, 4> T) {
    setT(T);
    if (get_best_angle()){
        serialMove(_best_angle);
        _last_angle = _best_angle;
        Eigen::Matrix<double, 6, 1> pose;
        pose = T2pose(T);
        for (int i = 0; i < 6; ++i){
            _last_pose[i] = pose[i];
        }
        return true;
    }
    else{
        return false;
    }
}

bool Kinetic::servos_move(int pulse[6]){
    // 将角度转换为字符串，格式为：+servos,angle1,angle2,angle3,angle4,angle5,angle6,\r\n
    for (int i = 0; i < 6; i++){
        if(pulse[i] < 500 || pulse[i] > 2500){
            return false;
        }
    }
    std::string s = "+servos,";
    for(int i = 0; i < 6; i++){
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

void Kinetic::init_Serial(){
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

void Kinetic::printPossibleResult(){
    std::cout << "Possible Result: " << std::endl;
    for (int i = 0; i < _possible_result.size(); ++i){
        for (int j = 0; j < _possible_result[i].size(); ++j){
            std::cout << _possible_result[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Kinetic::printBestResult(){
    std::cout << "Best Result: " << std::endl;
    for (int i = 0; i < _best_angle.size(); ++i){
        std::cout << _best_angle[i] << " ";
    }
    std::cout << std::endl;
}



bool Kinetic::moveL(double x, double y, double z, double rx, double ry, double rz, int speed){
    // Eigen::Matrix4d last_T;
    // if (forward_Kine(_last_angle, last_T)){
        int cnt = 0;
        // Eigen::Matrix<double, 6, 1> pose_t = T2pose(last_T);

        double tx, ty, tz, trx, tryy, trz;
        // 插值，总共插值20次
        for (int i = 0; i < speed; ++i){
            // Eigen::Matrix<double, 6, 1> pose;
            // pose << pose_t(0) + (x - pose_t(0)) / 20 * i, 
            //         pose_t(1) + (y - pose_t(1)) / 20 * i, 
            //         pose_t(2) + (z - pose_t(2)) / 20 * i, 
            //         pose_t(3) + (rx - pose_t(3)) / 20 * i, 
            //         pose_t(4) + (ry - pose_t(4)) / 20 * i, 
            //         pose_t(5) + (rz - pose_t(5)) / 20 * i;
            // Eigen::Matrix<double, 4, 4> T = pose2T(pose);
            tx = _last_pose[0] + (x - _last_pose[0]) / speed * i;
            ty = _last_pose[1] + (y - _last_pose[1]) / speed * i;
            tz = _last_pose[2] + (z - _last_pose[2]) / speed * i;
            trx = _last_pose[3] + (rx - _last_pose[3]) / speed * i;
            tryy = _last_pose[4] + (ry - _last_pose[4]) / speed * i;
            trz = _last_pose[5] + (rz - _last_pose[5]) / speed * i;

            // std::cout << i << std::endl;
            if (!moveJ(tx, ty, tz, trx, tryy, trz)){
                cnt++;
                std::cout << "moveL cannot reach!" << cnt << std::endl;

                if (cnt >= 5){
                    std::cout << "moveL error!" << std::endl;
                    return false;
                }

                continue;
            }

            // 延迟0.05s
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (i >= speed - 1){
                return true;
            }
        }
    // }
    // else{
    //     return false;
    // }
}

bool Kinetic::moveL(Eigen::Matrix<double, 4, 4> T, int speed){
    Eigen::Matrix4d last_T;
    Eigen::Matrix<double, 6, 1> current_pose = T2pose(T);
    if (moveL(current_pose[0], current_pose[1], current_pose[2], current_pose[3], current_pose[4], current_pose[5], speed)){
        return true;
    }
    else{
        return false;
    }
}

bool Kinetic::moveL(double angle[6], int speed){
    std::vector<double> angle_t;
    for (int i = 0; i < 6; ++i){
        angle_t.push_back(angle[i]);
    }
    Eigen::Matrix4d current_T;
    if (forward_Kine(angle_t, current_T)){
        return moveL(current_T, speed);
    }
    else{
        return false;
    }
}

void Kinetic::draw_word(const char *path, double z, double scale, bool flag){
    // 读取txt文件，其中每行分别为x,y,flag
    std::ifstream infile;
    infile.open(path);
    std::string line;
    std::vector<std::vector<double>> points;
    while (std::getline(infile, line)){
        std::istringstream iss(line);
        std::vector<double> point;
        double x, y, flag;
        if (!(iss >> x >> y >> flag)){
            break;
        }
        point.push_back((2-y)*scale);
        point.push_back((0.5-x)*scale);
        point.push_back(flag);
        points.push_back(point);
    }

    for(int i = 0; i < points.size(); i++){
        std::cout << "points " << i << ": " << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;

        if (i == 0){
            std::cout << "1" << std::endl;
            moveJ(points[i][0], points[i][1], z + 10, PI, 0, 0);
            // 延迟1s
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::cout << "2" << std::endl;
            moveL(points[i][0], points[i][1], z, PI, 0, 0, 10);
            std::cout << "3" << std::endl;
        }
        else{
            if (points[i][2] == 0){
                if (flag){
                    double dist = sqrt((points[i][0] - points[i-1][0])*(points[i][0] - points[i-1][0]) + (points[i][1] - points[i-1][1])*(points[i][1] - points[i-1][1]));
                    int speed = int(dist * 2);
                    if (speed < 10){
                        speed = 10;
                    }
                    moveL(points[i][0], points[i][1], z, PI, 0, 0, speed);  
                }
                else{
                    moveL(points[i][0], points[i][1], z, PI, 0, 0, 20);
                }
            }
            else{
                moveL(points[i-1][0], points[i-1][1], z + 10, PI, 0, 0, 5);
                moveL(points[i][0], points[i][1], z + 10, PI, 0, 0, 5);
                moveL(points[i][0], points[i][1], z, PI, 0, 0, 10);
            }
        }
    }
}