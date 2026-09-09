//////////////
// INCLUDES //
//////////////

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <queue>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <ublox_msgs/NavPVT.h>
#include <erp42_msgs/DriveCmd.h>
#include <erp42_msgs/ModeCmd.h>
#include <erp42_msgs/SerialFeedBack.h>

#include "rddf.h"

/////////////
// DEFINES //
/////////////

#define PI_ 3.141592653f
#define CONTROL_FREQUENCY 16.0f
#define CONTROL_PERIOD 1.0f/CONTROL_FREQUENCY

#define DEFAULT_SPEED 5.0f
#define DEFAULT_LD 1.0f
#define DEFAULT_RDDF "2025-2-11_17-24_rddf-01.txt"



/////////////////////
// NAMESPACE SETUP //
/////////////////////

using namespace std;



///////////////////////
// ENUMS AND STRUCTS //
///////////////////////

enum SectionType {
    STOP = 0,
    GPS_NAVIGATION = 1,
    LANE_FOLLOWING = 2,
    LIDAR_ONLY = 3,
    TEST_STEERING = 99
};

struct CarState {
    // spec
    float wheelbase;
    float wheel_radius;
    float gps_to_rear;
    // position
    double x;
    double y;
    float heading;
    double last_position_update_time;
    // other states
    float speed;
    int idx_current;
    // cmd
    int cmd_steer;
    int cmd_speed;
    // feedback
    int feedback_steer;
    int feedback_speed;
    // gps
    float gps_speed;
};

struct PurePursuitConfig {
    float lookahead;
    float speed;
    float timeahead;
    float k_ld;
};

struct DriveCmd {
    int steer;
    int speed;
};

//////////////////////
// GLOBAL VARIABLES //
//////////////////////

CarState car;
Rddf rddf;
PurePursuitConfig config;
DriveCmd vision_cmd;
DriveCmd lidar_cmd;

double target_x_log;
double target_y_log;

///////////////////////
// LOGGING FUNCTIONS //
///////////////////////

string getPathOfLogFile() {
    string ROS_HOME = ros::package::getPath("stier");
 
    time_t now = time(0);
    tm *ltm = localtime(&now);
   
    stringstream ss;

    ss << ROS_HOME << "/log/"
    << setw(2) << setfill('0') << 1 + ltm->tm_mon
    << setw(2) << setfill('0') << ltm->tm_mday << "_"
    << setw(2) << setfill('0') << ltm->tm_hour
    << setw(2) << setfill('0') << ltm->tm_min << ".txt";
    
    return ss.str();
}

ofstream log_file(getPathOfLogFile());

void startLogging() {
    if (!log_file) {
        ROS_WARN("Faile To Write Log File!");
    }
    log_file << "no." << "\t";
    log_file << "car.x" << "\t";
    log_file << "car.y" << "\t";
    log_file << "car.heading" << "\t";
    log_file << "car.cmd_steer" << "\t";
    log_file << "car.feedback_steer" << "\t";
    log_file << "target_x" << "\t";
    log_file << "target_y" << "\t";
    log_file << endl;
    log_file.precision(12);
}

void updateLogging() {
    static int num = 0;
    log_file << num << "\t";
    log_file << car.x << "\t";
    log_file << car.y << "\t";
    log_file << car.heading << "\t";
    log_file << car.cmd_steer << "\t";
    log_file << car.feedback_steer << "\t";
    log_file << target_x_log << "\t";
    log_file << target_y_log << "\t";
    log_file << endl;
    num++;
}

void finishLogging() {
    log_file.close();
}


///////////////////////
// CONTROL FUNCTIONS //
///////////////////////

float wrapToPi(float a){
    a = std::fmod(a + M_PI, 2.0f*M_PI);
    if (a < 0) a += 2.0f*M_PI;
    return a - M_PI;
}

float adaptive_lookahead()
{
     // damping mode
    float distance_error = rddf.calculateDistancePowFromIdx(car.x, car.y, car.idx_current);
    static bool damping_mode = false;
    if (distance_error > 0.5) {
        damping_mode = true;
    }
    float desired_heading = rddf.calculateHeading(car.idx_current);

    static float ld_prev = config.lookahead;
    // 1) 오차 계산
    // 헤딩오차 |epsi|
    const float epsi_abs = std::fabs(wrapToPi(desired_heading - car.heading));

    // 거리오차 |ey|
    // NOTE:
    //  - calculateDistancePowFromIdx() 가 "제곱거리"를 반환한다면 sqrt 사용
    //  - 이미 "거리[m]"면 아래 한 줄을 distance_error 그대로 쓰면 됨
    float ey_abs = std::sqrt(std::max(0.0f, distance_error));
    // float ey_abs = std::max(0.0f, distance_error); // <-- distance_error가 이미 [m]이면 이 줄로 교체

    // 2) 속도(m/s)
    //const float v = std::max(0.0f, kph_current) / 3.6f;

    // 3) 파라미터(튜닝값)
    const float ld_base = config.lookahead;
    const float kv   = 0.6f;   // [s]  v 1 m/s 증가 시 Ld 0.6 m 증가
    const float ky   = 0.8f;   // [m/m]
    const float kpsi = 1.2f;   // [m/rad]
    const float Lmin = 1.5f;
    const float Lmax = 8.0f;
    const float alpha= 0.2f;   // 0~1, 클수록 빠르게 추종(저역통과 계수)

    // 4) 목표 Ld* 계산 + 클램프
    float Ld_star = ld_base + ky*ey_abs + kpsi*epsi_abs;
    Ld_star = std::clamp(Ld_star, Lmin, Lmax);

    float ld_final = (1.0f - alpha)*ld_prev + alpha*Ld_star;
    ld_prev = ld_final;
    
    cout<<" distance_error = "<<distance_error;
    cout<<" desired_heading = "<<desired_heading;
    cout<<" car.heading = "<<car.heading;
    cout<<" ld_final = "<<ld_final<<endl;
    return ld_final;
}


void calculate_lookahead_point_position(double* x_target, double* y_target, double x, double y) {
    int idx = rddf.calculateNearestIdx(x, y);
    int idx_max = rddf.getMaxIdx();
    
    float lookahead = config.lookahead;
    //float lookahead = adaptive_lookahead();

    float lookahead_pow = std::pow(lookahead, 2);
    while (idx <= idx_max) {
        double x_p = rddf.getX(idx);
        double y_p = rddf.getY(idx);
        if (idx == idx_max) {
            (*x_target) = x_p;
            (*y_target) = y_p;
            return;
        }
        double dx = x_p - x;
        double dy = y_p - y;
        float dist_pow = std::pow(dx, 2) + std::pow(dy, 2);
        if (dist_pow < lookahead_pow) {
            idx ++;
        }
        else {
            break;
        }
    }
    if (idx == 0) {
        (*x_target) = rddf.getX(0);
        (*y_target) = rddf.getY(0);
        return;
    }
    (*x_target) = rddf.getX(idx);
    (*y_target) = rddf.getY(idx);
    return;
}

float calculate_steer_using_pure_pursuit(double x, double y, float heading) {
    double current_x = car.x;
    double current_y = car.y;
    
    int cnt = rddf.getCount();
    int cur_idx = car.idx_current;
    //cout<<" cur_idx = "<<cur_idx;
    //cout<<" cog = "<<cog;
    
    // 현재 위치 기준 타겟 포인트 찾기
    double x_target;
    double y_target;
    calculate_lookahead_point_position(&x_target, &y_target, x, y);
    float dis = std::sqrt(std::pow(x_target - current_x, 2)+std::pow(y_target - current_y, 2));
    /*
    int rangeEnd = std::min(cnt - 1, cur_idx + 50);
    rangeEnd = std::min(cnt-1, rangeEnd);
    static int target_idx = 0;
    float dis;
    for (int i = cur_idx; i < rangeEnd; i++) {
        float dist = rddf.calculateDistanceFromIdx(current_x, current_y , i);
        //cout<<" dist("<<i<<")="<<dist;
        if (dist > Ld) {
            target_idx = i;
            dis = dist;
            break;
        }
    }
    */
    //cout<<" target_idx = "<<target_idx;
    // 현재 위치 기준 조향각 계산
    double dx = x_target - current_x;
    double dy = y_target - current_y;
    float alpha = ((float(atan2(dx,dy))));
    //cout<<" alpha = "<<alpha;
    float cog = car.heading;
    float temp_alpha = (alpha - cog);
    //cout<<" temp_alpha = "<<temp_alpha;
    if (cog > PI_ && cog <= 2 * PI_) temp_alpha += 2 * PI_;
    static float wheelbase = car.wheelbase;
    float current_steer;
    current_steer = atan2f(2.0f * wheelbase * sinf(temp_alpha) / (dis), 1.0f);
    current_steer = current_steer * 180.0f / PI_;
    current_steer = std::clamp(current_steer, -25.0f, 25.0f);
    //cout<<" current_steer = "<<current_steer;
    //cout<<endl;
    return current_steer;
}

SectionType get_section_from_rddf_idx(int idx_current) {
    int max_idx = rddf.getMaxIdx();
    SectionType section;
    if (car.idx_current >= max_idx) {
        section = SectionType::STOP;
    }
    else if(car.idx_current >= 21 && car.idx_current <= 54) {
        section = SectionType::LANE_FOLLOWING;
    }
    else {
        section = SectionType::GPS_NAVIGATION;
    }
   return section;
}

// drive msg updaters

// old pp
/*
erp42_msgs::DriveCmd update_drive_cmd_gps_navigation() {
    erp42_msgs::DriveCmd drive_cmd;
    // speed control
    float speed = config.speed;
    static float speed_temp = 0;
    static float speed_step = 5 * (1/CONTROL_FREQUENCY);
    if (speed > speed_temp) {
        speed_temp = speed_temp + speed_step;
        
    }
    else {
        speed_temp = speed;
    }
    drive_cmd.KPH = (int)speed_temp;
    // steer control
    drive_cmd.Deg = calculate_steer_using_pure_pursuit(car.x, car.y, car.heading);
    return drive_cmd;
}
*/

erp42_msgs::DriveCmd update_drive_cmd_gps_navigation() {
    erp42_msgs::DriveCmd drive_cmd;
    
    // steer control
    drive_cmd.Deg = calculate_steer_using_pure_pursuit(car.x, car.y, car.heading) * -1;

    // speed control
    static float speed = config.speed;
    static float speed_cmd = config.speed;
    speed_cmd = config.speed;
    static float speed_step = 1;
    static int idx_step = 1;
    static int idx_prev = 0;
    if (speed_cmd > speed) {
        // soft start
        if ((car.idx_current-idx_prev) >= idx_step) {
            speed = speed + speed_step;
            idx_prev = car.idx_current;
        }
    }
    else {
        // instant downspeed
        speed = speed_cmd;
    }

    // soft stop
    int idx_remained = std::max(0, rddf.getMaxIdx() - car.idx_current);
    float speed_soft_stop = (float)idx_remained * speed_step; // +5 제거, float 유지
    speed = std::min(speed, speed_soft_stop); // (int) 캐스팅 제거
    if (idx_remained == 0) {
        speed = 0.0f;     // 도착 시 완전 정지
        drive_cmd.brake = 200;
    }
    drive_cmd.KPH = speed;
    return drive_cmd;
}

///////////////////////////
// predict next position //
///////////////////////////

void update_position_with_prediction(double current_x, double current_y, float current_speed, 
    float current_heading, float current_steer, double dt) {

    // 자전거 모델 파라미터
    const double wheelbase = car.wheelbase;  // 축거 (L)
    // 속도를 m/s로 변환
    double v = current_speed * (1000.0f / 3600.0f);
    // 조향각을 라디안으로 변환
    double steer_rad = current_steer * (PI_ / 180.0f) * 2;
    
    double new_x, new_y, new_heading;

    current_heading = PI_/2 - current_heading;
    
    // 조향각이 0이면 직진 운동
    if (fabs(steer_rad) < 1e-6) {
        new_x = current_x + v * dt * cos(current_heading);
        new_y = current_y + v * dt * sin(current_heading);
        new_heading = current_heading;
    } else {
        // 회전 반경 R 계산
        double R = wheelbase / tan(steer_rad);
        double omega = v / R; // 각속도 계산
        
        new_x = current_x + R * (sin(current_heading + omega * dt) - sin(current_heading));
        new_y = current_y - R * (cos(current_heading + omega * dt) - cos(current_heading));
        new_heading = PI_/2 - (current_heading) + omega * dt;
    }
    
    car.x = new_x;
    car.y = new_y;
    car.heading = new_heading;
    car.last_position_update_time = ros::Time::now().toSec();
}

////////////////////////
// CALLBACK FUNCTIONS //
////////////////////////
void callback_utm(const geometry_msgs::PoseStamped::ConstPtr& coordinate)
{
    car.x = coordinate->pose.position.x;
    car.y = coordinate->pose.position.y;
    car.last_position_update_time = ros::Time::now().toSec();
}

void callback_navpvt(const ublox_msgs::NavPVT::ConstPtr& msg)
{
    /*
    float cog = msg->heading * 1e-5;
    cog= cog * PI_ / 180.0;

    float heading_rad = PI_/2 - cog;
    float delta = car.feedback_steer * (25.0/300.0) * (PI_ / 180.0);
    float kappa = tan(delta) / car.wheelbase; 
    float heading_temp = atan2(sin(heading_rad) + car.wheelbase * kappa * cos(heading_rad),
        cos(heading_rad) - car.wheelbase * kappa * sin(heading_rad));
    car.heading = PI_/2 - heading_temp;
    */
    float cog = msg->heading * 1e-5;
    cog= cog * PI_ / 180.0;
    car.heading = cog;
    
    car.gps_speed = msg->gSpeed * 0.0036;
    car.last_position_update_time = ros::Time::now().toSec();
}

void serialFeedback(const erp42_msgs::SerialFeedBack::ConstPtr& feedback_msg){
    car.feedback_speed = feedback_msg->speed;
    car.feedback_steer = feedback_msg->steer;
    double dt = (ros::Time::now().toSec() - car.last_position_update_time);
    //update_position_with_prediction(car.x, car.y, car.gps_speed, car.heading, car.feedback_steer, dt);
    /*
    ROS_INFO("car.last_position_update_time:%12.6f | dt:%12.6f",
        car.last_position_update_time, dt);
    */
    car.last_position_update_time = ros::Time::now().toSec();
}

void callback_insideDeg(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO("Received insideDeg: %f", msg->data);
    vision_cmd.steer = msg->data;
    vision_cmd.speed = 20;
}

void callback_isLaneDetect(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("Received islanedetect: %s", msg->data ? "true" : "false");
    //vision_cmd.steer = 0;
    vision_cmd.speed = 15;
}

void missionKPHCallback(const std_msgs::Float32::ConstPtr& msgs){
    lidar_cmd.speed = static_cast<uint16_t>(msgs->data);
}

void missionDegCallback(const std_msgs::Float32::ConstPtr& msgs){
    lidar_cmd.steer = static_cast<uint16_t>(msgs->data);
}


///////////////////
// MAIN FUNCTION //
///////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pp");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("utm", 1, callback_utm);
    ros::Subscriber gps_sub = nh.subscribe("/ublox_position_receiver/navpvt", 1, callback_navpvt);
    ros::Subscriber feedback_sub = nh.subscribe("/erp42_serial/feedback", 1, serialFeedback);

    ros::Subscriber sub_insideDeg = nh.subscribe("insideDeg", 10, callback_insideDeg);
    ros::Subscriber sub_islanedetect = nh.subscribe("islanedetect", 10, callback_isLaneDetect);

    ros::Subscriber lidar_sub1 = nh.subscribe("/missionKPH", 1, missionKPHCallback);
    ros::Subscriber lidar_sub2 = nh.subscribe("/missionDeg", 1, missionDegCallback);

    ros::Publisher drive_pub = nh.advertise<erp42_msgs::DriveCmd>("/erp42_serial/drive", 1);
    ros::Publisher section_pub = nh.advertise<std_msgs::Int32>("/section", 1);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float32>("/gps_speed", 1); 
    ros::Publisher current_idx_pub = nh.advertise<std_msgs::Int32>("/current_idx", 1);
    ros::Publisher mode_pub = nh.advertise<erp42_msgs::ModeCmd>("/erp42_serial/mode", 1);


    ros::Rate loop_rate(CONTROL_FREQUENCY);
    ROS_INFO("ros node started!");

    car.wheelbase = 1.04;
    car.wheel_radius = 0.053;
    car.gps_to_rear = 0;
    car.last_position_update_time = ros::Time::now().toSec();

    int section_fix = -1;

    vision_cmd.steer = 0;
    vision_cmd.speed = 0;

    lidar_cmd.steer = 0;
    lidar_cmd.speed = 0;

    // load parameters
    ROS_INFO("loading parameters..");
    string rddf_name = nh.param("rddf", (string)DEFAULT_RDDF); 
    if (nh.hasParam("rddf")) {
        ROS_INFO("-- rddf : %s", rddf_name.c_str());
    } else {
        ROS_WARN("-- rddf(default) : %s", rddf_name.c_str());
    }

    config.lookahead = nh.param("ld", DEFAULT_LD);
    if (nh.hasParam("ld")) {
        ROS_INFO("-- ld : %f", config.lookahead);
    } else {
        ROS_WARN("-- ld(default) : %f", config.lookahead);
    }

    config.speed = nh.param("speed", DEFAULT_SPEED);
    if (nh.hasParam("speed")) {
        ROS_INFO("-- speed : %f", config.speed);
    } else {
        ROS_WARN("-- speed(default) : %f", config.speed);
    }

    section_fix = nh.param("section_fix", -1);
    if (nh.hasParam("section_fix")) {
        ROS_INFO("-- section_fix : %d", section_fix);
    } else {
        ROS_WARN("-- not using section_fix : %d", section_fix);
    }

    // load rddf
    ROS_INFO("loading rddf..");
    string rddf_path = ros::package::getPath("stier") + "/paths/" + rddf_name;
    if(rddf.load(rddf_path) == 0) {
        ROS_INFO("-- rddf name : %s", rddf.getFilePath().c_str());
        ROS_INFO("-- rddf count : %d", rddf.getCount());
        //rddf.printData();
    }
    else {
        ROS_WARN("-- rddf loading failed");
        ROS_WARN("-- terminating main function");
        ros::shutdown();
    }
    
    // check if the car is on start point of rddf
    ROS_INFO("check if the car is on start point of rddf..");
    bool on_start_point = true;
    double dist_pow;
    while(!on_start_point) {
        ros::spinOnce();
        dist_pow = rddf.calculateDistancePowFromIdx(car.x, car.y, 0);
        if (dist_pow < 9.0) {
            ROS_INFO("-- the car is on the start point! (dist = %f)", sqrt(dist_pow));
            on_start_point = true;
        }
        else {
            ROS_WARN("-- not on the start point (dist = %f)", sqrt(dist_pow));
            ros::Duration(2.0).sleep();
        }
    }

    // start logging
    startLogging();

    // count down
    int countdown = 5;
    ROS_INFO("the car is ready! wait for %d seconds..", countdown);
    for ( int i=countdown; i>0; i--) {
        ROS_INFO("-- %d", i);
        ros::Duration(1.0).sleep();
    }
    
    ROS_INFO("control loop will start now!"); 
    while (ros::ok()) {
        erp42_msgs::ModeCmd mode_msg;
        mode_msg.MorA = 0x01;
        //mode_msg.EStop = 0x00;
        mode_pub.publish(mode_msg);

        ros::spinOnce();
        car.idx_current = rddf.calculateNearestIdx(car.x, car.y);
        SectionType section = get_section_from_rddf_idx(car.idx_current);
        if (section_fix != -1) {
            section = static_cast<SectionType>(section_fix);
        }
        
        erp42_msgs::DriveCmd drive_cmd;
        switch (section) {
            case SectionType::GPS_NAVIGATION:
                drive_cmd = update_drive_cmd_gps_navigation();
                ROS_INFO("car.x:%12.4f | car.y:%12.4f | section:%02d | idx:%4d/%4d",
                    car.x, car.y, (int)section, car.idx_current, rddf.getMaxIdx());
                ROS_INFO("drive_cmd.KPH:%2d | drive_cmd.Deg:%3d | feedback_steer:%3d",
                    drive_cmd.KPH, drive_cmd.Deg, car.feedback_steer);
                ROS_INFO("------------------------------------------");
                break;
            case SectionType::LANE_FOLLOWING:
                drive_cmd.KPH = vision_cmd.speed;
                drive_cmd.Deg = vision_cmd.steer;
                ROS_INFO("drive_cmd.KPH:%2d | drive_cmd.Deg:%3d | section:%02d",
                    drive_cmd.KPH, drive_cmd.Deg, (int)section);
                ROS_INFO("------------------------------------------");
                break;
            case SectionType::LIDAR_ONLY:
                drive_cmd.KPH = lidar_cmd.speed;
                drive_cmd.Deg = lidar_cmd.steer;
                ROS_INFO("drive_cmd.KPH:%2d | drive_cmd.Deg:%3d | section:%02d",
                    drive_cmd.KPH, drive_cmd.Deg, (int)section);
                ROS_INFO("------------------------------------------");
                break;
            default:
                //stop vehicle
                drive_cmd.KPH = 0;
                drive_cmd.Deg = 0;
                break;
        }
        car.cmd_steer = drive_cmd.KPH;
        car.cmd_steer = drive_cmd.Deg;
        drive_pub.publish(drive_cmd);

        std_msgs::Int32 section_msg;
        section_msg.data = (int)section;
        section_pub.publish(section_msg);

        std_msgs::Float32 speed_msg;
        speed_msg.data = car.gps_speed;
        speed_pub.publish(speed_msg);

        std_msgs::Int32 current_idx_msg;
        current_idx_msg.data = car.idx_current;
        current_idx_pub.publish(current_idx_msg);

        updateLogging();
        loop_rate.sleep();
    }

    finishLogging();
    return 0;
}



//cout<<" cog="<<cog;
/*
float heading_rad = cog;
float delta = car.feedback_steer * PI_ / 180.0;
float kappa = tan(delta) / car.wheelbase; 
float cog_temp = atan2(sin(heading_rad) + car.wheelbase * kappa * cos(heading_rad),
    cos(heading_rad) - car.wheelbase * kappa * sin(heading_rad));
if (cog > PI_ && cog <= 2 * PI_) cog_temp += 2 * PI_;
car.heading = cog_temp;
*/
//cout<<" car.heading="<<car.heading;
//cout<<endl;
