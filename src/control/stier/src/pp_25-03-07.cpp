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
    TUNNEL = 3,
    STATIC_OBSTACLE_AVOIDANCE = 4,
    DYNAMIC_OBSTACLE_AVOIDANCE = 5
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
    float feedback_steer;
    float feedback_speed;
    // gps
    float gps_speed;
};

struct PurePursuitConfig {
    float lookahead;
    float speed;
};

//////////////////////
// GLOBAL VARIABLES //
//////////////////////

CarState car;
Rddf rddf;
PurePursuitConfig config;

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

void calculate_lookahead_point_position(double* x_target, double* y_target, double x, double y) {
    int idx = rddf.calculateNearestIdx(x, y);
    int idx_max = rddf.getMaxIdx();
    float lookahead = config.lookahead;
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
    int idx_1 = std::max(idx-1, 0);
    double x_p1 = rddf.getX(idx_1);
    double y_p1 = rddf.getY(idx_1);
    double dx_p1 = x_p1 - x;
    double dy_p1 = y_p1 - y;
    float dist_1 = std::sqrt(std::pow(dx_p1, 2)+std::pow(dy_p1, 2));
    /*
    ROS_INFO("idx_1:%d | x_p1:%12.4f | y_p1:%12.4f | dist_1:%8.4f",
        idx_1, x_p1, y_p1, dist_1);
    */
    int idx_2 = idx;
    double x_p2 = rddf.getX(idx_2);
    double y_p2 = rddf.getY(idx_2);
    double dx_p2 = x_p2 - x;
    double dy_p2 = y_p2 - y;
    float dist_2 = std::sqrt(std::pow(dx_p2, 2)+std::pow(dy_p2, 2));
    /*
    ROS_INFO("idx_2:%d | x_p2:%12.4f | y_p2:%12.4f | dist_2:%8.4f",
        idx_2, x_p2, y_p2, dist_2);
    */
    double dx_p = dx_p2 - dx_p1;
    double dy_p = dy_p2 - dy_p1;
    double dist_point_pow = std::pow(dx_p, 2)+std::pow(dy_p, 2);
    double dot_product_point = dx_p1 * dx_p + dy_p1 * dy_p;
    double a = dist_point_pow;
    double b = 2.0 * dot_product_point;
    double c = std::pow(dist_1, 2)-lookahead_pow;
    double discriminant = b*b - 4.0 * a * c;
    if (discriminant < 0) {
        std::cerr << "오류: 판별식이 음수입니다. (해가 없음)" << std::endl;
        (*x_target) = x_p1;
        (*y_target) = y_p1;
        return;
    }
    double t1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
    double t2 = (-b - std::sqrt(discriminant)) / (2.0 * a);
    double t;
    if(t1 >= 0.0 && t1 <= 1.0)
        t = t1;
    else if(t2 >= 0.0 && t2 <= 1.0)
        t = t2;
    else {
        std::cerr << "오류: [0,1] 구간 내의 유효한 t값이 없습니다." << std::endl;
        (*x_target) = x_p1;
        (*y_target) = y_p1;
        return;
    }
    (*x_target) = x_p1 + t * dx_p;
    (*y_target) = y_p1 + t * dy_p;
    target_x_log = (*x_target);
    target_y_log = (*y_target);
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
    static float wheelbase = 0.73;
    float current_steer;
    current_steer = atan2f(2.0f * wheelbase * sinf(temp_alpha) / (dis), 1.0f);
    current_steer = current_steer * 180.0f / PI_;
    current_steer = std::clamp(current_steer, -20.0f, 20.0f);
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
    drive_cmd.Deg = calculate_steer_using_pure_pursuit(car.x, car.y, car.heading);
    // speed control
    float distance_error = rddf.calculateDistancePowFromIdx(car.x, car.y, car.idx_current);
    static bool damping_mode = false;
    cout<<" distance_error = "<<distance_error;
    if (distance_error > 0.05) {
        damping_mode = true;
    }
    float desired_heading = rddf.calculateHeading(car.idx_current);
    cout<<" desired_heading = "<<desired_heading;
    cout<<" car.heading = "<<car.heading;
    bool is_heading_aligned = std::abs(desired_heading-car.heading) < PI_/24;
    cout<<" is_heading_aligned = "<<is_heading_aligned;
    if ((distance_error < 0.01) && is_heading_aligned) {
        damping_mode = false;
    }
    cout<<" damping_mode = "<<damping_mode<<endl;
    static float speed_temp = 0;
    float speed = config.speed;
    if (damping_mode) {
        speed = 10;
        speed_temp = 10;
        drive_cmd.KPH = speed;
        return drive_cmd;
    }
    static float speed_step = 5 * (1/CONTROL_FREQUENCY);
    if (speed > speed_temp) {
        speed_temp = speed_temp + speed_step;
    }
    else {
        speed_temp = speed;
    }
    drive_cmd.KPH = (int)speed_temp;
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
    ROS_INFO("car.last_position_update_time:%12.6f | dt:%12.6f",
        car.last_position_update_time, dt);
    car.last_position_update_time = ros::Time::now().toSec();
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
    ros::Publisher drive_pub = nh.advertise<erp42_msgs::DriveCmd>("/erp42_serial/drive", 1);
    ros::Rate loop_rate(CONTROL_FREQUENCY);
    ROS_INFO("ros node started!");

    
    car.wheelbase = 0.730;
    car.wheel_radius = 0.115;
    car.gps_to_rear = 0.565;
    car.last_position_update_time = ros::Time::now().toSec();

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
    bool on_start_point = false;
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
        ros::spinOnce();
        car.idx_current = rddf.calculateNearestIdx(car.x, car.y);
        SectionType section = get_section_from_rddf_idx(car.idx_current);
        
        erp42_msgs::DriveCmd drive_cmd;
        switch (section) {
            case SectionType::GPS_NAVIGATION:
                drive_cmd = update_drive_cmd_gps_navigation();
                ROS_INFO("car.x:%12.4f | car.y:%12.4f | section:%02d | idx:%4d/%4d",
                    car.x, car.y, (int)section, car.idx_current, rddf.getMaxIdx());
                ROS_INFO("drive_cmd.KPH:%2d | drive_cmd.Deg:%3d",
                    drive_cmd.KPH, drive_cmd.Deg);
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
