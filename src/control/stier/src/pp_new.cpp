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

#define PI_ 3.141592653f
#define CONTROL_FREQUENCY 16.0f
#define CONTROL_PERIOD 1.0f/CONTROL_FREQUENCY
#define DEFAULT_SPEED 5.0f
#define DEFAULT_LD 1.0f
#define DEFAULT_LD_LANE_CHANGE 2.5f
#define DEFAULT_RDDF1 "lane1.txt"
#define DEFAULT_RDDF2 "lane2.txt"

using namespace std;

enum SectionType {
    STOP = 0,
    GPS_NAVIGATION = 1,
    VISION_ONLY = 2,
    LIDAR_ONLY = 3,
    TEST_STEERING = 99
};

struct CarState {
    float wheelbase;
    float wheel_radius;
    float gps_to_rear;
    double x;
    double y;
    float heading;
    double last_position_update_time;
    float speed;
    int idx_current;
    int cmd_steer;
    int cmd_speed;
    int feedback_steer;
    int feedback_speed;
    float gps_speed;
};

struct PurePursuitConfig {
    float lookahead;
    float speed;
};

struct DriveCmd {
    int steer;
    int speed;
};

CarState car;
Rddf rddf;
PurePursuitConfig config;
DriveCmd vision_cmd;
DriveCmd lidar_cmd;

double target_x_log;
double target_y_log;

int current_lane = 1;
int desired_lane = 1;
bool rddf_change_req = false;
bool rddf_switching = false;

int target_speed_x10 = static_cast<int>(DEFAULT_SPEED * 10.0f);
bool estop_flag = false;

uint8_t led_signal = 0; 

string getPathOfLogFile() {
    string ROS_HOME = ros::package::getPath("control");
 
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

    int idx_2 = idx;
    double x_p2 = rddf.getX(idx_2);
    double y_p2 = rddf.getY(idx_2);
    double dx_p2 = x_p2 - x;
    double dy_p2 = y_p2 - y;
    float dist_2 = std::sqrt(std::pow(dx_p2, 2)+std::pow(dy_p2, 2));

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

    double x_target;
    double y_target;
    calculate_lookahead_point_position(&x_target, &y_target, x, y);
    float dis = std::sqrt(std::pow(x_target - current_x, 2)+std::pow(y_target - current_y, 2));

    double dx = x_target - current_x;
    double dy = y_target - current_y;
    float alpha = ((float(atan2(dx,dy))));
    float cog = car.heading;
    float temp_alpha = (alpha - cog);
    if (cog > PI_ && cog <= 2 * PI_) temp_alpha += 2 * PI_;
    static float wheelbase = 0.73;
    float current_steer;
    current_steer = atan2f(2.0f * wheelbase * sinf(temp_alpha) / (dis), 1.0f);
    current_steer = current_steer * 180.0f / PI_;
    current_steer = std::clamp(current_steer, -25.0f, 25.0f);
    return current_steer;
}

SectionType get_section_from_rddf_idx(int idx_current) {
    int max_idx = rddf.getMaxIdx();
    SectionType section;
    if (car.idx_current >= max_idx) {
        section = SectionType::STOP;
    }
    else if(car.idx_current >= 21 && car.idx_current <= 54) {
        section = SectionType::VISION_ONLY;
    }
    else {
        section = SectionType::GPS_NAVIGATION;
    }
   return section;
}

erp42_msgs::DriveCmd update_drive_cmd_gps_navigation() {
    erp42_msgs::DriveCmd drive_cmd;

    drive_cmd.Deg = calculate_steer_using_pure_pursuit(car.x, car.y, car.heading);

    static int speed_x10 = static_cast<int>(DEFAULT_SPEED * 10.0f);
    static int speed_cmd_x10 = static_cast<int>(DEFAULT_SPEED * 10.0f);

    speed_cmd_x10 = target_speed_x10;
    speed_x10 = speed_cmd_x10;

    static int speed_step_x10 = 10;
    int offset_x10 = 50;
    int idx_remained = rddf.getMaxIdx() - car.idx_current;

    int speed_soft_stop_x10 = idx_remained * speed_step_x10 + offset_x10;
    if (speed_x10 > speed_soft_stop_x10) {
        speed_x10 = speed_soft_stop_x10;
    }

    if (estop_flag) {
        speed_x10 = 0;
    }

    drive_cmd.KPH = static_cast<uint16_t>(speed_x10);
    return drive_cmd;
}


void update_position_with_prediction(double current_x, double current_y, float current_speed, 
    float current_heading, float current_steer, double dt) {

    const double wheelbase = car.wheelbase;
    double v = current_speed * (1000.0f / 3600.0f);
    double steer_rad = current_steer * (PI_ / 180.0f) * 2;
    
    double new_x, new_y, new_heading;

    current_heading = PI_/2 - current_heading;
    
    if (fabs(steer_rad) < 1e-6) {
        new_x = current_x + v * dt * cos(current_heading);
        new_y = current_y + v * dt * sin(current_heading);
        new_heading = current_heading;
    } else {
        double R = wheelbase / tan(steer_rad);
        double omega = v / R; 
        
        new_x = current_x + R * (sin(current_heading + omega * dt) - sin(current_heading));
        new_y = current_y - R * (cos(current_heading + omega * dt) - cos(current_heading));
        new_heading = PI_/2 - (current_heading) + omega * dt;
    }
    
    car.x = new_x;
    car.y = new_y;
    car.heading = new_heading;
    car.last_position_update_time = ros::Time::now().toSec();
}

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
    car.last_position_update_time = ros::Time::now().toSec();
}

void callback_insideDeg(const std_msgs::Float32::ConstPtr& msg)
{
    vision_cmd.steer = msg->data;
    vision_cmd.speed = 20;
}

void callback_isLaneDetect(const std_msgs::Bool::ConstPtr& msg)
{
    vision_cmd.speed = 15;
}

void missionKPHCallback(const std_msgs::Float32::ConstPtr& msgs){
    lidar_cmd.speed = static_cast<uint16_t>(msgs->data);
}

void missionDegCallback(const std_msgs::Float32::ConstPtr& msgs){
    lidar_cmd.steer = static_cast<uint16_t>(msgs->data);
}

void callback_target_lane(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data != current_lane) {
        desired_lane = msg->data;
        rddf_change_req = true;
        ROS_INFO("RDDF lane change request: %d -> %d", current_lane, desired_lane);
    }
}

void callback_target_speed(const std_msgs::Float32::ConstPtr& msg) {
    float target_speed = msg->data;
    target_speed_x10 = static_cast<int>(target_speed * 10.0f + 0.5f);
    ROS_INFO("Received /target_speed_x10 = %d (%.1f km/h)", target_speed_x10, target_speed);
}

void callback_estop(const std_msgs::Int32::ConstPtr& msg) {
    estop_flag = (msg->data == 1);
}

void callback_led(const std_msgs::Int32::ConstPtr& msg) {
    led_signal = static_cast<uint8_t>(msg->data);
}

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
    ros::Subscriber lane_sub = nh.subscribe("/target_lane", 1, callback_target_lane);
    ros::Subscriber target_speed_sub = nh.subscribe("/target_speed", 1, callback_target_speed);
    ros::Subscriber estop_sub = nh.subscribe("/ESTOP", 1, callback_estop);
    ros::Subscriber led_sub   = nh.subscribe("/LED",   1, callback_led);

    ros::Publisher drive_pub = nh.advertise<erp42_msgs::DriveCmd>("/erp42_serial/drive", 1);
    ros::Publisher section_pub = nh.advertise<std_msgs::Int32>("/section", 1);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float32>("/gps_speed", 1); 
    ros::Publisher current_idx_pub = nh.advertise<std_msgs::Int32>("/current_idx", 1);
    ros::Publisher lane_pub = nh.advertise<std_msgs::Int32>("/current_lane", 1);
    ros::Publisher ld_pub = nh.advertise<std_msgs::Float32>("/lookahead", 1);

    ros::Rate loop_rate(CONTROL_FREQUENCY);
    ROS_INFO("ros node started!");

    car.wheelbase = 0.730;
    car.wheel_radius = 0.115;
    car.gps_to_rear = 0.565;
    car.last_position_update_time = ros::Time::now().toSec();

    int section_fix = -1;

    vision_cmd.steer = 0;
    vision_cmd.speed = 0;

    lidar_cmd.steer = 0;
    lidar_cmd.speed = 0;

    ROS_INFO("loading parameters..");
    string rddf1_name = nh.param("rddf1", (string)DEFAULT_RDDF1); 
    string rddf2_name = nh.param("rddf2", (string)DEFAULT_RDDF2); 

    if (nh.hasParam("rddf1")) {
        ROS_INFO("-- rddf1 : %s", rddf1_name.c_str());
    } else {
        ROS_WARN("-- rddf1(default) : %s", rddf1_name.c_str());
    }

    if (nh.hasParam("rddf2")) {
        ROS_INFO("-- rddf2 : %s", rddf2_name.c_str());
    } else {
        ROS_WARN("-- rddf2(default) : %s", rddf2_name.c_str());
    }
    
    config.lookahead = nh.param("ld", DEFAULT_LD);
    if (nh.hasParam("ld")) {
        ROS_INFO("-- ld : %f", config.lookahead);
    } else {
        ROS_WARN("-- ld(default) : %f", config.lookahead);
    }

    float ld_lane_change = nh.param("ld_lane_change", DEFAULT_LD_LANE_CHANGE);
    if (nh.hasParam("ld_lane_change")) {
        ROS_INFO("-- ld_lane_change : %f", ld_lane_change);
    } else {
        ROS_WARN("-- ld_lane_change(default) : %f", DEFAULT_LD_LANE_CHANGE);
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

    ROS_INFO("loading rddf..");
    string rddf_path = ros::package::getPath("control") + "/paths/" + rddf1_name;
    if(rddf.load(rddf_path) == 0) {
        ROS_INFO("-- rddf name : %s", rddf.getFilePath().c_str());
        ROS_INFO("-- rddf count : %d", rddf.getCount());
    }
    else {
        ROS_WARN("-- rddf loading failed");
        ROS_WARN("-- terminating main function");
        ros::shutdown();
    }
    
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

    startLogging();
    
    ROS_INFO("control loop will start now!"); 
    while (ros::ok()) {
        ros::spinOnce();
        
        float distance_error = rddf.calculateDistancePowFromIdx(car.x, car.y, car.idx_current);
        if (rddf_switching) {
            if (distance_error > 0.5) {
                config.lookahead = ld_lane_change;
            } else {
                config.lookahead = DEFAULT_LD;
                rddf_switching = false;
                current_lane = desired_lane;
                ROS_INFO("Vehicle converged to new RDDF. LD restored.");
            }
        } else {
            config.lookahead = DEFAULT_LD;
        }

        if (rddf_change_req) {
            string rddf_filename;
            if (desired_lane == 1) {
                rddf_filename = rddf1_name;
            } else if (desired_lane == 2) {
                rddf_filename = rddf2_name;
            } else {
                ROS_WARN("WARNING!!!! Invalid lane number: %d (only 1 or 2 allowed)", desired_lane);
                rddf_change_req = false;
                continue;
            }

            string rddf_path = ros::package::getPath("control") + "/paths/" + rddf_filename;
            if (rddf.load(rddf_path) == 0) {
                car.idx_current = rddf.calculateNearestIdx(car.x, car.y);
                rddf_switching = true;
                rddf_change_req = false;
                ROS_INFO("RDDF %s loaded. Waiting for convergence to switch to lane %d", rddf_filename.c_str(), desired_lane);
            } else {
                ROS_WARN("XXXXXXX Failed to load RDDF %s. Keeping current lane.", rddf_filename.c_str());
                rddf_change_req = false;
            }
        }
        
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
            case SectionType::VISION_ONLY:
                drive_cmd.KPH = vision_cmd.speed;
                drive_cmd.Deg = vision_cmd.steer;
                break;
            case SectionType::LIDAR_ONLY:
                drive_cmd.KPH = lidar_cmd.speed;
                drive_cmd.Deg = lidar_cmd.steer;
                break;
            default:
                drive_cmd.KPH = 0;
                drive_cmd.Deg = 0;
                break;
        }
        car.cmd_speed = drive_cmd.KPH;
        car.cmd_steer = drive_cmd.Deg;
        drive_cmd.brake = led_signal;
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

        std_msgs::Int32 lane_msg;
        lane_msg.data = current_lane;
        lane_pub.publish(lane_msg);

        std_msgs::Float32 ld_msg;
        ld_msg.data = config.lookahead;
        ld_pub.publish(ld_msg);

        updateLogging();
        loop_rate.sleep();
    }

    finishLogging();
    return 0;
}