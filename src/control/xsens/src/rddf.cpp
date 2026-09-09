#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>
#include <ctime>
#include <iostream>
#include <sstream>
#include <ros/package.h>
#include <math.h>

#include <ublox_msgs/NavSTATUS.h>


using namespace std;

string ROS_HOME;
ofstream f;
geometry_msgs::PoseStamped::ConstPtr prev_coordinate;
int path_count = 0;  // 경로 카운터 추가

int fix_type = 0;
int fix_stat = 0;

double resolution_distance = 0.5;
string name = "";


void callback(const geometry_msgs::PoseStamped::ConstPtr &coordinate)
{
    f.precision(15);
    if (!prev_coordinate) {
        f << coordinate->pose.position.x << "\t" << coordinate->pose.position.y << "\t" << endl;
        prev_coordinate = coordinate;
        path_count++;  // 첫 번째 경로 포인트 추가
        cout << "새로운 경로 포인트가 추가되었습니다. (현재까지 총 " << path_count << "개의 포인트)" << endl;
    } else {
        //거리 계산
        
        double distance = sqrt(pow(coordinate->pose.position.x - prev_coordinate->pose.position.x, 2) +
                              pow(coordinate->pose.position.y - prev_coordinate->pose.position.y, 2));
        
        if (distance >= resolution_distance) {
            f << coordinate->pose.position.x << "\t" << coordinate->pose.position.y << "\t";
            f << endl;
            prev_coordinate = coordinate;
            path_count++;  // 새로운 경로 포인트 추가
            cout << "새로운 경로 포인트가 추가되었습니다. (현재까지 총 " << path_count << "개의 포인트)" << endl;
        }
    }
}

void navStatusCallback(const ublox_msgs::NavSTATUS::ConstPtr& msg)
{
    fix_type = (int) msg->gpsFix;
    fix_stat = (int) msg->fixStat;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rddf");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("utm", 1, callback);
    ros::Subscriber navstatus_sub = nh.subscribe("/ublox_position_receiver/navstatus", 10, navStatusCallback);

    if (nh.hasParam("resolution")) {
        resolution_distance = nh.param("resolution", resolution_distance);
        ROS_INFO("resolution : %f", resolution_distance);
    } else {
        ROS_WARN("default resolution : %f", resolution_distance);
    }

    if (nh.hasParam("name")) {
        name = nh.param("name", name);
    } else {
        ROS_WARN("default name");
    }

    ROS_HOME = ros::package::getPath("stier");
    time_t now = time(0);
    tm *ltm = localtime(&now);
    stringstream ss;
    if (name == "") {
        ss << ROS_HOME << "/paths/";
        ss << 1900 + ltm->tm_year << "-" << 1 + ltm->tm_mon << "-" << ltm->tm_mday;
        ss << "_" << ltm->tm_hour << "-" << ltm->tm_min;
        ss << ".txt";
    }
    else {
        ss << ROS_HOME << "/paths/";
        ss << 1900 + ltm->tm_year << "-" << 1 + ltm->tm_mon << "-" << ltm->tm_mday;
        ss << "_" << ltm->tm_hour << "-" << ltm->tm_min;
        ss << "_" << name;
        ss << ".txt";
    }
    f.open(ss.str());
    cout << "경로 기록을 시작합니다..." << endl;  // 시작 메시지 추가
    ros::spin();
    cout << "경로 기록을 종료합니다. 총 " << path_count << "개의 경로 포인트가 저장되었습니다." << endl;  // 종료 메시지 추가
    f.close();
    return 0;
}
