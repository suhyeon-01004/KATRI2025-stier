#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>
#include <ctime>
#include <iostream>
#include <sstream>
#include <ros/package.h>
#include <math.h>
using namespace std;

string ROS_HOME;
ofstream f;
geometry_msgs::PoseStamped::ConstPtr prev_coordinate;
int path_count = 0;  // 경로 카운터 추가

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
        if (distance >= 0.5) {
            f << coordinate->pose.position.x << "\t" << coordinate->pose.position.y << "\t" << endl;
            prev_coordinate = coordinate;
            path_count++;  // 새로운 경로 포인트 추가
            cout << "새로운 경로 포인트가 추가되었습니다. (현재까지 총 " << path_count << "개의 포인트)" << endl;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rddf");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("utm", 1, callback);

    // 현재 패키지(rddf_recorder)의 경로를 가져옵니다.
    ROS_HOME = ros::package::getPath("rddf_recorder");

    // 현재 시간을 이용하여 파일 이름 생성
    time_t now = time(0);
    tm *ltm = localtime(&now);
    stringstream ss;
    ss << ROS_HOME << "/paths/"
    << 1900 + ltm->tm_year << "-" << 1 + ltm->tm_mon << "-" << ltm->tm_mday 
    << "_" << ltm->tm_hour << "-" << ltm->tm_min;
    
    // 인풋 인자(argv[1])가 있으면 파일 이름 마지막에 붙입니다.
    if (argc > 1) {
        ss << "_" << argv[1];
    }
    
    ss << ".txt";
    
    // 파일을 열고 경로 기록 시작
    f.open(ss.str());
    cout << "경로 기록을 시작합니다..." << ss.str() << endl;
    ros::spin();
    cout << "경로 기록을 종료합니다. 총 " << path_count << "개의 경로 포인트가 저장되었습니다." << endl;  // 종료 메시지 추가
    f.close();
    return 0;
}