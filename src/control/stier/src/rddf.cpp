#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>
#include <ctime>
#include <iostream>
#include <sstream>
#include <ros/package.h>
#include <math.h>
#include <iomanip>

using namespace std;

string ROS_HOME;
ofstream f;
geometry_msgs::PoseStamped::ConstPtr prev_coordinate;
int path_count = 0;  // 경로 카운터

static std::string ensure_txt_ext(std::string name) {
    if (name.size() < 4 || name.substr(name.size() - 4) != ".txt") name += ".txt";
    return name;
}

void callback(const geometry_msgs::PoseStamped::ConstPtr &coordinate)
{
    f.precision(15);
    if (!prev_coordinate) {
        f << coordinate->pose.position.x << "\t"
          << coordinate->pose.position.y << "\t" << endl;
        prev_coordinate = coordinate;
        path_count++;
        cout << "새로운 경로 포인트가 추가되었습니다. (현재까지 총 "
             << path_count << "개의 포인트)" << endl;
    } else {
        // 거리 계산 (m)
        double dx = coordinate->pose.position.x - prev_coordinate->pose.position.x;
        double dy = coordinate->pose.position.y - prev_coordinate->pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        if (distance >= 0.5) {
            f << coordinate->pose.position.x << "\t"
              << coordinate->pose.position.y << "\t" << endl;
            prev_coordinate = coordinate;
            path_count++;
            cout << "새로운 경로 포인트가 추가되었습니다. (현재까지 총 "
                 << path_count << "개의 포인트)" << endl;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rddf");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");  // ★ 프라이빗 파라미터

    // 1) 파일 이름 파라미터(폴더는 그대로 stier/paths)
    std::string outfile_param;
    pnh.param<std::string>("outfile", outfile_param, std::string("")); // 예: _outfile:=lap1

    // 2) 기본 경로 + 파일명 결정
    ROS_HOME = ros::package::getPath("stier");
    const std::string base_dir = ROS_HOME + "/paths";

    std::string filename;
    if (!outfile_param.empty()) {
        filename = ensure_txt_ext(outfile_param);
    } else {
        // 기본: 타임스탬프
        time_t now = time(0);
        tm *ltm = localtime(&now);
        std::stringstream def;
        def << (1900 + ltm->tm_year) << "-"
            << std::setw(2) << std::setfill('0') << (1 + ltm->tm_mon) << "-"
            << std::setw(2) << std::setfill('0') << ltm->tm_mday << "_"
            << std::setw(2) << std::setfill('0') << ltm->tm_hour << "-"
            << std::setw(2) << std::setfill('0') << ltm->tm_min;
        filename = ensure_txt_ext(def.str());
    }

    std::stringstream ss;
    ss << base_dir << "/" << filename;
    f.open(ss.str().c_str());
    if (!f.is_open()) {
        std::cerr << "[rddf] 파일을 열 수 없습니다: " << ss.str() << std::endl;
        return 1;
    }

    // 3) 구독 및 기록 시작
    ros::Subscriber sub = nh.subscribe("utm", 1, callback);
    cout << "경로 기록을 시작합니다... (파일: " << ss.str() << ")" << endl;

    ros::spin();

    cout << "경로 기록을 종료합니다. 총 "
         << path_count << "개의 경로 포인트가 저장되었습니다." << endl;
    f.close();
    return 0;
}

