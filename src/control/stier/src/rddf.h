#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>

class Rddf {
public:
    Rddf() : rddf(nullptr), count(0), idx_prev(0), first_time(true) {}

    ~Rddf() {
        unload();
    }

    // 파일로부터 rddf 데이터를 로드합니다.
    // 파일의 각 줄에 두 개의 double 값(x, y)이 있다고 가정합니다.
    int load(const std::string &file_path_input) {
        file_path = file_path_input;
        std::ifstream file(file_path);
        if (!file) {
            std::cerr << "Error: Cannot open file " << file_path << std::endl;
            return -1;
        }

        // 데이터의 개수를 카운트
        count = 0;
        double x, y;
        while (file >> x >> y) {
            ++count;
        }
        file.close();

        // 메모리 할당
        rddf = new double*[count];
        for (int i = 0; i < count; ++i) {
            rddf[i] = new double[2];
        }

        // 실제 데이터를 읽어옴
        file.open(file_path);
        for (int i = 0; i < count; i++) {
            file >> rddf[i][0] >> rddf[i][1];
        }
        file.close();

        // 검색 관련 변수 초기화
        idx_prev = 0;
        first_time = true;

        return 0;
    }

    // 할당한 메모리를 해제합니다.
    void unload() {
        if (rddf != nullptr) {
            for (int i = 0; i < count; ++i) {
                delete[] rddf[i];
            }
            delete[] rddf;
            rddf = nullptr;
        }
        count = 0;
    }

    // (x, y)에 가장 가까운 점의 인덱스를 반환합니다.
    int calculateNearestIdx(double x, double y) {
        if (count == 0) return -1; // 데이터가 없으면 -1 반환

        int idx_nearest = idx_prev;

        int range_start, range_end;
        if (first_time) {
            range_start = 0;
            range_end = count - 1;
            first_time = false;
        } else {
            range_start = std::max(0, idx_prev - 300);
            range_end = std::min(count - 1, idx_prev + 500);
        }

        double dist_pow_min = 1e9; // 매우 큰 초기값
        for (int i = range_start; i <= range_end; i++) {
            double dx = x - rddf[i][0];
            double dy = y - rddf[i][1];
            double dist_pow = dx * dx + dy * dy;
            if (dist_pow < dist_pow_min) {
                dist_pow_min = dist_pow;
                idx_nearest = i;
            }
        }
        idx_prev = idx_nearest;
        return idx_nearest;
    }

    // (x, y)로부터 일정 거리(distance) 이상 떨어진 첫번째 점의 인덱스를 반환합니다.
    // 만약 범위 내에 그러한 점이 없다면, 범위의 마지막 인덱스를 반환합니다.
    int calculateDistantIdx(double x, double y, double distance) {
        int idx_current = calculateNearestIdx(x, y);
        if (idx_current < 0) return -1;

        int range_end = std::min(count - 1, idx_current + 50);
        for (int i = idx_current; i <= range_end; i++) {
            double dx = x - rddf[i][0];
            double dy = y - rddf[i][1];
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > distance) {
                return i;
            }
        }
        return range_end;
    }

    int getMaxIdx() {
        return count-1;
    }

    void printData() {
        ROS_INFO("printing all of rddf waypoints..");
        for (int i = 0; i <= count-1; i++) {
            double x = rddf[i][0];
            double y = rddf[i][1];
            ROS_INFO("-- [%4d] %12.4f, %12.4f", i, x, y);
        }
    }

    int getCount() {
        return count;
    }

    std::string getFilePath() {
        return file_path;
    }

    float calculateDistancePowFromIdx(double x, double y, int idx) {
        return std::pow(x - rddf[idx][0], 2) + std::pow(y - rddf[idx][1], 2);
    }

    float calculateDistanceFromIdx(double x, double y, int idx) {
        return std::sqrt(calculateDistancePowFromIdx(x, y, idx));
    }

    double getX(int idx) {
        return rddf[idx][0];
    }

    double getY(int idx) {
        return rddf[idx][1];
    }

    float calculateHeading(int idx) {
        if (count < 2) return 0.0; 
        
        int idx_prev = std::max(0, idx - 1);
        int idx_next = std::min(count - 1, idx + 1);

        double dx = rddf[idx_next][0] - rddf[idx_prev][0];
        double dy = rddf[idx_next][1] - rddf[idx_prev][1];

        float angle = std::atan2(dy, dx);
        float heading;
        if (angle >= 0) {
            if (angle <= M_PI/2) heading = M_PI/2 - angle;
            else heading = (5/2)*M_PI - angle;
        }
        else {
            heading = -1*angle + M_PI/2;
        }
        
        return heading;
    }


private:
    double** rddf;  // [count][2] 형태의 데이터
    int count;      // 데이터 개수
    int idx_prev;   // 이전 검색 결과(최적화용)
    bool first_time; // 최초 검색 여부
    std::string file_path;
};
