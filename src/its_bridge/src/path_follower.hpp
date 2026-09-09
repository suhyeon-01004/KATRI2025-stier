#pragma once
#include <string>
#include <vector>
#include <optional>
#include <fstream>
#include <sstream>
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

class PathFollower {
public:
    // 유일한 인스턴스를 반환하는 전역 접근 함수
    static PathFollower& getInstance();

    // 복사 및 대입을 방지하여 싱글턴 패턴을 보장
    PathFollower(const PathFollower&) = delete;
    void operator=(const PathFollower&) = delete;

    float findNearestDistance(float x, float y) const;

private:
    // 생성자를 private으로 만들어 외부에서 객체 생성을 막음
    PathFollower();

    // PIMPL(Pointer to implementation) 패턴 사용
    class Impl;
    std::unique_ptr<Impl> pimpl_; 
};