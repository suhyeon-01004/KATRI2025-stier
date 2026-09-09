#include "path_follower.hpp"
#include "utm_converter.hpp"

class PathFollower::Impl {
public:
    Impl() {
        path_cloud_.reset(new pcl::PointCloud<pcl::PointXY>());
        path_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXY>());
        
        loadPath();
    }
    pcl::PointCloud<pcl::PointXY>::Ptr path_cloud_;
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr path_kdtree_;
    const std::string rddf_path_ = "/home/stier/catkin_ws/src/its_bridge/rddf/rddf.txt";

    void loadPath() {
        std::ifstream file(rddf_path_);
        if (!file.is_open()) {
            ROS_ERROR("Could not open RDDF file");
            return;
        }

        path_cloud_->clear();
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            double x, y;
            if (ss >> x >> y) {
                std::pair<float, float> utm = poseToUtm(x, y);
                pcl::PointXY point = pcl::PointXY{utm.first, utm.second};
                path_cloud_->push_back(point);
            }
        }
        
        if (path_cloud_->empty()) {
            ROS_ERROR("Path cloud is empty after loading");
            return;
        }
        path_kdtree_->setInputCloud(path_cloud_);
    }
};

// --- PathFollower의 싱글턴 및 공개 함수 구현 ---

PathFollower& PathFollower::getInstance() {
    // 이 함수가 처음 호출될 때 단 한번만 instance가 생성됨 (Meyers' Singleton)
    static PathFollower instance;
    return instance;
}

PathFollower::PathFollower() : pimpl_(new Impl()) {}

float PathFollower::findNearestDistance(float x, float y) const {
    if (!pimpl_->path_kdtree_->getInputCloud()) {
        return MAXFLOAT;
    }

    pcl::PointXY search_point = pcl::PointXY{x, y};
    std::vector<int> point_indices(1);
    std::vector<float> point_distances(1);

    if (pimpl_->path_kdtree_->nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
        return point_distances[0];
    }
    return MAXFLOAT;
}