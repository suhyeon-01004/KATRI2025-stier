#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include "utm_lla_converter.h"
#include <nav_msgs/Path.h>
#include <erp42_msgs/SerialFeedBack.h>
#include "rddf.h"

class GPSIMUFusion {
public:
    GPSIMUFusion()
        : gps_ready_(false),
          use_gps_(false),
          prev_use_gps_(false),
          fixStat_(0),
          flags2_(0),
          state_(Eigen::VectorXd::Zero(5)),
          P_(Eigen::MatrixXd::Identity(5, 5)),
          converter_("North", 52, 6378137.0, 1.0 / 298.257223563, 0.9996),
          initialized_(false),
          offset_x_(0.0),
          offset_y_(0.0),
          last_rddf_idx_(-1)
    {
        ros::NodeHandle nh;
        ros::NodeHandle nh_private("~");

        // 파라미터
        nh_private.param<std::string>("hemisphere", hemisphere_, std::string("North"));
        nh_private.param<int>("utm_zone", utm_zone_, 52);

        // RDDF 로드
        std::string rddf_path;
        nh_private.param<std::string>("rddf_path", rddf_path,
            std::string("/catkin_ws/src/control/rddf_recorder/paths/2025-7-30_17-41.txt"));
        if(rddf_.load(rddf_path) != 0) {
            ROS_ERROR("Failed to load RDDF: %s", rddf_path.c_str());
        } else {
            ROS_INFO("Loaded RDDF: %d points", rddf_.getCount());
        }

        // Subscriber
        sub_imu_ = nh.subscribe("/imu/data", 100, &GPSIMUFusion::imuCallback, this);
        sub_gps_ = nh.subscribe("/ublox_position_receiver/navpvt", 100, &GPSIMUFusion::gpsCallback, this);
        sub_status_ = nh.subscribe("/ublox_position_receiver/navstatus", 100, &GPSIMUFusion::statusCallback, this);
        sub_erp42_feedback_ = nh.subscribe("/erp42_serial/feedback", 50, &GPSIMUFusion::erp42Callback, this);

        // Publisher
        pub_odom_ = nh.advertise<nav_msgs::Odometry>("/odom/fused", 50);
        path_pub_ = nh.advertise<nav_msgs::Path>("/odom/path", 10);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/rddf_markers", 1);

        path_msg_.header.frame_id = "odom";
        state_(4) = 0.0; // 초기 yaw
    }

private:
    ros::Subscriber sub_imu_, sub_gps_, sub_status_, sub_erp42_feedback_;
    ros::Publisher pub_odom_;
    ros::Publisher marker_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    bool gps_ready_;
    bool use_gps_;
    bool prev_use_gps_;
    bool initialized_;
    double offset_x_, offset_y_;
    int fixStat_;
    int flags2_;
    ros::Time last_time_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_msg_;

    ULConverter converter_;
    std::string hemisphere_;
    int utm_zone_;
    double current_speed_mps_ = 0.0;
    Rddf rddf_;
    bool need_alignment_ = false;

    // 로컬 RDDF 데이터
    std::vector<double> rddf_x_local_;
    std::vector<double> rddf_y_local_;
    bool rddf_offset_applied_ = false;

    // 최근 RDDF 인덱스
    int last_rddf_idx_;

    // --- 상태 콜백 ---
    void statusCallback(const ublox_msgs::NavSTATUS::ConstPtr &msg) {
        fixStat_ = msg->fixStat;
        flags2_ = msg->flags2;
        use_gps_ = (flags2_ >= 72);

        if (prev_use_gps_ != use_gps_) {
            if (!use_gps_) {
                ROS_WARN("GPS LOST -> DR 모드 전환");
                snapToRDDF();   // 가장 가까운 또는 마지막 RDDF idx로 스냅
            } else {
                ROS_INFO("GPS RECOVERED -> Alignment 예정");
                need_alignment_ = true;
            }
            prev_use_gps_ = use_gps_;
        }
    }

    // --- GPS 콜백 ---
    void gpsCallback(const ublox_msgs::NavPVT::ConstPtr &msg) {
        double lat = msg->lat * 1e-7;
        double lon = msg->lon * 1e-7;

        double utm_x, utm_y;
        latLonToUTM(lat, lon, utm_x, utm_y);

        if (!gps_ready_) {
            // 첫 GPS 수신 시 offset 설정
            offset_x_ = utm_x;
            offset_y_ = utm_y;

            // RDDF 좌표 로컬화
            applyOffsetToRDDF(offset_x_, offset_y_);

            // 초기 위치 RDDF 첫 포인트
            if (!rddf_x_local_.empty()) {
                state_(0) = rddf_x_local_[0];
                state_(1) = rddf_y_local_[0];
                last_rddf_idx_ = 0;
            }

            state_.segment<3>(2) << 0.0, 0.0, 0.0;
            P_ = Eigen::MatrixXd::Identity(5,5) * 0.01;

            gps_ready_ = true;
            initialized_ = true;

            // RDDF 시각화
            publishRDDFMarkers();
            ROS_INFO("GPS initialized with offset (%.2f, %.2f) -> RDDF localized", offset_x_, offset_y_);
            return;
        }

        double gps_local_x = utm_x - offset_x_;
        double gps_local_y = utm_y - offset_y_;

        if (use_gps_) {
            if (need_alignment_) {
                performAlignment(gps_local_x, gps_local_y);
                need_alignment_ = false;
                P_ = Eigen::MatrixXd::Identity(5,5) * 0.01;
            }
            Eigen::Vector2d z(gps_local_x, gps_local_y);
            update(z);
        }

        // GPS 사용 시에도 RDDF idx 갱신
        updateLastRDDFIdx();
    }

    // --- IMU 콜백 ---
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        ros::Time now = msg->header.stamp;
        if (!gps_ready_) return;
        double dt = (last_time_.isZero()) ? 0.01 : (now - last_time_).toSec();
        last_time_ = now;

        double yaw_rate = msg->angular_velocity.z;
        double new_yaw = state_(4) + yaw_rate * dt;

        if (new_yaw > M_PI) new_yaw -= 2*M_PI;
        if (new_yaw < -M_PI) new_yaw += 2*M_PI;

        predict(new_yaw, dt);
        if (!use_gps_) applyRDDFSoftCorrection();

        // DR 모드에서도 RDDF idx 갱신
        updateLastRDDFIdx();

        publishOdom(now);
    }

    // --- ERP42 속도 ---
    void erp42Callback(const erp42_msgs::SerialFeedBack::ConstPtr &msg) {
        current_speed_mps_ = msg->speed / 3.6;
    }

    // --- Snap ---
    void snapToRDDF() {
        if (last_rddf_idx_ >= 0 && last_rddf_idx_ < (int)rddf_x_local_.size()) {
            state_(0) = rddf_x_local_[last_rddf_idx_];
            state_(1) = rddf_y_local_[last_rddf_idx_];
            ROS_WARN("Snapped to last RDDF idx=%d (%.2f, %.2f)",
                     last_rddf_idx_, state_(0), state_(1));
        } else {
            int idx = rddf_.calculateNearestIdx(state_(0), state_(1));
            state_(0) = rddf_x_local_[idx];
            state_(1) = rddf_y_local_[idx];
            last_rddf_idx_ = idx;
            ROS_WARN("Snapped to nearest RDDF idx=%d (%.2f, %.2f)",
                     idx, state_(0), state_(1));
        }
    }

    // --- RDDF idx 갱신 ---
    void updateLastRDDFIdx() {
        if (!rddf_x_local_.empty()) {
            int nearest_idx = rddf_.calculateNearestIdx(state_(0), state_(1));
            last_rddf_idx_ = nearest_idx;
        }
    }

    // --- Predict ---
    void predict(double yaw, double dt) {
        double x = state_(0);
        double y = state_(1);

        double speed_x = current_speed_mps_ * cos(yaw);
        double speed_y = current_speed_mps_ * sin(yaw);

        x += speed_x * dt;
        y += speed_y * dt;

        state_ << x, y, speed_x, speed_y, yaw;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5,5);
        F(0,2) = dt;
        F(1,3) = dt;
        P_ = F * P_ * F.transpose() + Eigen::MatrixXd::Identity(5,5) * 0.05;
    }

    // --- Update ---
    void update(const Eigen::Vector2d &z) {
        Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
        H(0,0) = 1;
        H(1,1) = 1;

        double gps_quality = (flags2_ >= 72) ? 1.0 : 0.3;
        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (0.5 / gps_quality);

        Eigen::Vector2d y = z - H * state_;
        Eigen::Matrix2d S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, 5, 2> K = P_ * H.transpose() * S.inverse();

        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(5,5) - K * H) * P_;
    }

    // --- Soft correction ---
    void applyRDDFSoftCorrection() {
        if (last_rddf_idx_ < 0 || last_rddf_idx_ >= (int)rddf_x_local_.size()) return;
        double rddf_x = rddf_x_local_[last_rddf_idx_];
        double rddf_y = rddf_y_local_[last_rddf_idx_];
        double alpha = use_gps_ ? 0.1 : 0.3; // GPS 없을 때 강도 ↑
        state_(0) = state_(0)*(1-alpha)+rddf_x*alpha;
        state_(1) = state_(1)*(1-alpha)+rddf_y*alpha;
    }

    // --- Alignment ---
    void performAlignment(double gps_x, double gps_y) {
        int idx = rddf_.calculateNearestIdx(gps_x, gps_y);
        if (idx < 0 || idx >= (int)rddf_x_local_.size()) return;
        double dx = rddf_x_local_[idx] - gps_x;
        double dy = rddf_y_local_[idx] - gps_y;
        state_(0) += dx;
        state_(1) += dy;
        last_rddf_idx_ = idx;
        ROS_INFO("Alignment: Δx=%.2f, Δy=%.2f", dx, dy);
    }

    // --- RDDF 로컬 변환 함수 ---
    void applyOffsetToRDDF(double offset_x, double offset_y) {
        rddf_x_local_.clear();
        rddf_y_local_.clear();
        for (int i = 0; i < rddf_.getCount(); i++) {
            rddf_x_local_.push_back(rddf_.getX(i) - offset_x);
            rddf_y_local_.push_back(rddf_.getY(i) - offset_y);
        }
        rddf_offset_applied_ = true;
        ROS_INFO("Applied offset to RDDF: (%.2f, %.2f)", offset_x, offset_y);
    }

    // --- RDDF Marker ---
    void publishRDDFMarkers() {
        if (!rddf_offset_applied_) return;

        visualization_msgs::MarkerArray markers;
        for (int i = 0; i < (int)rddf_x_local_.size(); i++) {
            visualization_msgs::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp = ros::Time::now();
            m.ns = "rddf";
            m.id = i;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = rddf_x_local_[i];
            m.pose.position.y = rddf_y_local_[i];
            m.scale.x = m.scale.y = m.scale.z = 0.2;
            m.color.a = 1.0;
            m.color.g = 1.0;
            markers.markers.push_back(m);
        }
        marker_pub_.publish(markers);
    }

    // --- Odom publish ---
    void publishOdom(const ros::Time &stamp) {
        if (!gps_ready_) return;
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = state_(0);
        odom.pose.pose.position.y = state_(1);
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_(4));
        odom.twist.twist.linear.x = state_(2);
        odom.twist.twist.linear.y = state_(3);
        pub_odom_.publish(odom);

        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = stamp;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = state_(0);
        odom_tf.transform.translation.y = state_(1);
        odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(state_(4));
        tf_broadcaster_.sendTransform(odom_tf);

        geometry_msgs::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = stamp;
        path_pub_.publish(path_msg_);
    }

    // --- LatLon to UTM ---
    void latLonToUTM(double lat, double lon, double &x, double &y) {
        Hemi hemi_enum = (hemisphere_ == "South") ? SouthH : NorthH;
        converter_.LLAConvert2UTM(hemi_enum, utm_zone_, lat, lon, 0.0);
        std::vector<double> utm_vals = converter_.get_utm();
        x = utm_vals[0];
        y = utm_vals[1];
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_imu_fusion");
    GPSIMUFusion fusion;
    ros::spin();
    return 0;
}
