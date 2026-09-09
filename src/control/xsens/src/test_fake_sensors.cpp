#include <ros/ros.h>
#include <ublox_msgs/NavSTATUS.h>

class FakeStatusPublisher {
public:
    FakeStatusPublisher() {
        ros::NodeHandle nh;
        // rosbag NavSTATUS 구독
        sub_ = nh.subscribe("/ublox_position_receiver/navstatus", 10, &FakeStatusPublisher::statusCallback, this);
        // 같은 토픽으로 재퍼블리시
        pub_ = nh.advertise<ublox_msgs::NavSTATUS>("/ublox_position_receiver/navstatus", 10);
        start_time_ = ros::Time::now();
    }

    void statusCallback(const ublox_msgs::NavSTATUS::ConstPtr& msg) {
        ros::Duration elapsed = ros::Time::now() - start_time_;

        ublox_msgs::NavSTATUS modified_msg = *msg;

        // 실행 후 10초 동안만 교란
        if (elapsed.toSec() <= 10.0) {
            modified_msg.fixStat = 2;
            modified_msg.flags2 = 8;
        }

        pub_.publish(modified_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Time start_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_fake_sensors");
    FakeStatusPublisher node;
    ros::spin();
    return 0;
}
