#include <ros/ros.h>
#include <ublox_msgs/NavSTATUS.h>
#include <boost/bind.hpp>

class GpsStatusDropper {
public:
    GpsStatusDropper(ros::NodeHandle& nh) {
        pub_ = nh.advertise<ublox_msgs::NavSTATUS>("/ublox_position_receiver/navstatus", 10);
        sub_ = nh.subscribe("/ublox_position_receiver/navstatus", 10, &GpsStatusDropper::callback, this);
    }

    void callback(const ublox_msgs::NavSTATUS::ConstPtr& msg) {
        ublox_msgs::NavSTATUS modified = *msg;
        modified.fixStat = 2;
        modified.flags2 = 16;
        pub_.publish(modified);
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_status_drop_test");
    ros::NodeHandle nh;

    GpsStatusDropper dropper(nh);

    ros::spin();
    return 0;
}
