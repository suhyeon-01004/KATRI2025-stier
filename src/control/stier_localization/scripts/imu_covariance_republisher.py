#!/usr/bin/env python
import rospy, yaml
from sensor_msgs.msg import Imu

def _as_list9(param_name, default):
    val = rospy.get_param("~" + param_name, default)
    # 문자열이면 YAML로 파싱
    if isinstance(val, str):
        try:
            val = yaml.safe_load(val)
        except Exception as e:
            rospy.logwarn("%s parse failed (%s). Using default.", param_name, e)
            return default
    # 길이/형식 검증
    if not isinstance(val, (list, tuple)) or len(val) != 9:
        rospy.logwarn("%s must be list/tuple of length 9. Using default.", param_name)
        return default
    return [float(x) for x in val]

class ImuCovarianceRepublisher:
    def __init__(self):
        rospy.init_node("imu_covariance_republisher")
        self.orientation_cov = _as_list9("orientation_covariance",
                                         [0.0004,0,0, 0,0.0004,0, 0,0,0.01])
        self.angular_vel_cov = _as_list9("angular_velocity_covariance",
                                         [0.0004,0,0, 0,0.0004,0, 0,0,0.0025])
        self.linear_acc_cov = _as_list9("linear_acceleration_covariance",
                                        [0.09,0,0, 0,0.09,0, 0,0,0.25])

        self.pub = rospy.Publisher("/imu/data_cov", Imu, queue_size=10)
        rospy.Subscriber("/imu/data", Imu, self.callback)

    def callback(self, msg):
        msg.orientation_covariance = self.orientation_cov
        msg.angular_velocity_covariance = self.angular_vel_cov
        msg.linear_acceleration_covariance = self.linear_acc_cov
        self.pub.publish(msg)

if __name__ == "__main__":
    ImuCovarianceRepublisher()
    rospy.spin()
