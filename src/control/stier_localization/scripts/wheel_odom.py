#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

from erp42_msgs.msg import SerialFeedBack
from sensor_msgs.msg import Imu


class WheelOdomTwistNode(object):
    def __init__(self):
        # ---------- Params ----------
        self.feedback_topic   = rospy.get_param("~feedback_topic", "/erp42_serial/feedback")
        self.pub_topic        = rospy.get_param("~pub_topic",      "wheel/odom")

        # frames
        self.frame_odom       = rospy.get_param("~frame_odom", "odom")
        self.frame_base       = rospy.get_param("~frame_base", "base_link")

        # encoder spec
        self.pulses_per_rev   = float(rospy.get_param("~pulses_per_rev",   2048.0))
        self.wheel_diameter_m = float(rospy.get_param("~wheel_diameter_m", 0.5))
        self.encoder_modulus  = int(rospy.get_param("~encoder_modulus",    65536))
        self.encoder_sign     = int(rospy.get_param("~encoder_sign",       1))  # invert if needed

        # vehicle geometry (좌전륜 엔코더 기준)
        self.wheelbase_m      = float(rospy.get_param("~wheelbase_m", 1.04))   # L
        self.front_track_m    = float(rospy.get_param("~front_track_m", 0.985))# Wf
        self.wheel_side       = rospy.get_param("~wheel_side", "front_left")   # "front_left" | "front_right"

        # steer → rad 변환
        self.steer_is_deg     = bool(rospy.get_param("~steer_is_deg", False))  # ERP42 스케일에 맞게
        self.steer_scale      = float(rospy.get_param("~steer_scale", 1.0))    # 곱
        self.steer_offset     = float(rospy.get_param("~steer_offset", 0.0))   # 단위는 steer_is_deg에 따름
        self.steer_sign       = float(rospy.get_param("~steer_sign", 1.0))     # 좌회전 양(+)이 되도록 ±1
        self.limit_tan        = float(rospy.get_param("~limit_tan", 10.0))     # |tan(delta)| 클램프

        # ω (yaw rate) 구독 설정
        self.imu_topic        = rospy.get_param("~imu_topic", "/imu/data")
        self.imu_timeout_sec  = float(rospy.get_param("~imu_timeout_sec", 0.2)) # 최근 ω 없으면 0

        # 속도 소스 선택
        self.use_encoder_velocity = bool(rospy.get_param("~use_encoder_velocity", True))
        self.speed_is_kmh         = bool(rospy.get_param("~speed_is_kmh", True))

        # 퍼블리시 고정 주기 & 스테일 가드
        self.pub_rate_hz     = float(rospy.get_param("~pub_rate_hz", 50.0))
        self.max_stale_sec   = float(rospy.get_param("~max_stale_sec", 0.5))  # 최근 샘플 없으면 publish skip

        # 지수평활 필터 (속도)
        self.smooth_alpha    = float(rospy.get_param("~smooth_alpha", 0.3))   # 0(부드러움)~1(원시)
        # 지수평활 필터 (yaw rate)
        self.smooth_alpha_omega = float(rospy.get_param("~smooth_alpha_omega", 0.2))

        # 물리 한계(아웃라이어 가드)
        self.max_speed_mps   = float(rospy.get_param("~max_speed_mps", 60.0))  # 216 km/h

        # covariance
        self.twist_cov_lin   = float(rospy.get_param("~twist_cov_linear", 0.05))
        self.twist_cov_yaw   = float(rospy.get_param("~twist_cov_yaw",    0.2))

        # ---------- State ----------
        self.last_ticks  = None
        self.last_stamp  = None          # encoder 측정 시각

        self.vx_filt     = 0.0           # 50 Hz로 퍼블리시될 필터 속도
        self.vyaw        = 0.0           # 여기선 0 (yaw rate는 IMU에서 쓰되, odom.angular.z는 0 유지)

        # IMU yaw rate state
        self.omega_z     = 0.0
        self.omega_stamp = None

        if self.pulses_per_rev <= 0.0:
            rospy.logfatal("~pulses_per_rev must be > 0")
            raise SystemExit
        self.circ = math.pi * self.wheel_diameter_m

        # ---------- IO ----------
        self.pub_odom = rospy.Publisher(self.pub_topic, Odometry, queue_size=50)
        rospy.Subscriber(self.feedback_topic, SerialFeedBack, self.cb_feedback)
        rospy.Subscriber(self.imu_topic, Imu, self.cb_imu)

        # 50 Hz 고정 주기 퍼블리셔 (ZOH)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate_hz), self.timer_publish)

        rospy.loginfo("wheel_odom_twist_only(ZOH+front-left correction): fb=%s, imu=%s -> pub=%s | "
                      "PPR=%.1f D=%.3f m modulus=%d enc_sign=%+d pub=%.1f Hz L=%.3f Wf=%.3f side=%s",
                      self.feedback_topic, self.imu_topic, self.pub_topic,
                      self.pulses_per_rev, self.wheel_diameter_m,
                      self.encoder_modulus, self.encoder_sign, self.pub_rate_hz,
                      self.wheelbase_m, self.front_track_m, self.wheel_side)

    # ---------- IMU (yaw rate) ----------
    def cb_imu(self, msg):
        try:
            omega = float(msg.angular_velocity.z)
            if math.isfinite(omega):
                a = max(0.0, min(1.0, self.smooth_alpha_omega))
                self.omega_z = a*omega + (1.0 - a)*self.omega_z
                self.omega_stamp = msg.header.stamp if hasattr(msg, "header") else rospy.Time.now()
        except Exception:
            pass

    # ---------- Encoder callback: 최신 속도 추정/보정/필터 ----------
    def cb_feedback(self, msg):
        # 가능한 한 센서가 준 측정 시각 사용
        stamp = msg.header.stamp if hasattr(msg, "header") else rospy.Time.now()

        # 첫 샘플 초기화
        if self.last_stamp is None:
            self.last_stamp = stamp
            self.last_ticks = int(getattr(msg, "encoder"))
            return

        dt = (stamp - self.last_stamp).to_sec()
        if dt <= 0.0:
            # 과거/동일 타임스탬프 샘플은 무시
            return

        # encoder ticks
        ticks = int(getattr(msg, "encoder"))
        dticks = ticks - self.last_ticks

        # wrap-around 보정
        if self.encoder_modulus and abs(dticks) > (self.encoder_modulus // 2):
            if dticks > 0:
                dticks -= self.encoder_modulus
            else:
                dticks += self.encoder_modulus
        self.last_ticks = ticks

        # 바퀴 선속도 s (엔코더 원시 → 둘레 기반)
        rot = (dticks * self.encoder_sign) / self.pulses_per_rev
        ds  = rot * self.circ  # [m]
        if self.use_encoder_velocity:
            s = ds / dt
        else:
            spd = float(getattr(msg, "speed"))
            s = (spd * (1000.0/3600.0)) if self.speed_is_kmh else spd

        # 비현실 속도/NaN 가드
        if (not math.isfinite(s)) or abs(s) > self.max_speed_mps:
            self.last_stamp = stamp
            return

        # 조향각 delta (좌전륜)
        steer_raw = float(getattr(msg, "steer"))
        delta = (self.steer_sign *
                 (self.steer_scale * steer_raw + self.steer_offset))
        if self.steer_is_deg:
            delta = math.radians(delta)

        # 안전 클램프
        c = math.cos(delta)
        if abs(c) < 1e-6:
            c = 1e-6
        t = math.tan(delta)
        if self.limit_tan > 0.0:
            t = max(min(t, self.limit_tan), -self.limit_tan)

        # 최신 ω (없으면 0)
        omega = 0.0
        if self.omega_stamp is not None:
            if (stamp - self.omega_stamp).to_sec() <= self.imu_timeout_sec:
                omega = self.omega_z

        # 좌/우 전륜에 따른 보정식
        Wf = self.front_track_m
        L  = self.wheelbase_m

        if self.wheel_side == "front_left":
            vx_corr = (s / c) + omega*(Wf/2.0) - omega*L*t
        elif self.wheel_side == "front_right":
            vx_corr = (s / c) - omega*(Wf/2.0) - omega*L*t
        else:
            # 기본값: 좌전륜 가정
            vx_corr = (s / c) + omega*(Wf/2.0) - omega*L*t

        # 지수평활
        a = max(0.0, min(1.0, self.smooth_alpha))
        self.vx_filt = a * vx_corr + (1.0 - a) * self.vx_filt

        self.last_stamp = stamp

    # ---------- Fixed-rate publisher (50 Hz) ----------
    def timer_publish(self, event):
        now = rospy.Time.now()

        # 최근 샘플 오래되면 퍼블리시 생략 (센서 끊김 보호)
        if self.last_stamp is None or (now - self.last_stamp).to_sec() > self.max_stale_sec:
            return

        odom = Odometry()
        odom.header.stamp = now                     # 현재 시간에 유효한 속도라는 의미로 퍼블리시
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base

        # pose는 EKF에서 무시하도록 더미 + 큰 공분산
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        odom.pose.covariance = [999999.0]*36

        # twist: base_link 기준
        odom.twist.twist.linear.x  = self.vx_filt
        odom.twist.twist.angular.z = 0.0  # yaw rate는 IMU에서 EKF로
        odom.twist.covariance = [
            self.twist_cov_lin, 0, 0, 0, 0, 0,
            0, self.twist_cov_lin, 0, 0, 0, 0,
            0, 0, 999999.0, 0, 0, 0,
            0, 0, 0, 999999.0, 0, 0,
            0, 0, 0, 0, 999999.0, 0,
            0, 0, 0, 0, 0, self.twist_cov_yaw
        ]

        self.pub_odom.publish(odom)


if __name__ == "__main__":
    rospy.init_node("wheel_odom_twist_only")
    try:
        WheelOdomTwistNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
