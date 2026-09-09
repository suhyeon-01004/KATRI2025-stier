#include <ros/ros.h>
#include <ublox_msgs/NavPVT.h>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <string>
#include <cstdlib>
#include <cstdio>    // popen, pclose
#include <signal.h>  // kill, SIGINT
#include <unistd.h>  // kill(0)

// "~" -> $HOME 치환
static std::string expand_user(const std::string& path) {
  if (path.empty() || path[0] != '~') return path;
  const char* home = std::getenv("HOME");
  if (!home) return path;
  if (path.size() == 1) return std::string(home);
  if (path[1] == '/')   return std::string(home) + path.substr(1);
  return path;
}

// mkdir -p
static void ensure_dir(const std::string& path) {
  std::string cmd = "mkdir -p \"" + path + "\"";
  int ret = ::system(cmd.c_str());
  if (ret != 0) ROS_WARN("mkdir -p failed (ret=%d) for %s", ret, path.c_str());
}

class SpeedTriggerFixedRecorder {
public:
  SpeedTriggerFixedRecorder(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
    // 트리거/저장 파라미터
    pnh_.param<std::string>("speed_topic",      speed_topic_,     "/ublox_position_receiver/navpvt");
    pnh_.param<double>     ("threshold_kmh",    threshold_kmh_,   20.0);
    pnh_.param<double>     ("hold_up_s",        hold_up_s_,       1.5);
    pnh_.param<int>        ("fixed_duration_s", fixed_duration_s_,60);
    pnh_.param<std::string>("out_dir",          out_dir_,         std::string("~/bags"));
    pnh_.param<std::string>("prefix",           prefix_,          "speed20");
    pnh_.param<std::string>("bag_filename",     bag_filename_,    std::string(""));
    pnh_.param<bool>       ("use_timestamp",    use_timestamp_,   true);

    // rosbag 옵션
    pnh_.param<int>        ("split_size_mb",    split_size_mb_,   0);
    pnh_.param<bool>       ("compress_lz4",     compress_lz4_,    false);
    pnh_.param<bool>       ("record_all",       record_all_,      true);

    // rddf 실행 옵션
    pnh_.param<bool>       ("run_rddf",         run_rddf_,        true);
    pnh_.param<std::string>("rddf_cmd",         rddf_cmd_,        std::string("rosrun stier rddf"));
    pnh_.param<std::string>("rddf_outfile",     rddf_outfile_,    std::string("")); // ★ 추가: rddf 파일 이름만

    if (fixed_duration_s_ <= 0) {
      ROS_WARN("fixed_duration_s <= 0, set to 30");
      fixed_duration_s_ = 30;
    }
    out_dir_ = expand_user(out_dir_);
    ensure_dir(out_dir_);

    sub_ = nh_.subscribe(speed_topic_, 10, &SpeedTriggerFixedRecorder::navpvtCb, this);

    ROS_INFO_STREAM("Ready. Trigger >= " << threshold_kmh_ << " km/h for "
                     << hold_up_s_ << "s -> record " << fixed_duration_s_
                     << "s to " << out_dir_
                     << (run_rddf_ ? " (rddf ON)" : " (rddf OFF)"));
  }

private:
  // 속도 콜백 (gSpeed mm/s → km/h)
  void navpvtCb(const ublox_msgs::NavPVT::ConstPtr& msg) {
    if (triggered_) return;

    double kmh = (msg->gSpeed / 1000.0) * 3.6;
    ros::Time now = ros::Time::now();

    if (kmh >= threshold_kmh_) {
      if (!above_) { above_ = true; above_since_ = now; }
      if ((now - above_since_).toSec() >= hold_up_s_) {
        startProcesses();  // rosbag + (옵션) rddf 함께 실행
        triggered_ = true;
      }
    } else {
      above_ = false;
    }
  }

  // bag 파일 경로 만들기
  std::string make_bag_path() const {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

    std::string base;
    if (!bag_filename_.empty()) {
      base = bag_filename_;
      if (base.size() < 4 || base.substr(base.size()-4) != ".bag") base += ".bag";
      if (use_timestamp_) {
        auto pos = base.rfind(".bag");
        base.insert(pos, "_" + ts.str());
      }
    } else {
      base = prefix_ + "_" + ts.str() + ".bag";
    }
    return out_dir_ + "/" + base;
  }

  // 경로에서 파일명(확장자 없는 stem) 추출 ★
  static std::string stem_of(const std::string& path) {
    auto slash = path.find_last_of('/');
    std::string name = (slash == std::string::npos) ? path : path.substr(slash+1);
    auto dot = name.rfind('.');
    if (dot != std::string::npos) name = name.substr(0, dot);
    return name;
  }

  // 공용: "bash -lc '<cmd> & echo $!'"로 백그라운드 실행 & PID 캡처
  pid_t launchAndGetPid(const std::string& full_cmd) {
    std::string bash = "bash -lc '" + full_cmd + " & echo $!'";
    FILE* pipe = popen(bash.c_str(), "r");
    if (!pipe) return -1;
    char buf[64] = {0};
    if (!fgets(buf, sizeof(buf), pipe)) { pclose(pipe); return -1; }
    pclose(pipe);
    return static_cast<pid_t>(::strtol(buf, nullptr, 10));
  }

  void startProcesses() {
    const std::string bag_path = make_bag_path();

    // 1) rosbag record 커맨드
    std::ostringstream bag_cmd;
    bag_cmd << "rosbag record -O \"" << bag_path << "\" ";
    bag_cmd << "-a ";
    bag_cmd << "--duration=" << fixed_duration_s_ << " ";
    if (split_size_mb_ > 0) bag_cmd << "--split --size " << split_size_mb_ << " ";
    if (compress_lz4_)      bag_cmd << "--lz4 ";

    ROS_INFO("Launching rosbag: %s", bag_cmd.str().c_str());
    rosbag_pid_ = launchAndGetPid(bag_cmd.str());
    if (rosbag_pid_ > 0) {
      ROS_INFO("rosbag PID = %d", rosbag_pid_);
      ROS_INFO("Recording to: %s", bag_path.c_str());
      ROS_INFO("Note: During recording, file may appear as *.bag.active");
    } else {
      ROS_ERROR("Failed to launch rosbag.");
    }

    // 2) (옵션) rddf 실행 — outfile 지정 지원 ★
    if (run_rddf_) {
      // rddf 파일명: 파라미터가 비어있으면 bag 파일명(stem)과 자동 매칭
      std::string outname = rddf_outfile_.empty() ? stem_of(bag_path) : rddf_outfile_;

      std::ostringstream rcmd;
      rcmd << rddf_cmd_;            // 기본: "rosrun stier rddf"
      rcmd << " _outfile:=" << outname;  // ★ 파일명만 넘김 (폴더는 rddf 기본)

      ROS_INFO("Launching rddf: %s", rcmd.str().c_str());
      rddf_pid_ = launchAndGetPid(rcmd.str());
      if (rddf_pid_ > 0) {
        ROS_INFO("rddf PID = %d", rddf_pid_);
      } else {
        ROS_ERROR("Failed to launch rddf (cmd: %s).", rcmd.str().c_str());
      }
    }

    // 3) 종료 타이머: duration + 2초 후, 남아 있으면 정중히 SIGINT
    kill_timer_ = nh_.createTimer(
      ros::Duration(fixed_duration_s_ + 2.0),
      [this](const ros::TimerEvent&) { gracefulStop(); },
      true
    );
  }

  void gracefulStop() {
    // rosbag 종료 확인/유도
    if (rosbag_pid_ > 0) {
      if (kill(rosbag_pid_, 0) == 0) {
        ROS_WARN("rosbag still running; sending SIGINT to %d", rosbag_pid_);
        kill(rosbag_pid_, SIGINT);
      }
      rosbag_pid_ = -1;
    }
    // rddf 종료 유도
    if (rddf_pid_ > 0) {
      if (kill(rddf_pid_, 0) == 0) {
        ROS_INFO("Stopping rddf; sending SIGINT to %d", rddf_pid_);
        kill(rddf_pid_, SIGINT);
      }
      rddf_pid_ = -1;
    }
  }

  // 멤버들
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;

  std::string speed_topic_;
  double threshold_kmh_{20.0};
  double hold_up_s_{1.5};
  int    fixed_duration_s_{60};

  std::string out_dir_{"~/bags"};
  std::string prefix_{"speed20"};
  std::string bag_filename_{};
  bool        use_timestamp_{true};

  int  split_size_mb_{0};
  bool compress_lz4_{false};
  bool record_all_{true};

  bool run_rddf_{true};
  std::string rddf_cmd_{"rosrun stier rddf"};
  std::string rddf_outfile_; // ★ 추가

  bool triggered_{false};
  bool above_{false};
  ros::Time above_since_;

  pid_t rosbag_pid_{-1};
  pid_t rddf_pid_{-1};
  ros::Timer kill_timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosbag_record_speed");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SpeedTriggerFixedRecorder node(nh, pnh);
  ros::spin();
  return 0;
}

