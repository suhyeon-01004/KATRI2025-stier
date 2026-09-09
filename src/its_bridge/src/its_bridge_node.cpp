#include <ros/ros.h>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <unordered_map>
#include <set>
#include <optional>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include "utm_converter.hpp"
#include "path_follower.hpp"
#include "id_generator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <its_bridge/CustomSPaT.h>
#include <its_bridge/CustomState.h>

using boost::asio::ip::address;
using boost::asio::ip::udp;

extern "C" {
    #include "MessageFrame.h"
    #include "asn_application.h"
    #include "per_decoder.h"
}

struct ConnectedLane {
    std::string id;
    SignalGroupID_t signalGroup;
};

struct Lane {
    std::string id;
    float_t x;
    float_t y;
    std::vector<ConnectedLane> next_lanes;
};

class ItsBridge {
public:
    ItsBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh),
          socket_(io_service_),
          recv_buffer_{},
          pathFollower_(PathFollower::getInstance())
    {
        loadParams();

        socket_.open(udp::v4());
        socket_.bind(udp::endpoint(udp::v4(), 60000));

        pub_its_ = nh_.advertise<its_bridge::CustomSPaT>(its_topic_, ROS_QUEUE_SIZE_);
        pub_drive_state_ = nh.advertise<std_msgs::Int32>(drive_state_topic_, 1);
        sub_gps_ = nh_.subscribe(utm_topic_, 1, &ItsBridge::callbackUtm, this);
        sub_speed_ = nh_.subscribe(speed_topic_, 1, &ItsBridge::callbackSpeed, this);

        map_update_timer_ = nh_.createTimer(ros::Duration(3.0), &ItsBridge::callbackTimer, this);

        spatial_cloud_.reset(new pcl::PointCloud<pcl::PointXY>());
        kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXY>());
        
        startReceive();

        io_thread_ = std::thread([this]() {
            io_service_.run();
        });
    }

    ~ItsBridge() {
        io_service_.stop();
        if (io_thread_.joinable()) io_thread_.join();
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_its_, pub_drive_state_;
    ros::Subscriber sub_gps_, sub_speed_;
    ros::Timer map_update_timer_;
    std::string its_topic_, utm_topic_, speed_topic_, drive_state_topic_;
    static constexpr std::size_t ROS_QUEUE_SIZE_ = 10;

    // socket
    boost::asio::io_service io_service_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::array<uint8_t, 2048> recv_buffer_;
    std::thread io_thread_;
    const std::string OBU_ENDPOINT_ = "192.168.1.2";

    float_t currentX_, currentY_, currentSpeed_;

    IntersectionID_t targetIntersectionId_;
    LaneID_t targetLaneId_;
    SignalGroupID_t targetSignalGroup_;

    static constexpr float_t DECELERATION_DISTANCE_ = 3.0;
    static constexpr float_t STOP_DISTANCE_ = 1.0;
    
    PathFollower& pathFollower_;

    // kd-tree
    std::unordered_map<std::string, Lane> lanes_;
    pcl::PointCloud<pcl::PointXY>::Ptr spatial_cloud_;
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_;
    std::vector<std::string> lane_id_map_; 

    void startReceive() {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                if (!ec && remote_endpoint_.address().to_string() == OBU_ENDPOINT_ && bytes_recvd > 16) {
                    handlePacket(recv_buffer_.data(), bytes_recvd);
                };
                startReceive();
            }
        );
    }

    void handlePacket(const uint8_t* data, std::size_t length) {
        // Header Parsing
        auto getInt32 = [](const uint8_t* d, size_t offset) -> int32_t {
            return (d[offset] << 24) | (d[offset+1] << 16) | (d[offset+2] << 8) | (d[offset+3]);
        };

        int32_t fsSec = getInt32(data, 0);
        int32_t fsUsec = getInt32(data, 4);
        int32_t dataCnt = getInt32(data, 8);
        int32_t dataLength = getInt32(data, 12);

        // ROS_INFO("length: %zu, fsSec: %d, fsUsec: %d, dataCnt: %d, dataLength: %d", length, fsSec, fsUsec, dataCnt, dataLength);

        if (16 + dataLength > length) {
            ROS_WARN("Invalid packet length");
            return;
        }

        // Payload Extract
        std::vector<uint8_t> payload(data + 16, data + 16 + dataLength);

        // ASN.1 UPER Decode
        MessageFrame_t* msg = (MessageFrame_t*)calloc(1, sizeof(MessageFrame_t));
        asn_dec_rval_t rval = uper_decode(nullptr, &asn_DEF_MessageFrame, (void**)&msg, payload.data(), payload.size(), 0, 0);
        if (rval.code != RC_OK) {
            ROS_ERROR("UPER decode failed");
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, msg);
            return;
        }

        if (msg->value.present == MessageFrame__value_PR_MapData) {
            MapData_t* mapData = &msg->value.choice.MapData;

            if (!mapData->intersections 
                || mapData->intersections->list.count == 0
                || !mapData->intersections->list.array[0]) {
                return;
            }

            IntersectionID_t intersectionId = mapData->intersections->list.array[0]->id.id;

            Position3D_t refPoint = mapData->intersections->list.array[0]->refPoint;
            LaneList_t laneList = mapData->intersections->list.array[0]->laneSet;

            for (int i=0; i<laneList.list.count; i++) { 
                GenericLane_t* lane = laneList.list.array[i];
                if (lane->nodeList.present == NodeListXY_PR_NOTHING) continue;

                NodeXY_t* node = lane->nodeList.choice.nodes.list.array[0];
                if (node->delta.present == NodeOffsetPointXY_PR_NOTHING) continue;
                
                int64_t nodeLat = refPoint.lat + node->delta.choice.node_LatLon.lat;
                int64_t nodeLon = refPoint.Long + node->delta.choice.node_LatLon.lon;

                std::pair<float, float> utm = rawToUtm(nodeLat, nodeLon);

                Lane savedLane;
                savedLane.id = generateId(intersectionId, lane->laneID);
                savedLane.x = utm.first;
                savedLane.y = utm.second;
                
                if (lane->connectsTo && lane->connectsTo->list.count > 0) {
                    for (int j=0; j<lane->connectsTo->list.count; j++){
                        auto connectedLane = lane->connectsTo->list.array[j];
                        if (!connectedLane || !connectedLane->signalGroup) continue;
                        SignalGroupID_t signalGroup = *(connectedLane->signalGroup);
                        std::string connectedLaneId = generateId(intersectionId, connectedLane->connectingLane.lane);
                        ConnectedLane connected = ConnectedLane{connectedLaneId, signalGroup};
                        savedLane.next_lanes.push_back(connected);
                    }
                }
                
                lanes_[savedLane.id] = savedLane;
            }
            
        }
        else if(msg->value.present == MessageFrame__value_PR_SPAT) {
            auto nearestLaneInCurrentPositionOpt = findNearestLane(currentX_, currentY_);
            if (!nearestLaneInCurrentPositionOpt) {
                ROS_WARN("Can't find nearest lane");
                return;
            }

            std::string currentLaneId = nearestLaneInCurrentPositionOpt.value();

            auto currentLaneOpt = getLane(currentLaneId);
            if (!currentLaneOpt.has_value()) {
                ROS_WARN("Current lane is null");
                return;
            }
            Lane currentLane = currentLaneOpt.value();

            auto nextLanes = currentLane.next_lanes;
            if (nextLanes.empty()) {
                ROS_WARN("Empty next lanes");
                return;
            };

            // decide next lane
            float_t minDistance = MAXFLOAT;
            IntersectionID_t tempTargetIntersectionId;
            LaneID_t tempTargetLaneId;
            SignalGroupID_t tempTargetSignalGroup;

            for (const ConnectedLane& connectedLane : nextLanes) {
                // ROS_INFO("connectedLaneId : %s", connectedLane.id.c_str());
                auto it = lanes_.find(connectedLane.id);
                if (it == lanes_.end()) {
                    ROS_WARN("Not exist target lane");
                    continue;
                }
                Lane nextLaneCandidate = it->second;
                std::string nextLaneCandidateId = nextLaneCandidate.id;
                float_t distance = pathFollower_.findNearestDistance(nextLaneCandidate.x, nextLaneCandidate.y);
                if (minDistance > distance) {
                    std::pair<long, long> intersectionIdAndLaneId = decomposeId(nextLaneCandidateId);
                    tempTargetIntersectionId = intersectionIdAndLaneId.first;
                    tempTargetLaneId = intersectionIdAndLaneId.second;
                    tempTargetSignalGroup = connectedLane.signalGroup;
                    minDistance = distance;
                }
            }
            
            targetIntersectionId_ = tempTargetIntersectionId;
            targetLaneId_ = tempTargetLaneId;
            targetSignalGroup_ = tempTargetSignalGroup;
            
            // check and decide signal
            SPAT_t* spat = &msg->value.choice.SPAT;

            IntersectionState_t* intersection = spat->intersections.list.array[0];
            IntersectionID_t intersectionId = intersection->id.id;

            if (intersectionId == targetIntersectionId_) {
                its_bridge::CustomSPaT spatMsg;
                MovementList_t* intersectionStates = &intersection->states;

                for (int i=0; i<intersectionStates->list.count; i++) {
                    MovementState_t* movementState = intersectionStates->list.array[i];
                    MovementEvent_t* stateTimeSpeed = movementState->state_time_speed.list.array[0];

                    int64_t signalGroup = static_cast<int64_t>(movementState->signalGroup); 
                    if (signalGroup == targetSignalGroup_) {    
                        std::string movementName((char*)movementState->movementName->buf, movementState->movementName->size);
                        MovementPhaseState_t eventState = stateTimeSpeed->eventState; if (eventState == MovementPhaseState::MovementPhaseState_unavailable) continue;
                        int64_t minEndTime = static_cast<int64_t>(stateTimeSpeed->timing->minEndTime)/10;
                        
                        ROS_INFO("%s -> %s:%s, signalGroup: %ld", currentLaneId.c_str(), std::to_string(targetIntersectionId_).c_str(), std::to_string(targetLaneId_).c_str(), signalGroup);

                        its_bridge::CustomState stateMsg;
                        spatMsg.messageId = std::to_string(msg->messageId);
                        stateMsg.signalGroup = signalGroup;
                        stateMsg.movementName = movementName;
                        stateMsg.eventState = eventState; 
                        stateMsg.minEndTime = minEndTime;
                        spatMsg.value.push_back(stateMsg);
                        
                        std_msgs::Int32 driveStateMsg;
                        float remainDistance = calculateDistance(currentLane.x, currentLane.y);

                        switch (eventState)
                        {
                        case e_MovementPhaseState::MovementPhaseState_stop_And_Remain:
                            if (currentSpeed_ * minEndTime > remainDistance) {
                                driveStateMsg.data = 1;
                                break;
                            }
                            if (remainDistance < STOP_DISTANCE_) {
                                driveStateMsg.data = 0;
                                break;
                            }
                            if (remainDistance < DECELERATION_DISTANCE_) {
                                driveStateMsg.data = 2;
                                break;
                            }
                            break;
                        case e_MovementPhaseState::MovementPhaseState_protected_Movement_Allowed:
                            driveStateMsg.data = 1;
                            break;
                        case e_MovementPhaseState::MovementPhaseState_protected_clearance:
                            if (currentSpeed_ * minEndTime > remainDistance) {
                                driveStateMsg.data = 1;
                                break;
                            }
                            if (remainDistance < STOP_DISTANCE_) {
                                driveStateMsg.data = 0;
                                break;
                            }
                            if (remainDistance < DECELERATION_DISTANCE_) {
                                driveStateMsg.data = 2;
                                break;
                            }
                            break;
                        default:
                            break;
                        }
                        pub_drive_state_.publish(driveStateMsg);
                    }
                    pub_its_.publish(spatMsg);
                }
            }
        }

        ASN_STRUCT_FREE(asn_DEF_MessageFrame, msg);
    }

    void loadParams() {
        pnh_.param<std::string>("its_topic", its_topic_, "/its");
        pnh_.param<std::string>("utm_topic", utm_topic_, "/utm");
        pnh_.param<std::string>("speed_topic", speed_topic_, "/gps_speed");
        pnh_.param<std::string>("drive_state_topic", drive_state_topic_, "/drive_state");
    }

    void callbackUtm(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::pair<float, float> utm = poseToUtm(msg->pose.position.x, msg->pose.position.y);
        this->currentX_ = utm.first;
        this->currentY_ = utm.second;
        // ROS_INFO("%f, %f", this->currentX_, this->currentY_);
    }

    void callbackSpeed(const std_msgs::Float32::ConstPtr& msg) {
        this->currentSpeed_ = msg->data;
        // ROS_INFO("speed : %f", this->currentSpeed_);
    }

    void callbackTimer(const ros::TimerEvent& event) {
        buildLaneIndex();
    }

    std::optional<std::string> findNearestLane(float x, float y) {
        if (spatial_cloud_->empty()) {
            return std::nullopt; // 실패: 비어있는 optional 반환
        }

        pcl::PointXY search_point = pcl::PointXY{x, y};
        std::vector<int> point_indices(1);
        std::vector<float> point_distances(1);

        if (kdtree_->nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
            return lane_id_map_[point_indices[0]]; // 성공: 값을 담아 반환
        }
        
        return std::nullopt; // 실패: 비어있는 optional 반환
    }

    std::optional<Lane> getLane(const std::string& current_lane_id) {
        auto it = lanes_.find(current_lane_id);
        if (it != lanes_.end()) {
            return it->second; 
        }
        
        // 실패: 해당 ID의 차선이 없음. 비어있는 optional 반환.
        return std::nullopt;
    }

    void buildLaneIndex() {

        spatial_cloud_->clear();
        lane_id_map_.clear();
        
        // 모든 차선을 순회하며 대표 포인트 좌표와 차선 ID를 바로 가져옴
        for (const auto& pair : lanes_) {
            const Lane& lane = pair.second;
            
            // PointCloud에는 Lane 구조체에 저장된 대표 좌표를 바로 사용
            spatial_cloud_->push_back(pcl::PointXY{lane.x, lane.y});
            // ID 맵에는 차선의 문자열 ID를 저장
            lane_id_map_.push_back(lane.id);
        }

        kdtree_->setInputCloud(spatial_cloud_);
    }

    float calculateDistance(float stoplineX, float stoplineY) {
        return std::sqrt(std::pow(stoplineX - currentX_, 2) + std::pow(stoplineY - currentY_, 2));
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "its_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ItsBridge node(nh, pnh);

    ros::spin();
    return 0;
}
