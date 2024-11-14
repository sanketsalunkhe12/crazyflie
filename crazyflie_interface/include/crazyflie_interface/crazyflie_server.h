#ifndef CRAZYFLIE_SERVER_H
#define CRAZYFLIE_SERVER_H

#include <memory>
#include <vector>
#include <regex>

#include <crazyflie_cpp/Crazyflie.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "crazyflie_msgs/srv/start_trajectory.hpp"
#include "crazyflie_msgs/srv/takeoff.hpp"
#include "crazyflie_msgs/srv/land.hpp"
#include "crazyflie_msgs/srv/go_to.hpp"
#include "crazyflie_msgs/srv/notify_setpoints_stop.hpp"
#include "crazyflie_msgs/srv/arm.hpp"
#include "crazyflie_msgs/srv/upload_trajectory.hpp"

#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"

#include "crazyflie_msgs/msg/full_state.hpp"
#include "crazyflie_msgs/msg/position.hpp"
#include "crazyflie_msgs/msg/status.hpp"
#include "crazyflie_msgs/msg/log_data_generic.hpp"
#include "crazyflie_msgs/msg/connection_statistics_array.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


// Include necessary headers


// Logging Helper Class which convert Crazyflie-specific log messages into 
// ROS log messages 
class CrazyflieLogger : public Logger
{   
    public:
        CrazyflieLogger(rclcpp::Logger logger, const std::string& prefix)
                        : Logger(), logger_(logger), prefix_(prefix) {}
        
        virtual ~CrazyflieLogger() {}

        virtual void info(const std::string& msg) 
        {RCLCPP_INFO(logger_, "%s %s", prefix_.c_str(), msg.c_str());}

        virtual void warning(const std::string& msg) 
        {RCLCPP_WARN(logger_, "%s %s", prefix_.c_str(),  msg.c_str());}

        virtual void error(const std::string& msg) 
        {RCLCPP_ERROR(logger_, "%s %s", prefix_.c_str(),  msg.c_str());}

    private:
        rclcpp::Logger logger_;
        std::string prefix_;
};

std::set<std::string> extract_names(const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
                            const std::string &pattern)
{
    std::set<std::string> result;
    for (const auto &i : parameter_overrides)
    {
        if (i.first.find(pattern) == 0)
        {
            size_t start = pattern.size() + 1;
            size_t end = i.first.find(".", start);
            result.insert(i.first.substr(start, end - start));
        }
    }
    return result;
}


//  A wrapper for handling a single Crazyflie drone.
class CrazyflieROS
{
    private:
        // struct to represent sensor data from CF (pose, laser scan, status)
        struct logPose 
        {
            float x;
            float y;
            float z;
            int32_t quatCompressed;
        } __attribute__((packed)); // packed attribute to prevent padding and less use of memory


        struct logScan 
        {
            uint16_t front;
            uint16_t left;
            uint16_t back;
            uint16_t right;
        } __attribute__((packed));

        struct logStatus 
        {
            // general status
            uint16_t supervisorInfo; // supervisor.info
            // battery related
            // Note that using BQ-deck/Bolt one can actually have two batteries at the same time.
            // vbat refers to the battery directly connected to the CF board and might not reflect
            // the "external" battery on BQ/Bolt builds
            uint16_t vbatMV;  // pm.vbatMV
            uint8_t pmState;  // pm.state
            // radio related
            uint8_t rssi;     // radio.rssi
            uint16_t numRxBc; // radio.numRxBc
            uint16_t numRxUc; // radio.numRxUc
        } __attribute__((packed));

    public:
        CrazyflieROS(const std::string& link_uri, const std::string& cf_type,
                     const std::string& name, rclcpp::Node* node,
                     rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd, 
                     rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv,
                     const CrazyflieBroadcaster* cfbc, bool enable_parameters = true);
                           
        // ~CrazyflieROS();

        void spin_once();
        std::string broadcastUri() const;
        uint8_t id() const;
        const Crazyflie::ParamTocEntry* paramTocEntry(const std::string& group, const std::string& name);
        const std::string& name() const;
        void change_parameter(const rclcpp::Parameter& p);

    private:
        void cmd_full_state_changed(const crazyflie_msgs::msg::FullState::SharedPtr msg);
        void cmd_position_changed(const crazyflie_msgs::msg::Position::SharedPtr msg); 
        void cmd_vel_legacy_changed(const geometry_msgs::msg::Twist::SharedPtr msg);

        void on_console(const char *msg);
        void emergency(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void arm(const std::shared_ptr<crazyflie_msgs::srv::Arm::Request> request,
                 std::shared_ptr<crazyflie_msgs::srv::Arm::Response> response);
        void takeoff(const std::shared_ptr<crazyflie_msgs::srv::Takeoff::Request> request,
                    std::shared_ptr<crazyflie_msgs::srv::Takeoff::Response> response);
        void land(const std::shared_ptr<crazyflie_msgs::srv::Land::Request> request,
                    std::shared_ptr<crazyflie_msgs::srv::Land::Response> response);
        void go_to(const std::shared_ptr<crazyflie_msgs::srv::GoTo::Request> request,
                    std::shared_ptr<crazyflie_msgs::srv::GoTo::Response> response);

        void upload_trajectory(const std::shared_ptr<crazyflie_msgs::srv::UploadTrajectory::Request> request,
                                std::shared_ptr<crazyflie_msgs::srv::UploadTrajectory::Response> response);
        void start_trajectory(const std::shared_ptr<crazyflie_msgs::srv::StartTrajectory::Request> request,
                                std::shared_ptr<crazyflie_msgs::srv::StartTrajectory::Response> response);
        void notify_setpoints_stop(const std::shared_ptr<crazyflie_msgs::srv::NotifySetpointsStop::Request> request,
                                   std::shared_ptr<crazyflie_msgs::srv::NotifySetpointsStop::Response> response);

        void on_logging_pose(uint32_t time_in_ms, const logPose* data);
        void on_logging_scan(uint32_t time_in_ms, const logScan* data);
        void on_logging_status(uint32_t time_in_ms, const logStatus* data);
        void on_logging_custom(uint32_t time_in_ms, const std::vector<float>* values, 
                                void* userData); 
        void on_link_statistics_timer();
        void on_latency(uint64_t latency_in_us);

    private:
        rclcpp::Logger logger_;
        CrazyflieLogger cf_logger_;

        Crazyflie cf_;
        std::string message_buffer_;
        std::string name_;  

        rclcpp::Node* node_;
        tf2_ros::TransformBroadcaster tf_broadcaster_; 

        // ROS2 Services for Crazyflie commands
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_emergency_;
        rclcpp::Service<crazyflie_msgs::srv::StartTrajectory>::SharedPtr service_start_trajectory_;
        rclcpp::Service<crazyflie_msgs::srv::Takeoff>::SharedPtr service_takeoff_;
        rclcpp::Service<crazyflie_msgs::srv::Land>::SharedPtr service_land_;
        rclcpp::Service<crazyflie_msgs::srv::GoTo>::SharedPtr service_go_to_;
        rclcpp::Service<crazyflie_msgs::srv::UploadTrajectory>::SharedPtr service_upload_trajectory_;
        rclcpp::Service<crazyflie_msgs::srv::NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
        rclcpp::Service<crazyflie_msgs::srv::Arm>::SharedPtr service_arm_;

        // ROS2 Subscriber for Crazyflie data
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_legacy_;
        rclcpp::Subscription<crazyflie_msgs::msg::FullState>::SharedPtr subscription_cmd_full_state_;
        rclcpp::Subscription<crazyflie_msgs::msg::Position>::SharedPtr subscription_cmd_position_;

        // ROS2 Publishers for Crazyflie data
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_robot_description_;

        std::unique_ptr<LogBlock<logPose>> log_block_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

        std::unique_ptr<LogBlock<logScan>> log_block_scan_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan_;

        std::unique_ptr<LogBlock<logStatus>> log_block_status_;
        bool status_has_radio_stats_;
        rclcpp::Publisher<crazyflie_msgs::msg::Status>::SharedPtr publisher_status_;

        uint16_t previous_numRxBc;
        uint16_t previous_numRxUc;
        bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_unicast_;
        bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_broadcast_;
        const CrazyflieBroadcaster* cfbc_;

        std::list<std::unique_ptr<LogBlockGeneric>> log_blocks_generic_;
        std::list<rclcpp::Publisher<crazyflie_msgs::msg::LogDataGeneric>::SharedPtr> publishers_generic_;

        // multithreading
        rclcpp::CallbackGroup::SharedPtr callback_group_cf_;
        rclcpp::TimerBase::SharedPtr spin_timer_;

        // link statistics
        rclcpp::TimerBase::SharedPtr link_statistics_timer_;
        std::chrono::time_point<std::chrono::steady_clock> last_on_latency_;
        uint16_t last_latency_in_ms_;
        float warning_freq_;
        float max_latency_;
        float min_ack_rate_;
        float min_unicast_receive_rate_;
        float min_broadcast_receive_rate_;
        bool publish_stats_;
        rclcpp::Publisher<crazyflie_msgs::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;
};


//  A server node that handles multiple Crazyflie instances.
class CrazyflieServer : public rclcpp::Node
{
    public:
        CrazyflieServer();
        ~CrazyflieServer();

    private:

        rclcpp::Logger logger_;

        // subscribers
        rclcpp::Subscription<crazyflie_msgs::msg::FullState>::SharedPtr subscription_cmd_full_state_;
        rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr sub_poses_;

        // services
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_emergency_;
        rclcpp::Service<crazyflie_msgs::srv::StartTrajectory>::SharedPtr service_start_trajectory_;
        rclcpp::Service<crazyflie_msgs::srv::Takeoff>::SharedPtr service_takeoff_;
        rclcpp::Service<crazyflie_msgs::srv::Land>::SharedPtr service_land_;
        rclcpp::Service<crazyflie_msgs::srv::GoTo>::SharedPtr service_go_to_;
        rclcpp::Service<crazyflie_msgs::srv::NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
        rclcpp::Service<crazyflie_msgs::srv::Arm>::SharedPtr service_arm_;

// IMP list of drone and its pointer to ROS2 instance
        std::map<std::string, std::unique_ptr<CrazyflieROS>> crazyflies_;
        
        // broadcastUri -> broadcast object
        std::map<std::string, std::unique_ptr<CrazyflieBroadcaster>> broadcaster_;
        std::map<std::string, uint8_t> name_to_id_;     // maps CF name -> CF id

        // global params
        int broadcasts_num_repeats_; // repeat broadcasts for reliability
        int broadcasts_delay_between_repeats_ms_; // prevent rapid repeated broadcasting

        // parameter updates
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;

        // sanity checks
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
        bool mocap_enabled_;
        float mocap_min_rate_;
        float mocap_max_rate_;
        std::vector<std::chrono::time_point<std::chrono::steady_clock>> mocap_data_received_timepoints_;
        bool publish_stats_;
        rclcpp::Publisher<crazyflie_msgs::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;
        
        // multithreading
        rclcpp::CallbackGroup::SharedPtr callback_group_mocap_;
        rclcpp::CallbackGroup::SharedPtr callback_group_all_cmd_;
        rclcpp::CallbackGroup::SharedPtr callback_group_all_srv_;
        rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd_;
        rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv_;


        // functions
        void emergency(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);
        void start_trajectory(const std::shared_ptr<crazyflie_msgs::srv::StartTrajectory::Request> request,
                            std::shared_ptr<crazyflie_msgs::srv::StartTrajectory::Response> response);
        void takeoff(const std::shared_ptr<crazyflie_msgs::srv::Takeoff::Request> request,
                        std::shared_ptr<crazyflie_msgs::srv::Takeoff::Response> response);
        void land(const std::shared_ptr<crazyflie_msgs::srv::Land::Request> request,
                    std::shared_ptr<crazyflie_msgs::srv::Land::Response> response);
        void go_to(const std::shared_ptr<crazyflie_msgs::srv::GoTo::Request> request,
                    std::shared_ptr<crazyflie_msgs::srv::GoTo::Response> response);
        void notify_setpoints_stop(const std::shared_ptr<crazyflie_msgs::srv::NotifySetpointsStop::Request> request,
                                    std::shared_ptr<crazyflie_msgs::srv::NotifySetpointsStop::Response> response);
        void arm(const std::shared_ptr<crazyflie_msgs::srv::Arm::Request> request,
                std::shared_ptr<crazyflie_msgs::srv::Arm::Response> response);
        
        void cmd_full_state_changed(const crazyflie_msgs::msg::FullState::SharedPtr msg);
        void posesChanged(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg);
        void on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event);
        void on_watchdog_timer();
        template<class T> void broadcast_set_param(const std::string& group, const std::string& name,
                                                    const T& value);
        void update_name_to_id_map(const std::string& name, uint8_t id);


};

// Declare your class or struct

#endif // CRAZYFLIE_SERVER_H