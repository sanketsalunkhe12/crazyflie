#include <memory>
#include <vector>
#include <chrono>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "crazyflie_msgs/srv/takeoff.hpp"
#include "crazyflie_msgs/srv/land.hpp"
#include "crazyflie_msgs/srv/arm.hpp"
#include <crazyflie_msgs/srv/notify_setpoints_stop.hpp>
#include "crazyflie_msgs/msg/full_state.hpp"

#include <Eigen/Geometry>

class TeleopNode : public rclcpp::Node
{
    public:
        TeleopNode();

    private:
        struct 
        {
            float x;
            float y;
            float z;
            float yaw;
        }state_;

        struct Axis
        { 
            int axis;
            float max;
            float deadband;
        };

        struct
        {
            Axis x;
            Axis y;
            Axis z;
            Axis yaw;
        } axes_;

        struct 
        { 
            float duration;
            float height;
            int button;
        } takeoff_paras_;


        float angle_normalize(float a);
        void on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event);
        void publish(); 
        void joyChanged(const sensor_msgs::msg::Joy::SharedPtr msg);
        sensor_msgs::msg::Joy::_axes_type::value_type getAxis(const sensor_msgs::msg::Joy::SharedPtr &msg, Axis a);
        void emergency();
        void arm();
        void takeoff();
        void land();
        void declareAxis(const std::string& name);
        void getAxis(const std::string& name, Axis& axis);
        void on_mode_switched();
        
        
        int land_button;
        int emergency_button;
        int arm_button;

        // monitor parameter changes
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
        
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_emergency_;
        rclcpp::Client<crazyflie_msgs::srv::Arm>::SharedPtr client_arm_;
        rclcpp::Client<crazyflie_msgs::srv::Takeoff>::SharedPtr client_takeoff_;
        rclcpp::Client<crazyflie_msgs::srv::Land>::SharedPtr client_land_;
        rclcpp::Client<crazyflie_msgs::srv::NotifySetpointsStop>::SharedPtr client_notify_setpoints_stop_;
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_legacy_;
        rclcpp::Publisher<crazyflie_msgs::msg::FullState>::SharedPtr pub_cmd_full_state_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_takeoff_;
        
        geometry_msgs::msg::Twist twist_;
        crazyflie_msgs::msg::FullState fullstate_;
        
        std::vector<double> x_limit_;
        std::vector<double> y_limit_;
        std::vector<double> z_limit_;
        
        std::string mode_;
        
        rclcpp::Parameter x_param;
        rclcpp::Parameter y_param;
        rclcpp::Parameter z_param;
        
        int frequency_;
        float dt_;
        bool is_low_level_flight_active_;
        double auto_yaw_rate_;
        bool is_armed_;
};