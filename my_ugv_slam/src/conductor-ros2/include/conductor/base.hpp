#ifndef BASE_HPP
#define BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include "conductor/mission_status.hpp"
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include  <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/odometry.hpp>
class BaseConductor
{
protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr set_raw_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odome_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Mid_pub_;


    void subStateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void subMidCallback(const nav_msgs::msg::Odometry::SharedPtr msg );
    

public:
    BaseConductor(rclcpp::Node::SharedPtr node);
    virtual void initNode();
    rclcpp::Logger get_logger();
    MissionState mission_state;
    geometry_msgs::msg::PoseStamped send_mid_data;
    rclcpp::Time last_request;
    mavros_msgs::msg::State current_state;
    nav_msgs::msg::Odometry receive_data;
    bool setModeGuided(double delay = 5.0);
    bool arm(double delay = 5.0);
    bool takeoff(double altitude, double delay = 3.0);

    bool isTimeElapsed(double delay);
    void updateLastRequestTime();

    rclcpp::Time now();

};

#endif // BASE_HPP