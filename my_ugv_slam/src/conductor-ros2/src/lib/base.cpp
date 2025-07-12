#include "conductor/base.hpp"
#include "conductor/ansi_color.hpp"


/// @brief
/// @param rate_num loop循环频率
BaseConductor::BaseConductor(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    mission_state = MissionState::kPrearm;
    this->initNode();
}


void BaseConductor::initNode()
{
    setlocale(LC_ALL, "");
    RCLCPP_INFO(this->get_logger(), COLORED_TEXT("Initializing node...", "\033[1m" ANSI_COLOR_MAGENTA));
    RCLCPP_INFO(this->get_logger(), COLORED_TEXT("启动动动动动动动动！！！！！", "\033[2m" ANSI_COLOR_WHITE));
    state_sub_ = this->node_->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10,
        std::bind(&BaseConductor::subStateCallback, this, std::placeholders::_1));

   Odome_sub_= this->node_->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry",10,
        std::bind(&BaseConductor::subMidCallback,this,std::placeholders::_1));
    
    Mid_pub_ = this->node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/vision_pose/pose",10);

    local_pos_pub_ = this->node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    set_raw_pub_ = this->node_->create_publisher<mavros_msgs::msg::PositionTarget>(
        "mavros/setpoint_raw/local", 10);
}

rclcpp::Logger BaseConductor::get_logger()
{
    return this->node_->get_logger();
}

void BaseConductor::subStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    this->current_state = *msg;
}

void BaseConductor::subMidCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this-> receive_data = *msg;
    send_mid_data.pose.position.x=receive_data.pose.pose.position.x;
    send_mid_data.pose.position.y=receive_data.pose.pose.position.y;
    send_mid_data.pose.position.z=0;
    send_mid_data.pose.orientation=receive_data.pose.pose.orientation;
    send_mid_data.header.stamp= this->now();
    Mid_pub_->publish (send_mid_data);
}


/// @brief 设置飞行模式为 guided
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::setModeGuided(double delay)
{
    auto _set_mode_client = this->node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    auto mode_guided_msg = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_guided_msg->custom_mode = "GUIDED";

    if (isTimeElapsed(delay))
    {
        while (!(_set_mode_client->wait_for_service(std::chrono::seconds(1))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service set_mode...");
        }

        auto result = _set_mode_client->async_send_request(mode_guided_msg);
        if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result.get()->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Guided enabled!");
                this->mission_state = MissionState::kArm;
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Guided Switch Failed!");
                this->mission_state = MissionState::kPrearm;
                updateLastRequestTime();
                return false;
            }
        }
    }
    return false;
}


/// @brief 解锁电机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::arm(double delay)
{
    if (!current_state.armed &&
        isTimeElapsed(delay))
    {
        auto arming_client = this->node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_cmd->value = true;

        if (arming_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto result = arming_client->async_send_request(arm_cmd);
            if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result) ==
                    rclcpp::FutureReturnCode::SUCCESS &&
                result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed!");
                this->mission_state = MissionState::kPose;
                RCLCPP_INFO(this->get_logger(), MISSION_SWITCH_TO("kPose"));
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Vehicle arm failed!");
                this->mission_state = MissionState::kPrearm;
                updateLastRequestTime();
                return false;
            }
        }
    }
    return false;
}

/// @brief 起飞到指定高度 ArduCopter
/// @param altitude 高度 单位 M
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::takeoff(double altitude, double delay)
{
    if (isTimeElapsed(delay))
    {
        auto takeoff_client = this->node_->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        auto takeoff_cmd = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_cmd->altitude = altitude;

        if (takeoff_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto result = takeoff_client->async_send_request(takeoff_cmd);
            if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS && result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), SUCCESS("Vehicle Takeoff to altitude: %0.2f"), altitude);
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Vehicle Takeoff Failed!");
                this->mission_state = MissionState::kArm;
                updateLastRequestTime();
                return false;
            }
        }
    }
    return false;
}

/// @brief 判断是否到延迟时间
/// @param delay 延迟时间
/// @return bool
bool BaseConductor::isTimeElapsed(double delay)
{
    return this->now() - this->last_request > rclcpp::Duration::from_seconds(delay);
}

/// @brief 更新 last_request 的值为当前的 ROS 时间
void BaseConductor::updateLastRequestTime()
{
    last_request = this->now();
}

rclcpp::Time BaseConductor::now()
{
    return this->node_->now();
}