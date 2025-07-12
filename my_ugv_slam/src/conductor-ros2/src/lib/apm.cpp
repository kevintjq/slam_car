#include "conductor/apm.hpp"



ArduConductor::ArduConductor(rclcpp::Node::SharedPtr node)
	: BaseConductor(node), flag_takeoff(APMTakeoffState::kArmed)
{
	initNode();
}


/// @brief 设置全局坐标原点
/// @param latitude 纬度
/// @param longitude 经度
void ArduConductor::sendGpOrigin(double latitude, double longitude)
{
	// 临时创建一个发布者对象
	auto set_gp_origin_pub_ = this->node_->create_publisher<geographic_msgs::msg::GeoPointStamped>(
		"mavros/global_position/set_gp_origin", 10);
	// 构建 GeoPointStamped 消息
	auto gp_origin = geographic_msgs::msg::GeoPointStamped();
	gp_origin.position.latitude = latitude;
	gp_origin.position.longitude = longitude;
	gp_origin.position.altitude = 0;
	gp_origin.header.stamp = this->now();

	set_gp_origin_pub_->publish(gp_origin);
	RCLCPP_INFO(this->get_logger(), SUCCESS("Setting gp origin...(%0.2f, %0.2f)"), longitude, latitude);
}



/// @brief flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
/// @param x
/// @param y
/// @param z
/// @param yaw_rate
/// @return
void ArduConductor::setSpeedBody(double x, double y, double z, double yaw_rate)
{
	auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
						   mavros_msgs::msg::PositionTarget::IGNORE_PY |
						   mavros_msgs::msg::PositionTarget::IGNORE_PZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
	if (fabs(yaw_rate) < 1e-3)
	{
		raw_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
		raw_target.yaw = 0;
	}
	else
	{
		raw_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
	}

	raw_target.velocity.x = x;
	raw_target.velocity.y = y;
	raw_target.velocity.z = z;
	raw_target.yaw_rate = yaw_rate;
	set_raw_pub_->publish(raw_target);
}


/// @brief 解锁电机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool ArduConductor::arm(double delay)
{
	if (!current_state.armed &&
		isTimeElapsed(delay))
		RCLCPP_ERROR(this->get_logger(), "Vehicle arm failed!");
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

/// @brief
/// @param x
/// @param y
/// @param z
/// @param yaw
/// @return
void ArduConductor::setPoseLocal(double x,double y,double z)
{
	geometry_msgs::msg::PoseStamped pose_target;
	pose_target.pose.position.x = x;
	pose_target.pose.position.y = y;
	pose_target.pose.position.z = z;

	pose_target.pose.orientation.w = 1;
	pose_target.pose.orientation.x = 0;
	pose_target.pose.orientation.y = 0;
	pose_target.pose.orientation.z = 0;
	local_pos_pub_ ->publish(pose_target);
} 






/// @brief flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
/// @param x
/// @param y
/// @param z
/// @param yaw
/// @return
void ArduConductor::setPoseBody(double x, double y, double z, double yaw)
{
    auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
						   mavros_msgs::msg::PositionTarget::IGNORE_VY |
						   mavros_msgs::msg::PositionTarget::IGNORE_VZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = x;
	raw_target.position.y = y;
	raw_target.position.z = z;
	raw_target.yaw = yaw;
	set_raw_pub_->publish(raw_target);
}


void ArduConductor::setPoseWorld(double x, double y, double z, double yaw) const
{
	sendTranslatedPoseWorld(x, y, z, yaw);
}



void ArduConductor::sendTranslatedPoseWorld(double x, double y, double z, double yaw) const
{
	
	auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
						   mavros_msgs::msg::PositionTarget::IGNORE_VY |
						   mavros_msgs::msg::PositionTarget::IGNORE_VZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = -y;
	raw_target.position.y = x;
	raw_target.position.z = z;
	raw_target.yaw = yaw ;

	// 使用 ROS2 语法发布消息
	set_raw_pub_->publish(raw_target);
}





void ArduConductor::initNode()
{
}

void ArduConductor::setPoseRelated(double x, double y, double z, double yaw)
{
	(void)x; (void)y; (void)z; (void)yaw;
}




void ArduConductor::setBreak()
{
	setSpeedBody(0.0, 0.0, 0.0, 0.0);
}