#ifndef APM_HPP
#define APM_HPP

#include "conductor/base.hpp"
// #include "conductor/waypoint.hpp"
#include "conductor/ansi_color.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/command_code.hpp>


//小车状态
enum class APMTakeoffState
{
    kArmed,
};

class ArduConductor : public BaseConductor
{
protected:
    void initNode() override;
    APMTakeoffState flag_takeoff;

public:
    ArduConductor(rclcpp::Node::SharedPtr node);
    
    bool arm(double delay = 5);
    void sendGpOrigin(double latitude = 32.108693377508494, double longitude = 118.92943049870283);
    void setSpeedBody(double x, double y, double z, double yaw_rate);
    void setPoseRelated(double x, double y, double z, double yaw);
    void setPoseBody(double x, double y, double z, double yaw);
    void setPoseLocal(double x,double y, double z);
    void setPoseWorld(double x, double y, double z, double yaw) const;
    void sendTranslatedPoseWorld(double x, double y, double z, double yaw) const;
    void setBreak();

    ~ArduConductor(){};
};

#endif // APM_HPP
