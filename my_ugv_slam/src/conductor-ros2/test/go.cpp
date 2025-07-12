
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>


#include "conductor/mission_status.hpp"
#include "conductor/ansi_color.hpp"
#include "conductor/apm.hpp"


#include "signal.h" //necessary for the Custom SIGINT handler
#include "stdio.h"  //necessary for the Custom SIGINT handlers
struct Pose_
{
    double x;
    double y;
    double z;
    double yaw;
}pose_mid;


void signal_handler(int signal) {
    if (signal == SIGINT) {
        // 处理退出逻辑
        std::cout << "Caught Ctrl+C, exiting..." << std::endl;
        rclcpp::shutdown(); 
        exit(0); // 强制退出程序
    }
}

class Mid_back : public  rclcpp :: Node
{
    public:
    Mid_back()
    :Node("zzj_np")
    {
        subscriptionx_ = this->create_subscription<std_msgs::msg::Float64>("pose_mid_x", 10,std::bind(&Mid_back::topic_callback1, this,std::placeholders::_1));
        subscriptiony_ = this->create_subscription<std_msgs::msg::Float64>("pose_mid_y", 10,std::bind(&Mid_back::topic_callback2, this,std::placeholders::_1));
        subscriptionz_ = this->create_subscription<std_msgs::msg::Float64>("pose_mid_z", 10,std::bind(&Mid_back::topic_callback3, this,std::placeholders::_1));
    }
    private:
    void topic_callback1(const std_msgs::msg::Float64 & msg) const
  {
    pose_mid.x = msg.data;
  }
     void topic_callback2(const std_msgs::msg::Float64 & msg) const
  {
    pose_mid.y = msg.data;
  }
    void topic_callback3(const std_msgs::msg::Float64 & msg) const
  {
    pose_mid.z = msg.data;
  }
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriptionx_,subscriptiony_,subscriptionz_;
    
};
 

bool pose_status(double x,double y){
        int i=0;
    if((fabs(fabs(x+y)-fabs(pose_mid.x+pose_mid.y))<0.5)||i>=10){
        return true;
    }
    else{
        printf("%lf,%lf\n",pose_mid.x,pose_mid.y);
        sleep(1);
        i++;
        return false;
    }
}


int main(int argc, char **argv)
{
     
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<rclcpp::Node>("conductor_node", options);
    ArduConductor apm(node);
    apm.last_request =apm.now();
    auto fuck_mid = std::make_shared<Mid_back>();
    executor.add_node(fuck_mid);
    
    while (rclcpp::ok())
{
    executor.spin_some();  // 处理 Mid_back 的回调函数

    // 任务执行状态机
    switch (apm.mission_state)
    {
        case MissionState::kPrearm:
            if (apm.setModeGuided(5.0)) // 修改飞行模式为 Guided (ArduCopter)
            {
                apm.sendGpOrigin(); // 如果切换成Guided模式就发送全局原点坐标
            }
            break;

        case MissionState::kArm:
            apm.arm(5.0);  //解锁电机
            break;

        case MissionState::kPose:
         apm.setPoseLocal(1,0,pose_mid.z);
            sleep(50);
            apm.setPoseBody(0,0,pose_mid.z, 1.57);
           
            sleep(10);
            //apm.setPoseLocal(1,2,pose_mid.z);
            //sleep(10);
            //apm.setPoseLocal(0,2,pose_mid.z);
            //sleep(10);
            //apm.setPoseLocal(0,0,pose_mid.z);
            sleep(40);
            break;

        case MissionState::kWaybackHome:
            apm.setSpeedBody(0,0,0,0);
            break;

        default:
            executor.spin_some();
            break;
    }

    rclcpp::spin_some(node);  // 处理 conductor_node 的回调函数
}
   
   rclcpp::shutdown();
    return 0;
}




