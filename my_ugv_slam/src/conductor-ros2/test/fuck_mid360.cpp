#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class Mid : public rclcpp::Node {
public:
    Mid() : Node("zzjnp") {
        // 创建 TF Buffer 与 Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ✅ 修复：直接传递节点引用 *this
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 创建定时器
        timer_ = create_wall_timer(100ms, std::bind(&Mid::on_timer, this));

        // 创建话题发布器
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        pub_x_ = create_publisher<std_msgs::msg::Float64>("/pose_mid_x", 10);
        pub_y_ = create_publisher<std_msgs::msg::Float64>("/pose_mid_y", 10);
        pub_z_ = create_publisher<std_msgs::msg::Float64>("/pose_mid_z", 10);
        pub_yaw_ = create_publisher<std_msgs::msg::Float64>("/pose_mid_yaw", 10);
        pub_pos_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose",10);
        pub_path_ = create_publisher<nav_msgs::msg::Path>("/path",10);
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pos_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_x_, pub_y_, pub_z_, pub_yaw_;
    rclcpp::TimerBase::SharedPtr timer_;

    

    void on_timer() {
        try {
            // ✅ 修正坐标系名称（删除空格）
            auto tf = tf_buffer_->lookupTransform(
                "camera_init",  // 目标坐标系
                "body",        // 源坐标系
                tf2::TimePointZero);
            nav_msgs::msg::Path path;
            geometry_msgs::msg::PoseStamped pose_stamped;
            double yaw_;
            tf2::Quaternion leveled_quat;
            leveled_quat.setRPY(0, 0, yaw_);  // 将横滚和俯仰设为0
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.y = tf.transform.translation.y;
            pose_stamped.pose.position.x = tf.transform.translation.x;
            pose_stamped.pose.position.z =tf.transform.translation.z;
            pose_stamped.pose.orientation.x = leveled_quat.x();
            pose_stamped.pose.orientation.y = leveled_quat.y();
            pose_stamped.pose.orientation.z = leveled_quat.z();
            pose_stamped.pose.orientation.w = leveled_quat.w();

            path.poses.push_back(pose_stamped);
            path.header.stamp = this->now();
            path.header.frame_id = "map";
            pub_pos_->publish(pose_stamped);
            pub_path_->publish(path);
            
            // 发布里程计            
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = tf.transform.translation.x;
            odom.pose.pose.position.y = tf.transform.translation.y;
            odom.pose.pose.position.z = tf.transform.translation.z;
            odom.pose.pose.orientation = tf.transform.rotation;
            odom_pub_->publish(odom);

            // 广播 TF
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header = odom.header;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base_link";
            odom_tf.transform = tf.transform;
            tf_broadcaster_->sendTransform(odom_tf);

            // 转为欧拉角
            tf2::Quaternion q(
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 发布 Float64
            std_msgs::msg::Float64 msg_x, msg_y, msg_z, msg_yaw;
            msg_x.data = tf.transform.translation.x;
            msg_y.data = tf.transform.translation.y;
            msg_z.data = tf.transform.translation.z;
            msg_yaw.data = yaw;

            pub_x_->publish(msg_x);
            pub_y_->publish(msg_y);
            pub_z_->publish(msg_z);
            pub_yaw_->publish(msg_yaw);

            RCLCPP_INFO(get_logger(), "x=%.2f y=%.2f z=%.2f yaw=%.2f", 
                        msg_x.data * 100, msg_y.data * 100, msg_z.data * 100, msg_yaw.data);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(),
                        "TF transform failed (camera_init to body): %s", ex.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mid>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}