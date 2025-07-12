#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <string>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>

class ControlCarNode : public rclcpp::Node
{
public:
    ControlCarNode() : Node("control_car_node"), fd_control(-1)
    {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB_A");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("wheel_base", 0.2);  // 轮距 (m)
        this->declare_parameter<double>("wheel_radius", 0.0325);  // 轮半径 (m)
        this->declare_parameter<double>("max_wheel_speed", 10);  // 最大轮速 (rad/s)
        this->declare_parameter<int>("max_speed_value", 25);  // 协议中最大速度值(0-99)
        this->declare_parameter<bool>("debug_mode", false);
        
        // 获取参数
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();
        max_speed_value_ = this->get_parameter("max_speed_value").as_int();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();

        // 初始化串口
        if (!uart_control_init(serial_port, baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize control serial port");
        } else {
            RCLCPP_INFO(this->get_logger(), "Control serial port initialized successfully");
        }

        // 订阅 /cmd_vel 话题
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ControlCarNode::cmd_vel_callback, this, std::placeholders::_1));

        // 创建超时检查定时器
        timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlCarNode::timeout_check_callback, this));

        last_cmd_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "控制小车节点初始化成功");
        RCLCPP_INFO(this->get_logger(), "轮距: %.3f m, 轮半径: %.3f m", wheel_base_, wheel_radius_);
    }

    ~ControlCarNode()
    {
        if (fd_control != -1) {
            sendStopCommand();
            close(fd_control);
            RCLCPP_INFO(this->get_logger(), "Control serial port closed");
        }
    }

private:
    int setup_serial(int fd, int baud_rate) {
        struct termios options;
         
        // 获取当前配置
        if (tcgetattr(fd, &options) == -1) {
            return -1;
        }
         
        // 设置波特率
        speed_t speed;
        switch(baud_rate) {
            case 9600:   speed = B9600;   break;
            case 19200:  speed = B19200;  break;
            case 38400:  speed = B38400;  break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
            default:     
                RCLCPP_WARN(this->get_logger(), "Unsupported baud rate: %d, using 115200", baud_rate);
                speed = B115200;
                break;
        }
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);
         
        // 设置数据位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
         
        // 设置停止位
        options.c_cflag &= ~CSTOPB;  // 1个停止位
         
        // 设置奇偶校验
        options.c_cflag &= ~PARENB;  // 无校验
        options.c_iflag &= ~INPCK;
         
        // 禁用硬件流控
        options.c_cflag &= ~CRTSCTS;
        
        // 禁用软件流控
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        // 设置为原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
        options.c_oflag &= ~OPOST;
        
        // 其他重要标志
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        // 设置读取超时
        options.c_cc[VTIME] = 0;  // 非规范模式读取时的超时时间
        options.c_cc[VMIN] = 0;   // 非规范模式读取时的最小字符数
         
        // 清空串口缓冲
        tcflush(fd, TCIOFLUSH);
        
        // 立即应用设置
        if (tcsetattr(fd, TCSANOW, &options) == -1) {
            return -1;
        }
        
        return 0;
    }

    bool uart_control_init(const std::string& port, int baud_rate) {
        // 打开控制串口
        fd_control = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_control == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error opening control serial port: %s", port.c_str());
            return false;
        }
         
        // 设置串口
        if (setup_serial(fd_control, baud_rate) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error setting up control serial port");
            close(fd_control);
            fd_control = -1;
            return false;
        }

        // 清空可能存在的数据
        tcflush(fd_control, TCIOFLUSH);
        
        return true;
    }

    bool send_control_data(const uint8_t *data, size_t size) {
        if (fd_control == -1) {
            RCLCPP_ERROR(this->get_logger(), "Control serial port not initialized");
            return false;
        }
        
        // 一个字符一个字符发送
        for(size_t i = 0; i < size; i++) {
            int written_len = write(fd_control, &data[i], 1);
            if (written_len != 1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to control serial port");
                return false;
            }
            // 每个字符之间添加小延时
            usleep(1000);  // 1ms延时
        }
        
        // 等待数据发送完成
        tcdrain(fd_control);
        
        // 始终打印发送信息，不管是否在debug模式
        RCLCPP_INFO(this->get_logger(), "发送数据成功: 写入 %d 字节, 数据: %.*s", 
                    (int)size, (int)size, data);
        return true;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 更新最后命令时间
        last_cmd_time_ = this->get_clock()->now();
        
        // 获取并限制线速度和角速度
        double linear_vel = std::max(-0.2, std::min(0.2, msg->linear.x));   // 限制线速度范围为 ±0.2 m/s
        double angular_vel = std::max(-1.5, std::min(1.5, msg->angular.z)); // 限制角速度范围为 ±1.5 rad/s
        
        // 差分运动学解算：从线速度和角速度计算左右轮速度
        double left_wheel_speed = (linear_vel - angular_vel * wheel_base_ / 2.0) / wheel_radius_;   // rad/s
        double right_wheel_speed = (linear_vel + angular_vel * wheel_base_ / 2.0) / wheel_radius_;  // rad/s
        
        // 限制轮子速度范围
        left_wheel_speed = std::max(-max_wheel_speed_, std::min(max_wheel_speed_, left_wheel_speed));
        right_wheel_speed = std::max(-max_wheel_speed_, std::min(max_wheel_speed_, right_wheel_speed));
        
        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), 
                "输入 - 线速度: %.3f m/s, 角速度: %.3f rad/s", 
                linear_vel, angular_vel);
            RCLCPP_INFO(this->get_logger(), 
                "轮速 - 左轮: %.3f rad/s, 右轮: %.3f rad/s", 
                left_wheel_speed, right_wheel_speed);
        }
        
        // 将轮速转换为控制命令
        int left_direction = (left_wheel_speed < 0) ? 1 : 0;
        int right_direction = (right_wheel_speed < 0) ? 1 : 0;
        
        // 计算速度值（0-25）
        int left_speed = static_cast<int>(std::abs(left_wheel_speed) / max_wheel_speed_ * 25.0);
        int right_speed = static_cast<int>(std::abs(right_wheel_speed) / max_wheel_speed_ * 25.0);
        
        // 限制速度范围
        left_speed = std::max(0, std::min(25, left_speed));
        right_speed = std::max(0, std::min(25, right_speed));
        
        // 发送控制命令
        send_wheel_control(left_direction, left_speed, right_direction, right_speed);
    }

    // 发送新的控制协议命令（8位数字格式）
    bool send_wheel_control(int left_direction, int left_speed, int right_direction, int right_speed) {
        // 限制速度范围 (0-25)
        left_speed = std::max(0, std::min(25, left_speed));
        right_speed = std::max(0, std::min(25, right_speed));
        
        // 构造固定8位控制命令字符串
        char command[9]; // 8位数字 + 结束符
        
        // 左轮：01=正转，02=反转
        if (left_direction == 0) {
            command[0] = '0';
            command[1] = '1';
        } else {
            command[0] = '0';
            command[1] = '2';
        }
        
        // 左轮速度
        command[2] = '0' + (left_speed / 10);
        command[3] = '0' + (left_speed % 10);
        
        // 右轮：01=正转，02=反转
        if (right_direction == 0) {
            command[4] = '0';
            command[5] = '1';
        } else {
            command[4] = '0';
            command[5] = '2';
        }
        
        // 右轮速度
        command[6] = '0' + (right_speed / 10);
        command[7] = '0' + (right_speed % 10);
        command[8] = '\0'; // 字符串结束符
        
        bool result = send_control_data(reinterpret_cast<const uint8_t*>(command), 8);
        if (result) {
            RCLCPP_INFO(this->get_logger(), 
                       "准备发送控制命令 - 左轮: %s转速度%02d, 右轮: %s转速度%02d -> 命令: %s", 
                       (left_direction == 0) ? "正" : "反", left_speed,
                       (right_direction == 0) ? "正" : "反", right_speed, command);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send wheel control command");
        }
        return result;
    }

    void timeout_check_callback()
    {
        auto now = this->get_clock()->now();
        auto time_since_last_cmd = (now - last_cmd_time_).seconds();
        
        // 2秒无命令则发送停止命令
        if (time_since_last_cmd > 2.0)
        {
            sendStopCommand();
        }
    }

    void sendStopCommand()
    {
        // 发送停止命令: 左右轮都停止
        char stop_command[9] = "00000000";
        send_control_data(reinterpret_cast<const uint8_t*>(stop_command), 8);
        
        if (debug_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "发送停止命令: %s", stop_command);
        }
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    int fd_control;  // 控制串口文件描述符
    
    double wheel_base_;        // 轮距 (m)
    double wheel_radius_;      // 轮半径 (m)
    double max_wheel_speed_;   // 最大轮速 (rad/s)
    int max_speed_value_;      // 协议中最大速度值
    bool debug_mode_;          // 调试模式
    
    rclcpp::Time last_cmd_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<ControlCarNode>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "节点启动失败: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
