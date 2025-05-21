#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread> 
#include <chrono> 
#include "sensor_msgs/msg/laser_scan.hpp"

#define SOCKET_PATH "/tmp/f_value_socket"

class FReceiverNode : public rclcpp::Node {
public:
    FReceiverNode() : Node("f_receiver_node"), 
                      current_linear_x_(0.0), 
                      current_angular_z_(0.0),
                      consecutive_forward_presses_(0),    
                      executing_square_pattern_(false), 
                      square_pattern_step_(0),          
                      obstacle_detected_(false),          
                      autonomous_avoidance_mode_(false) // 默认不进入自主避障模式
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        setup_socket();
        // 主速度发布定时器
        main_velocity_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FReceiverNode::publish_cmd_vel, this));
        
        last_forward_press_time_ = this->now(); // 初始化上次按键时间
        // 创建雷达数据订阅者
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::SensorDataQoS(),
            std::bind(&FReceiverNode::lidar_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "FReceiverNode initialized. Subscribing to lidar data.");
    }

    ~FReceiverNode() {
        if (socket_thread_.joinable()) { // 确保线程可以汇合
            // socket_thread_.join(); 
        }
        close(sockfd_);
        unlink(SOCKET_PATH);
    }

private:
    void setup_socket() {
        sockfd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket error");
            rclcpp::shutdown();
            return; 
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

        unlink(SOCKET_PATH); // 尝试删除已存在的socket文件
        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
            perror("bind error");
            rclcpp::shutdown();
            return; 
        }

        // 创建一个线程来监听套接字
        socket_thread_ = std::thread(&FReceiverNode::check_socket, this);
        socket_thread_.detach(); 
    }

    void check_socket() {
        int f;
        struct sockaddr_un remote_addr; // 用于recvfrom的对方地址
        socklen_t addr_len = sizeof(remote_addr);

        while (rclcpp::ok()) {
            // 接收 f 的值
            int n = recvfrom(sockfd_, &f, sizeof(f), 0, (struct sockaddr*)&remote_addr, &addr_len);
            if (n > 0) {
                RCLCPP_INFO(this->get_logger(), "Received f value: %d", f);
                update_velocity(f); 
            } else if (n == -1) {
                if (errno != EWOULDBLOCK && errno != EAGAIN) {
                    // perror("recvfrom error in check_socket"); // 可能会频繁打印（此处注释）
                }
            } 
        }
    }
         
   void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
       obstacle_detected_ = false; 
       float min_front_distance = msg->range_max;

    // 计算正前方中心点索引
      size_t center_index = static_cast<size_t>(std::round((0.0 - msg->angle_min) / msg->angle_increment));

    // 定义前方检测扇区的半宽度
      size_t scan_half_width_indices = 20; 

      size_t start_index = 0;
      if (center_index > scan_half_width_indices) { // 防止索引为负
          start_index = center_index - scan_half_width_indices;
    }

      size_t end_index = std::min(msg->ranges.size() - 1, center_index + scan_half_width_indices);

      float obstacle_threshold_distance = 0.25; // 此处定义小于0.25米算障碍物（可根据演示需要进行更改）

      RCLCPP_DEBUG(this->get_logger(), "Lidar scan: center_idx=%zu, start_idx=%zu, end_idx=%zu, total_ranges=%zu", 
                   center_index, start_index, end_index, msg->ranges.size());

      for (size_t i = start_index; i <= end_index; ++i) {
        // 确保索引在有效范围内
          if (i < msg->ranges.size()) { 
            // 检查距离值是否有效且小于阈值
              if (msg->ranges[i] > msg->range_min && msg->ranges[i] < obstacle_threshold_distance) {
                  obstacle_detected_ = true;
                  min_front_distance = std::min(min_front_distance, msg->ranges[i]);
                // break; // 一旦检测到就可以停止检查这个扇区，或者继续找最近的
            }
        }
    }

      if (obstacle_detected_) {
          RCLCPP_WARN(this->get_logger(), "Obstacle detected in front at %.2f meters!", min_front_distance);
      }

      if (autonomous_avoidance_mode_ && obstacle_detected_ && !executing_square_pattern_) {
           RCLCPP_INFO(this->get_logger(), "Autonomous mode: Obstacle detected, initiating avoidance maneuver.");
           initiate_avoidance_maneuver();
      }
}
    
    
    void initiate_avoidance_maneuver() {
    // 确保不在执行其他自动序列
    if (executing_square_pattern_ || (avoidance_action_timer_ && !avoidance_action_timer_->is_canceled())) {
        RCLCPP_INFO(this->get_logger(), "Already in an action sequence, not starting new avoidance.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Avoidance: Stopping and preparing to rotate.");
    // 立即停止
    current_linear_x_ = 0.0;
    current_angular_z_ = 0.0;
    publish_cmd_vel(); // 立即发布停止指令

    // 2. 短暂停顿后开始旋转
    const auto stop_duration = std::chrono::milliseconds(500);
    const auto rotation_action_duration = std::chrono::seconds(2); 
    const float rotation_action_speed = 0.5;

    // 定时器1: 用于实现初始的 stop_duration 延时
    avoidance_action_timer_ = this->create_wall_timer(
        stop_duration,
        [this, rotation_action_duration, rotation_action_speed]() { 
            if (this->avoidance_action_timer_ && !this->avoidance_action_timer_->is_canceled()) {
                this->avoidance_action_timer_->cancel();
            }

            RCLCPP_INFO(this->get_logger(), "Avoidance: Rotating.");
            this->current_linear_x_ = 0.0;
            this->current_angular_z_ = rotation_action_speed; // 开始旋转
            this->publish_cmd_vel();

            // 定时器2: 用于控制 rotation_action_duration 的旋转时长
            this->avoidance_action_timer_ = this->create_wall_timer(
                rotation_action_duration, 
                
                [this]() {
                    if (this->avoidance_action_timer_ && !this->avoidance_action_timer_->is_canceled()) {
                        this->avoidance_action_timer_->cancel();
                    }

                    RCLCPP_INFO(this->get_logger(), "Avoidance: Rotation finished, stopping.");
                    this->current_linear_x_ = 0.0;
                    this->current_angular_z_ = 0.0;
                    this->publish_cmd_vel();

                    // 避障动作完成，如果自主模式仍然开启，则尝试恢复缓慢前进
                    if (this->autonomous_avoidance_mode_) {
                        RCLCPP_INFO(this->get_logger(), "Avoidance maneuver complete. Resuming slow forward movement.");
                        this->current_linear_x_ = 0.2; 
                        this->current_angular_z_ = 0.0;
                       
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Avoidance maneuver complete, but autonomous mode was disabled.");
                        // 保持停止状态
                    }
                } 
            ); 
        } 
    ); 
} 
    
    
    
    
    void update_velocity(int f) {
        // 1. 画正方形模式优先
        if (executing_square_pattern_) {
            RCLCPP_INFO(this->get_logger(), "Currently executing square pattern, ignoring f_value: %d", f);
            return; 
        }

        // 2. 处理模式切换指令 f=10 (来自App的“障碍检测”开关)
        if (f == 10) { 
            autonomous_avoidance_mode_ = !autonomous_avoidance_mode_; 
            if (autonomous_avoidance_mode_) {
                RCLCPP_INFO(this->get_logger(), "Autonomous avoidance mode ENABLED by f=10. Starting slow forward movement.");
                current_linear_x_ = 0.2; // 自主前进速度（可根据演示需要进行更改）
                current_angular_z_ = 0.0;
            } else {
                RCLCPP_INFO(this->get_logger(), "Autonomous avoidance mode DISABLED by f=10. Stopping any action.");
                current_linear_x_ = 0.0; 
                current_angular_z_ = 0.0;
                if (avoidance_action_timer_ && !avoidance_action_timer_->is_canceled()) {
                    avoidance_action_timer_->cancel(); 
                }
            }
            consecutive_forward_presses_ = 0; 
            publish_cmd_vel(); // 确保模式切换时速度立即更新
            return; 
        }

        // 3. 判断是否处于自主避障的具体动作中
        if (autonomous_avoidance_mode_ && avoidance_action_timer_ && !avoidance_action_timer_->is_canceled()) {
             RCLCPP_DEBUG(this->get_logger(), "In active avoidance maneuver, manual f_value %d (including f=0 from 'Return' button) ignored.", f);
             return; 
        }
    
        // 4. 自主避障模式开启
        if (autonomous_avoidance_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "Autonomous mode ON (idle or moving autonomously), manual f_value %d ignored.", f);
            consecutive_forward_presses_ = 0; // 如果收到任何手动指令，也重置画图计数
            return; // 忽略所有来自方向键和回正键的 f 值
        }
    

        const int FORWARD_COMMAND_F_VALUE = 1;
        const rclcpp::Duration MAX_INTERVAL_BETWEEN_PRESSES = rclcpp::Duration(2, 0); 

        if (f == FORWARD_COMMAND_F_VALUE) {
            rclcpp::Time current_time = this->now();
            if ((current_time - last_forward_press_time_).seconds() < MAX_INTERVAL_BETWEEN_PRESSES.seconds()) {
                consecutive_forward_presses_++;
            } else {
                consecutive_forward_presses_ = 1; 
            }
            last_forward_press_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Forward press registered. Count: %d", consecutive_forward_presses_);

            if (consecutive_forward_presses_ >= 3) {
                RCLCPP_INFO(this->get_logger(), "Three consecutive forward presses detected! Starting square pattern.");
                start_square_pattern();
                consecutive_forward_presses_ = 0; 
                return; 
            }
        } else {
            consecutive_forward_presses_ = 0; 
        }

        // 手动控制逻辑（此处将前进速度设置为0.3，可根据需要进行更改）
        if (f == 1) { // 前进
            current_linear_x_ = 0.3; current_angular_z_ = 0.0;
        } else if (f == 0) { // 停止 
            current_linear_x_ = 0.0; current_angular_z_ = 0.0;
        } else if (f == 2) { // 后退
            current_linear_x_ = -0.3; current_angular_z_ = 0.0;
        } else if (f == 3) { // 左转
            current_linear_x_ = 0.0; current_angular_z_ = 0.5;
        } else if (f == 4) { // 右转
            current_linear_x_ = 0.0; current_angular_z_ = -0.5;
        } else if (f != 10) { // 排除掉已经处理过的 f=10
            current_linear_x_ = 0.0; current_angular_z_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "Received unknown f value for manual control: %d, stopping.", f);
        }
        // 只有在手动模式下打印
        if (!autonomous_avoidance_mode_) {
            RCLCPP_INFO(this->get_logger(), "Updated velocity (manual control): linear.x=%.2f, angular.z=%.2f",
                    current_linear_x_, current_angular_z_);
        }
    }

    void start_square_pattern() {
        if (executing_square_pattern_) {
            RCLCPP_WARN(this->get_logger(), "Square Pattern: Already executing.");
            return; 
        }

        executing_square_pattern_ = true;
        square_pattern_step_ = 0; 
        RCLCPP_INFO(this->get_logger(), "Square Pattern: Starting.");
        
        execute_square_step(); 
    }
    
    void execute_square_step() {
    if (!executing_square_pattern_) {
        RCLCPP_INFO(this->get_logger(), "Square Pattern: Not executing, exiting step function.");
        return;
    }


    if (square_pattern_timer_ && !square_pattern_timer_->is_canceled()) {
        square_pattern_timer_->cancel();
    }

    square_pattern_step_++;
    RCLCPP_INFO(this->get_logger(), "Square Pattern: Attempting to execute step %d", square_pattern_step_);

    const auto side_duration = std::chrono::seconds(2); 
    const auto turn_duration = std::chrono::seconds(2); // 这个时间可根据需要调整以实现不同角度转弯
    const auto stop_delay_duration = std::chrono::milliseconds(200); // 动作完成后到下一步开始前的短暂停顿

   
    auto action_completed_callback = [this, stop_delay_duration]() {
        this->current_linear_x_ = 0.0; 
        this->current_angular_z_ = 0.0; 
        this->publish_cmd_vel(); // 立即发布停止指令
        RCLCPP_INFO(this->get_logger(), "Square Pattern: Step %d action completed, stopping. Waiting for next step.", this->square_pattern_step_);

        // 如果已经完成了所有步骤，则结束模式
        if (this->square_pattern_step_ >= 8) { 
            RCLCPP_INFO(this->get_logger(), "Square Pattern: Finished all steps.");
            this->executing_square_pattern_ = false;
            this->square_pattern_step_ = 0;
            if(this->square_pattern_timer_ && !this->square_pattern_timer_->is_canceled()) this->square_pattern_timer_->cancel(); // 取消最后的定时器
            return;
        }

        this->square_pattern_timer_ = this->create_wall_timer(stop_delay_duration, [this]() {
            if(this->square_pattern_timer_ && !this->square_pattern_timer_->is_canceled()) this->square_pattern_timer_->cancel(); // 取消这个短暂停顿的定时器
            this->execute_square_step(); // 进入下一个动作步骤
        });
    };
    
    switch (square_pattern_step_) {
        case 1: 
        case 3: 
        case 5: 
        case 7: 
            RCLCPP_INFO(this->get_logger(), "Square Pattern: Step %d - Moving forward.", square_pattern_step_);
            current_linear_x_ = 0.3; current_angular_z_ = 0.0;
            publish_cmd_vel(); // 立即发布前进指令
            square_pattern_timer_ = this->create_wall_timer(side_duration, action_completed_callback);
            break;

        case 2: 
        case 4: 
        case 6: 
        case 8: 
            RCLCPP_INFO(this->get_logger(), "Square Pattern: Step %d - Turning.", square_pattern_step_);
            current_linear_x_ = 0.0; current_angular_z_ = -0.95; // 左转 (或根据需要设为右转)
            publish_cmd_vel(); // 立即发布转弯指令
            square_pattern_timer_ = this->create_wall_timer(turn_duration, action_completed_callback);
            break;
            
        default: 
            RCLCPP_ERROR(this->get_logger(), "Square Pattern: Unexpected step %d. Stopping pattern.", square_pattern_step_);
            current_linear_x_ = 0.0; current_angular_z_ = 0.0;
            publish_cmd_vel();
            executing_square_pattern_ = false;
            square_pattern_step_ = 0;
            if(square_pattern_timer_ && !square_pattern_timer_->is_canceled()) square_pattern_timer_->cancel();
            break;
        }
    }

    void publish_cmd_vel() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = current_linear_x_;
        msg.angular.z = current_angular_z_;
        publisher_->publish(msg);
        // 这条日志打印会非常频繁，可以考虑在特定条件下打印，或者降低打印频率，此处注释
        // RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x=%.2f, angular.z=%.2f",
        //               msg.linear.x, msg.angular.z);
    }

    // 已有的成员变量
    int sockfd_;
    std::thread socket_thread_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr main_velocity_timer_; // 重命名原来的timer_
    float current_linear_x_;
    float current_angular_z_;

    // 新增用于“画正方形”功能的成员变量
    int consecutive_forward_presses_;
    rclcpp::Time last_forward_press_time_;
    bool executing_square_pattern_;
    int square_pattern_step_;
    rclcpp::TimerBase::SharedPtr square_pattern_timer_; // 用于画图序列的定时器
    
    // 新增用于激光雷达和避障的成员变量
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    bool obstacle_detected_;
    bool autonomous_avoidance_mode_; // 标记是否处于自主避障模式
    // 一个定时器来控制旋转的时长
    rclcpp::TimerBase::SharedPtr avoidance_action_timer_; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FReceiverNode>());
    rclcpp::shutdown();
    return 0;
}