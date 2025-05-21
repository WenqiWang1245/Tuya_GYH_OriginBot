#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define SOCKET_PATH "/tmp/f_value_socket"

class FReceiverNode : public rclcpp::Node {
public:
    FReceiverNode() : Node("f_receiver_node"), current_linear_x_(0.0), current_angular_z_(0.0) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        setup_socket();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FReceiverNode::publish_cmd_vel, this));
    }

    ~FReceiverNode() {
        close(sockfd_);
        unlink(SOCKET_PATH);
    }

private:
    void setup_socket() {
        sockfd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket error");
            rclcpp::shutdown();
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

        unlink(SOCKET_PATH);
        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
            perror("bind error");
            rclcpp::shutdown();
        }

        // 创建一个线程来监听套接字
        socket_thread_ = std::thread(&FReceiverNode::check_socket, this);
        socket_thread_.detach();
    }

    void check_socket() {
        int f;
        struct sockaddr_un addr;
        socklen_t addr_len = sizeof(addr);

        while (rclcpp::ok()) {
            // 接收 f 的值
            int n = recvfrom(sockfd_, &f, sizeof(f), 0, (struct sockaddr*)&addr, &addr_len);
            if (n > 0) {
                RCLCPP_INFO(this->get_logger(), "Received f value: %d", f);
                update_velocity(f);
            }
        }
    }

    void update_velocity(int f) {
        if (f == 1) {
            current_linear_x_ = 0.5;
            current_angular_z_ = 0.0;
        } else if (f == 0) {
            current_linear_x_ = 0.0;
            current_angular_z_ = 0.0;
        }
        RCLCPP_INFO(this->get_logger(), "Updated velocity: linear.x=%.2f, angular.z=%.2f",
                    current_linear_x_, current_angular_z_);
    }

    void publish_cmd_vel() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = current_linear_x_;
        msg.angular.z = current_angular_z_;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x=%.2f, angular.z=%.2f",
                    msg.linear.x, msg.angular.z);
    }

    int sockfd_;
    std::thread socket_thread_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float current_linear_x_;
    float current_angular_z_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FReceiverNode>());
    rclcpp::shutdown();
    return 0;
}