#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <iomanip>
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

class Go2CmdVelNode : public rclcpp::Node {
public:
    Go2CmdVelNode() : Node("go2_cmd_vel_node"), sport_client_(this) {
        
        // 1. cmd_vel 구독자 설정 (표준 Twist 메시지)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Go2CmdVelNode::CmdVelHandler, this, std::placeholders::_1));

        // 2. 로봇 상태 피드백 구독 (선택 사항)
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 1,
            [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
                current_state_ = *msg;
            });

        // 3. 제어 명령을 주기적으로 전송할 타이머 (10Hz ~ 20Hz 권장)
        timer_ = this->create_wall_timer(50ms, std::bind(&Go2CmdVelNode::ControlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Go2 CmdVel Control Node Started.");
    }

private:
    // 속도 명령 수신 콜백
    void CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_vx_ = msg->linear.x;
        target_vy_ = msg->linear.y;
        target_vyaw_ = msg->angular.z;
    }

    // 주기적인 제어 루프
    void ControlLoop() {
        std::cout << "[target]" << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "vx: " << target_vx_ << ", vy: " << target_vy_ << ", vyaw: " << target_vyaw_ << std::endl;

        std::cout << "[current]" << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "vx: " << current_state_.velocity[0] << ", vy: " << current_state_.velocity[1] << ", vyaw: " << current_state_.yaw_speed << std::endl;

        // 속도 값이 모두 0인 경우 멈춤 명령, 아닌 경우 이동 명령
        if (std::abs(target_vx_) < 0.001 && std::abs(target_vy_) < 0.001 && std::abs(target_vyaw_) < 0.001) {
            sport_client_.StopMove(req_);
        } else {
            // Unitree SDK Move: (request, vx, vy, vyaw)
            // vx: 전진(+), vy: 좌측(+), vyaw: 반시계방향(+)
            sport_client_.Move(req_, target_vx_, target_vy_, target_vyaw_);

        }
    }

    // 변수 선언
    SportClient sport_client_;
    unitree_api::msg::Request req_;
    unitree_go::msg::SportModeState current_state_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_vyaw_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CmdVelNode>();
    
    // MultiThreadedExecutor를 사용하여 통신 지연 방지
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
