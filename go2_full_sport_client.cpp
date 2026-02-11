/**********************************************************************
 Copyright (c) 2024, Unitree Robotics. All rights reserved.
 ***********************************************************************/

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

// 공식 문서 Motion Services V2.0 기반 열거형 확장
enum TestMode {
    /*--- 기초 동작 ---*/
    DAMP = 0,               // 댐핑 (전원 끔 유사)
    STAND_UP = 1,           // 서기
    STAND_DOWN = 2,         // 엎드리기
    RECOVERY_STAND = 3,     // 복구 서기
    
    /*--- 이동 및 자세 ---*/
    MOVE = 9,               // 이동
    BALANCE_STAND = 10,     // 균형 서기
    STOP_MOVE = 11,         // 이동 정지

    /*--- 특수 동작 (V2.0 추가) ---*/
    SIT = 100,              // 앉기
    RISE_SIT = 101,         // 앉았다 일어서기
    WAVE = 102,             // 손 흔들기
    STRETCH = 103,          // 기지개 켜기
    WALLOW = 104,           // 뒹굴기
    PRAY = 105,             // 기도하기
    DANCE1 = 106,           // 춤 1
    DANCE2 = 107,           // 춤 2
    HEART = 108,            // 하트 그리기

    /*--- 점프 및 고급 (V2.0 추가) ---*/
    JUMP_YAW = 201,         // 제자리 회전 점프
    PRONOUNCE = 202,        // 짖기 (소리)
    WHEEL_LEG = 203,        // 휠-레그 모드 전환
};

class Go2FullSportClient : public rclcpp::Node {
public:
    explicit Go2FullSportClient(int test_mode) 
        : Node("go2_full_sport_client"), sport_client_(this), test_mode_(test_mode) {
        
        // 상태 구독
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 10,
            std::bind(&Go2FullSportClient::StateCallback, this, std::placeholders::_1));

        // 주기적 제어 타이머 (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&Go2FullSportClient::ExecuteControl, this));
        
        RCLCPP_INFO(this->get_logger(), "Go2 Sport V2.0 Node Started. Mode: %d", test_mode_);
    }

private:
    void StateCallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        current_state_ = *msg;
    }

    void ExecuteControl() {
        // 일회성 실행이 필요한 동작을 위한 플래그 처리
        if (action_done_) return;

        switch (test_mode_) {
            // 기본 동작
            case DAMP: sport_client_.Damp(req_); break;
            case STAND_UP: sport_client_.StandUp(req_); break;
            case STAND_DOWN: sport_client_.StandDown(req_); break;
            case RECOVERY_STAND: sport_client_.RecoveryStand(req_); break;
            
            // 이동 제어 (예시값 입력)
            case MOVE: sport_client_.Move(req_, 0.5, 0.0, 0.0); break;
            case STOP_MOVE: sport_client_.StopMove(req_); break;
            case BALANCE_STAND:                  // 1. Balance stand (controlled by dBodyHeight + rpy)
                sport_client_.Euler(req_, 0.1, 0.2, 0.3); // roll, pitch, yaw
                // sport_client_.BodyHeight(0.0);      // relative height [-0.18~0.03]
                sport_client_.BalanceStand(req_);
                break;

            // V2.0 특수 동작 (대부분 일회성 호출 권장)
            case SIT: sport_client_.Sit(req_); action_done_ = true; break;
            case RISE_SIT: sport_client_.RiseSit(req_); action_done_ = true; break;
            case WAVE: sport_client_.Hello(req_); action_done_ = true; break; // Hello가 Wave 역할
            case STRETCH: sport_client_.Stretch(req_); action_done_ = true; break;
            // case WALLOW: sport_client_.Wallow(req_); action_done_ = true; break;
            // case PRAY: sport_client_.Pray(req_); action_done_ = true; break;
            case DANCE1: sport_client_.Dance1(req_); action_done_ = true; break;
            case DANCE2: sport_client_.Dance2(req_); action_done_ = true; break;
            case HEART: sport_client_.Heart(req_); action_done_ = true; break;

            // 고급 기능
            // case JUMP_YAW: sport_client_.JumpYaw(req_); action_done_ = true; break;
            
            // 추가적인 파라미터 제어 예시 (속도/높이 등)
            case 300: // Gait 설정 변경 예시
                // sport_client_.GaitType(req_, 1); // 1: Trot, 2: Climb, 3: Obstacle
                action_done_ = true;
                break;

            default:
                sport_client_.StopMove(req_);
                break;
        }
    }

    SportClient sport_client_;
    unitree_api::msg::Request req_;
    unitree_go::msg::SportModeState current_state_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int test_mode_;
    bool action_done_ = false;
};

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Modes:" << std::endl;
        std::cout << "  0: DAMP           1: STAND_UP      2: STAND_DOWN" << std::endl;
        std::cout << "  3: RECOVERY_STAND 9: MOVE            10: BALANCE_STAND" << std::endl;
        std::cout << "  11: STOP_MOVE" << std::endl;
        std::cout << "Special Actions (V2.0):" << std::endl;
        std::cout << "  100: SIT          101: RISE_SIT    102: WAVE (Hello)" << std::endl;
        std::cout << "  103: STRETCH      106: DANCE1      107: DANCE2" << std::endl;
        std::cout << "  108: HEART" << std::endl;

        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2FullSportClient>(std::atoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}