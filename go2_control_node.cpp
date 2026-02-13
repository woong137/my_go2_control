/**********************************************************************
 * Go2 Unified Control Node - Full Features
 ***********************************************************************/

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>

#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

class Go2FinalControlNode : public rclcpp::Node {
public:
    Go2FinalControlNode() : Node("go2_final_control_node"), sport_client_(this) {
        
        // ---------------------------------------------------------
        // 1. 서비스 서버 등록
        // ---------------------------------------------------------

        // [기본 제어]
        srv_damp_        = create_srv("damp",         0);  // 댐핑(힘 풀기)
        srv_stand_up_    = create_srv("stand_up",     1);  // 일어서기
        srv_stand_down_  = create_srv("stand_down",   2);  // 엎드리기
        srv_recovery_    = create_srv("recovery",     3);  // 넘어졌을 때 복구
        srv_balance_     = create_srv("balance_stand", 4); // 밸런스 스탠딩
        srv_stop_        = create_srv("stop",         11); // 이동 정지

        // [특수 동작 - 소셜/표현]
        srv_sit_         = create_srv("sit",          100);
        srv_rise_sit_    = create_srv("rise_sit",     101);
        srv_wave_        = create_srv("wave",         102); // Hello
        srv_stretch_     = create_srv("stretch",      103);
        srv_content_     = create_srv("content",      104); // 기쁨/만족 표현
        srv_scrape_      = create_srv("scrape",       105); // 긁기 동작
        srv_dance1_      = create_srv("dance1",       106);
        srv_dance2_      = create_srv("dance2",       107);
        srv_heart_       = create_srv("heart",        108); // 하트 그리기

        // [고난도 동작 - 점프/플립]
        srv_front_flip_   = create_srv("front_flip",   110);
        srv_front_jump_   = create_srv("front_jump",   111);
        srv_front_pounce_ = create_srv("front_pounce", 112);
        srv_left_flip_    = create_srv("left_flip",    113);
        srv_back_flip_    = create_srv("back_flip",    114);
        
        // [특수 이동/자세 모드] (bool flag가 필요한 경우 true/Start로 기본 설정)
        srv_hand_stand_   = create_srv("hand_stand",   120);
        srv_cross_step_   = create_srv("cross_step",   121);
        srv_walk_upright_ = create_srv("walk_upright", 122); // 직립 보행
        srv_free_bound_   = create_srv("free_bound",   123);
        srv_free_jump_    = create_srv("free_jump",    124);
        srv_free_avoid_   = create_srv("free_avoid",   125);
        
        // [보행 스타일 변경]
        srv_static_walk_   = create_srv("static_walk",   200);
        srv_trot_run_      = create_srv("trot_run",      201);
        srv_economic_gait_ = create_srv("economic_gait", 202);
        srv_classic_walk_  = create_srv("classic_walk",  203);

        // ---------------------------------------------------------
        // 2. 구독자 설정
        // ---------------------------------------------------------
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Go2FinalControlNode::CmdVelHandler, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 1, [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
                current_state_ = *msg;
            });

        // ---------------------------------------------------------
        // 3. 제어 루프 타이머 (20Hz)
        // ---------------------------------------------------------
        timer_ = this->create_wall_timer(50ms, std::bind(&Go2FinalControlNode::ControlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Go2 Full Feature Node Started.");
    }

private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr create_srv(const std::string& name, int mode) {
        return this->create_service<std_srvs::srv::Empty>(
            name,
            [this, mode, name](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                               std::shared_ptr<std_srvs::srv::Empty::Response>) {
                this->current_mode_ = mode;
                this->action_done_ = false;
                RCLCPP_INFO(this->get_logger(), "Service Triggered: [%s] (Mode ID: %d)", name.c_str(), mode);
            }
        );
    }

    void CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_vx_ = msg->linear.x;
        target_vy_ = msg->linear.y;
        target_vyaw_ = msg->angular.z;

        // 속도 입력이 감지되면 즉시 MOVE 모드(9)로 전환
        if (std::abs(target_vx_) > 0.001 || std::abs(target_vy_) > 0.001 || std::abs(target_vyaw_) > 0.001) {
            RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, vyaw: %f", target_vx_, target_vy_, target_vyaw_);
            current_mode_ = 9; 
            action_done_ = false;
        }
    }

    void ControlLoop() {
        // 이미 명령을 수행했고, 연속적인 속도 제어(MOVE) 상태가 아니라면 리턴
        if (action_done_ && current_mode_ != 9) return;

        switch (current_mode_) {
            // --- 이동 제어 ---
            case 9: // MOVE
                if (std::abs(target_vx_) < 0.001 && std::abs(target_vy_) < 0.001 && std::abs(target_vyaw_) < 0.001) {
                    sport_client_.StopMove(req_);
                } else {
                    sport_client_.Move(req_, target_vx_, target_vy_, target_vyaw_);
                }
                break;
            case 11: sport_client_.StopMove(req_); action_done_ = true; break;

            // --- 기본 자세 ---
            case 0: sport_client_.Damp(req_); action_done_ = true; break;
            case 1: sport_client_.StandUp(req_); action_done_ = true; break;
            case 2: sport_client_.StandDown(req_); action_done_ = true; break;
            case 3: sport_client_.RecoveryStand(req_); action_done_ = true; break;
            case 4: sport_client_.BalanceStand(req_); action_done_ = true; break;

            // --- 소셜/표현 ---
            case 100: sport_client_.Sit(req_); action_done_ = true; break;
            case 101: sport_client_.RiseSit(req_); action_done_ = true; break;
            case 102: sport_client_.Hello(req_); action_done_ = true; break; // wave
            case 103: sport_client_.Stretch(req_); action_done_ = true; break;
            case 104: sport_client_.Content(req_); action_done_ = true; break;
            case 105: sport_client_.Scrape(req_); action_done_ = true; break;
            case 106: sport_client_.Dance1(req_); action_done_ = true; break;
            case 107: sport_client_.Dance2(req_); action_done_ = true; break;
            case 108: sport_client_.Heart(req_); action_done_ = true; break;

            // --- 고난도 점프/플립 ---
            case 110: sport_client_.FrontFlip(req_); action_done_ = true; break;
            case 111: sport_client_.FrontJump(req_); action_done_ = true; break;
            case 112: sport_client_.FrontPounce(req_); action_done_ = true; break;
            case 113: sport_client_.LeftFlip(req_); action_done_ = true; break;
            case 114: sport_client_.BackFlip(req_); action_done_ = true; break;

            // --- 특수 이동/자세 (Flag 필요한 함수는 true로 호출) ---
            case 120: sport_client_.HandStand(req_, true); action_done_ = true; break;
            case 121: sport_client_.CrossStep(req_, true); action_done_ = true; break;
            case 122: sport_client_.WalkUpright(req_, true); action_done_ = true; break;
            case 123: sport_client_.FreeBound(req_, true); action_done_ = true; break;
            case 124: sport_client_.FreeJump(req_, true); action_done_ = true; break;
            case 125: sport_client_.FreeAvoid(req_, true); action_done_ = true; break;

            // --- 보행 스타일 ---
            case 200: sport_client_.StaticWalk(req_); action_done_ = true; break;
            case 201: sport_client_.TrotRun(req_); action_done_ = true; break;
            case 202: sport_client_.EconomicGait(req_); action_done_ = true; break;
            case 203: sport_client_.ClassicWalk(req_, true); action_done_ = true; break;

            default: break;
        }
    }

    SportClient sport_client_;
    unitree_api::msg::Request req_;
    unitree_go::msg::SportModeState current_state_;

    // 서비스 객체 리스트
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr 
        srv_damp_, srv_stand_up_, srv_stand_down_, srv_recovery_, srv_balance_, srv_stop_,
        srv_sit_, srv_rise_sit_, srv_wave_, srv_stretch_, srv_content_, srv_scrape_,
        srv_dance1_, srv_dance2_, srv_heart_,
        srv_front_flip_, srv_front_jump_, srv_front_pounce_, srv_left_flip_, srv_back_flip_,
        srv_hand_stand_, srv_cross_step_, srv_walk_upright_, srv_free_bound_, srv_free_jump_, srv_free_avoid_,
        srv_static_walk_, srv_trot_run_, srv_economic_gait_, srv_classic_walk_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double target_vx_ = 0.0, target_vy_ = 0.0, target_vyaw_ = 0.0;
    int current_mode_ = 9; 
    bool action_done_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2FinalControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}