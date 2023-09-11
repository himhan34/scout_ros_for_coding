/*
 * scout_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description: 스카우트 로봇과 ROS 메시지를 처리하는 노드의 구현
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_base/scout_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include "scout_msgs/ScoutStatus.h"
#include "scout_msgs/ScoutBmsStatus.h"
#include "scout_msgs/ScoutLightCmd.h"

namespace westonrobot
{
  // ScoutROSMessenger 클래스 생성자 - 노드 핸들 초기화
  ScoutROSMessenger::ScoutROSMessenger(ros::NodeHandle *nh)
      : scout_(nullptr), nh_(nh) {}

  // ScoutROSMessenger 클래스 생성자 - 스카우트 로봇 및 노드 핸들 초기화
  ScoutROSMessenger::ScoutROSMessenger(ScoutRobot *scout, ros::NodeHandle *nh, bool is_scout_omni)
      : scout_(scout), nh_(nh), is_scout_omni(is_scout_omni) {}

  // ROS 토픽 및 메시지 구독 설정
  void ScoutROSMessenger::SetupSubscription()
  {
    // 오도메트리 메시지 발행자 설정
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);

    // 스카우트 상태 메시지 발행자 설정
    status_publisher_ = nh_->advertise<scout_msgs::ScoutStatus>("/scout_status", 10);

    // 스카우트 BMS 상태 메시지 발행자 설정
    BMS_status_publisher_ = nh_->advertise<scout_msgs::ScoutBmsStatus>("/BMS_status", 10);

    // 명령 메시지 구독자 설정
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this);

    // 조명 제어 메시지 구독자 설정
    light_cmd_subscriber_ = nh_->subscribe<scout_msgs::ScoutLightCmd>(
        "/scout_light_control", 5, &ScoutROSMessenger::LightCmdCallback, this);
  }

  // 이동 명령 메시지 수신 콜백 함수
  void ScoutROSMessenger::TwistCmdCallback(
      const geometry_msgs::Twist::ConstPtr &msg)
  {
    if (!simulated_robot_)
    {
      if (is_scout_omni)
        dynamic_cast<ScoutMiniOmniRobot *>(scout_)->SetMotionCommand(msg->linear.x, msg->angular.z, msg->linear.y);
      else
        scout_->SetMotionCommand(msg->linear.x, msg->angular.z);
    }
    else
    {
      // 시뮬레이션 로봇인 경우 수신한 메시지를 저장합니다.
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

void ScoutROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                    double &angular)
{
    // 뮤텍스(락)를 사용하여 다른 스레드와의 동기화를 보장합니다.
    std::lock_guard<std::mutex> guard(twist_mutex_);

    // 시뮬레이션 모드에서 현재 이동 명령을 얻어와서 linear와 angular 변수에 저장합니다.
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
}

void ScoutROSMessenger::LightCmdCallback(
      const scout_msgs::ScoutLightCmd::ConstPtr &msg)
{
    // 시뮬레이션 로봇이 아닌 경우에만 실행됩니다.
    if (!simulated_robot_)
    {
        // 조명 제어 활성화 여부 확인
        if (msg->enable_cmd_light_control)
        {
            LightCommandMessage cmd;

            // 전면 조명 모드 설정
            switch (msg->front_mode)
            {
                case scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF:
                {
                    cmd.front_light.mode = CONST_OFF;
                    break;
                }
                case scout_msgs::ScoutLightCmd::LIGHT_CONST_ON:
                {
                    cmd.front_light.mode = CONST_ON;
                    break;
                }
                case scout_msgs::ScoutLightCmd::LIGHT_BREATH:
                {
                    cmd.front_light.mode = BREATH;
                    break;
                }
                case scout_msgs::ScoutLightCmd::LIGHT_CUSTOM:
                {
                    cmd.front_light.mode = CUSTOM;
                    cmd.front_light.custom_value = msg->front_custom_value;
                    break;
                }
            }

            // 후면 조명 모드 설정
            switch (msg->rear_mode)
            {
                case scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF:
                {
                    cmd.rear_light.mode = CONST_OFF;
                    break;
                }
                case scout_msgs::ScoutLightCmd::LIGHT_CONST_ON:
                {
                    cmd.rear_light.mode = CONST_ON;
                    break;
                }
                case scout_msgs::ScoutLightCmd::LIGHT_BREATH:
                {
                    cmd.rear_light.mode = BREATH;
                    break;
                }
                case scout_msgs::ScoutLightCmd::LIGHT_CUSTOM:
                {
                    cmd.rear_light.mode = CUSTOM;
                    cmd.rear_light.custom_value = msg->rear_custom_value;
                    break;
                }
            }

            // 스카우트 로봇에 조명 제어 명령 설정을 전달합니다.
            scout_->SetLightCommand(cmd.front_light.mode, cmd.front_light.custom_value, cmd.rear_light.mode, cmd.rear_light.custom_value);
        }
        else
        {
            // 조명 제어 비활성화 명령을 스카우트 로봇에 전달합니다.
            scout_->DisableLightControl();
        }
    }
    else
    {
        // 시뮬레이션 로봇인 경우 메시지를 출력합니다.
        std::cout << "시뮬레이션 로봇이 조명 제어 명령을 수신했습니다." << std::endl;
    }
}


void ScoutROSMessenger::PublishStateToROS()
{
    // 현재 시간을 얻어옵니다.
    current_time_ = ros::Time::now();

    // 이전 시간과의 시간 차이를 계산하여 시간 간격(dt)을 얻어옵니다.
    double dt = (current_time_ - last_time_).toSec();

    // 초기 실행 여부를 나타내는 정적 변수입니다.
    static bool init_run = true;

    // 초기 실행인 경우, 이전 시간을 현재 시간으로 설정하고 함수 실행을 중지합니다.
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    // 스카우트 로봇의 상태 정보와 액추에이터 상태 정보를 얻어옵니다.
    auto robot_state = scout_->GetRobotState();
    auto actuator_state = scout_->GetActuatorState();

    // 스카우트 상태 메시지를 생성합니다.
    scout_msgs::ScoutStatus status_msg;
    scout_msgs::ScoutBmsStatus bms_status;

    // 메시지 헤더의 타임 스탬프를 현재 시간으로 설정합니다.
    status_msg.header.stamp = current_time_;

    // 로봇의 선속도와 각속도를 메시지에 설정합니다.
    status_msg.linear_velocity = robot_state.motion_state.linear_velocity;
    status_msg.angular_velocity = robot_state.motion_state.angular_velocity;

    // 로봇의 기본 상태, 제어 모드, 오류 코드, 배터리 전압을 메시지에 설정합니다.
    status_msg.base_state = robot_state.system_state.vehicle_state;
    status_msg.control_mode = robot_state.system_state.control_mode;
    status_msg.fault_code = robot_state.system_state.error_code;
    status_msg.battery_voltage = robot_state.system_state.battery_voltage;

    // 조명 제어가 활성화되어 있는지 메시지에 설정합니다.
    status_msg.light_control_enabled = robot_state.light_state.enable_cmd_ctrl;

    // 전면 조명 상태를 메시지에 설정합니다.
    status_msg.front_light_state.mode = robot_state.light_state.front_light.mode;
    status_msg.front_light_state.custom_value = robot_state.light_state.front_light.custom_value;

    // 후면 조명 상태를 메시지에 설정합니다.
    status_msg.rear_light_state.mode = robot_state.light_state.rear_light.mode;
    status_msg.rear_light_state.custom_value = robot_state.light_state.rear_light.custom_value;

    // 프로토콜 버전에 따라 모터 및 드라이버 상태 정보를 메시지에 설정합니다.
    if (scout_->GetParserProtocolVersion() == ProtocolVersion::AGX_V1)
    {
        for (int i = 0; i < 4; ++i)
        {
            status_msg.motor_states[i].current = actuator_state.actuator_state[i].current;
            status_msg.motor_states[i].rpm = actuator_state.actuator_state[i].rpm;
            status_msg.motor_states[i].temperature = actuator_state.actuator_state[i].motor_temp;
            status_msg.driver_states[i].driver_temperature = actuator_state.actuator_state[i].driver_temp;
        }
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            status_msg.motor_states[i].current = actuator_state.actuator_hs_state[i].current;
            status_msg.motor_states[i].rpm = actuator_state.actuator_hs_state[i].rpm;
            status_msg.motor_states[i].temperature = actuator_state.actuator_ls_state[i].motor_temp;
            status_msg.motor_states[i].motor_pose = actuator_state.actuator_hs_state[i].pulse_count;
            status_msg.driver_states[i].driver_state = actuator_state.actuator_ls_state[i].driver_state;
            status_msg.driver_states[i].driver_voltage = actuator_state.actuator_ls_state[i].driver_voltage;
            status_msg.driver_states[i].driver_temperature = actuator_state.actuator_ls_state[i].driver_temp;
        }
    }

    // 스카우트 상태 메시지를 ROS에 발행합니다.
    status_publisher_.publish(status_msg);

    // 이동 거리를 기반으로 오도메트리와 tf 메시지를 발행합니다.
    PublishOdometryToROS(robot_state.motion_state.linear_velocity, robot_state.motion_state.angular_velocity, dt);

    // 다음 통합을 위해 현재 시간을 이전 시간으로 저장합니다.
    last_time_ = current_time_;
}
  
// BMS 상태 정보를 `bms_status` 변수에 설정합니다.
bms_status.SOC = state.SOC; // 배터리 상태 (State of Charge)
bms_status.SOH = state.SOH; // 배터리 상태 (State of Health)
bms_status.battery_voltage = state.bms_battery_voltage; // 배터리 전압
bms_status.battery_current = state.battery_current; // 배터리 전류
bms_status.battery_temperature = state.battery_temperature; // 배터리 온도
bms_status.Alarm_Status_1 = state.Alarm_Status_1; // 경보 상태 1
bms_status.Alarm_Status_2 = state.Alarm_Status_2; // 경보 상태 2
bms_status.Warning_Status_1 = state.Warning_Status_1; // 경고 상태 1
bms_status.Warning_Status_2 = state.Warning_Status_2; // 경고 상태 2

// `bms_status` 메시지를 ROS에 발행합니다.
BMS_status_publisher_.publish(bms_status);

// 스카우트 상태 정보를 ROS에 발행합니다.
status_publisher_.publish(status_msg);

// 오도메트리 및 tf 메시지를 발행합니다. 이때, 로봇의 선속도와 각속도, 그리고 경과 시간(dt)을 사용합니다.
PublishOdometryToROS(robot_state.motion_state.linear_velocity, robot_state.motion_state.angular_velocity, dt);

// 다음 통합을 위해 현재 시간을 이전 시간으로 저장합니다.
last_time_ = current_time_;
}
void ScoutROSMessenger::PublishSimStateToROS(double linear, double angular)
{
    // 현재 시간을 기록합니다.
    current_time_ = ros::Time::now();

    // 경과 시간을 계산합니다.
    double dt = (current_time_ - last_time_).toSec();

    // 초기 실행 여부를 나타내는 변수입니다.
    static bool init_run = true;

    // 초기 실행 시, 이전 시간을 설정하고 함수를 종료합니다.
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    // 스카우트 상태 메시지를 생성합니다.
    scout_msgs::ScoutStatus status_msg;

    // 상태 메시지의 타임스탬프를 현재 시간으로 설정합니다.
    status_msg.header.stamp = current_time_;

    // 스카우트의 선속도와 각속도를 설정합니다.
    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;

    // 로봇의 기본 상태, 제어 모드, 오류 코드, 배터리 전압 및 조명 제어 상태를 설정합니다.
    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;
    status_msg.light_control_enabled = false;

    // 스카우트 상태 메시지를 ROS에 발행합니다.
    status_publisher_.publish(status_msg);

    // 오도메트리 및 tf 메시지를 발행합니다.
    PublishOdometryToROS(linear, angular, dt);

    // 다음 통합을 위해 현재 시간을 이전 시간으로 기록합니다.
    last_time_ = current_time_;
}


  void ScoutROSMessenger::PublishOdometryToROS(double linear, double angular, double dt)
{
    // 숫자적 적분을 수행하여 로봇의 위치 및 방향 추정

    // 로봇의 선속도 및 각속도 업데이트
    linear_speed_ = linear;
    angular_speed_ = angular;

    // 적분에 사용할 변위 및 회전량 계산
    double d_x = linear_speed_ * std::cos(theta_) * dt; // x 방향 변위
    double d_y = linear_speed_ * std::sin(theta_) * dt; // y 방향 변위
    double d_theta = angular_speed_ * dt;              // 회전량

    // 로봇의 위치 업데이트
    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    // 현재 각도(theta)를 쿼터니언(quaternion) 형태로 변환
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // tf 변환 정보 생성
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;  // 현재 시간 설정
    tf_msg.header.frame_id = odom_frame_; // tf 메시지의 기준 프레임 설정
    tf_msg.child_frame_id = base_frame_;  // tf 메시지의 자식 프레임 설정
    
    // 로봇의 위치 및 방향 정보를 tf 메시지에 추가
    tf_msg.transform.translation.x = position_x_; // x 위치
    tf_msg.transform.translation.y = position_y_; // y 위치
    tf_msg.transform.translation.z = 0.0;         // z 위치 (2D 로봇이므로 0)
    tf_msg.transform.rotation = odom_quat;          // 로봇의 방향 (쿼터니언 형태)
    
    // tf 메시지를 발행 (만약 pub_tf가 true인 경우에만 발행)
    if (pub_tf) tf_broadcaster_.sendTransform(tf_msg);
    
    // odometry와 tf 메시지를 발행하기 위한 정보 설정
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;      // 현재 시간 설정
    odom_msg.header.frame_id = odom_frame_;     // odometry 메시지의 기준 프레임 설정
    odom_msg.child_frame_id = base_frame_;      // odometry 메시지의 자식 프레임 설정


   // 오도메트리(Odometry) 메시지 설정
    // 로봇의 x, y, z 위치 설정 (2D 로봇이므로 z는 0)
    odom_msg.pose.pose.position.x = position_x_;  // x 위치
    odom_msg.pose.pose.position.y = position_y_;  // y 위치
    odom_msg.pose.pose.position.z = 0.0;          // z 위치 (2D 로봇이므로 0)
    
    // 로봇의 방향 정보를 설정 (쿼터니언 형태의 방향 정보)
    odom_msg.pose.pose.orientation = odom_quat;   // 로봇의 방향 설정
    
    // 로봇의 선속도 및 각속도 설정
    odom_msg.twist.twist.linear.x = linear_speed_;     // 선속도 (x 방향)
    odom_msg.twist.twist.linear.y = 0.0;               // 선속도 (y 방향, 2D 로봇에서는 0)
    odom_msg.twist.twist.angular.z = angular_speed_;   // 각속도 (z 방향)
    
    // 오도메트리 메시지를 발행
    odom_publisher_.publish(odom_msg);
}
