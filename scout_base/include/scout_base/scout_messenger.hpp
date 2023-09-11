/*
 * scout_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description: 스카우트 로봇의 ROS 메시지 처리를 담당하는 파일입니다.
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
  // ScoutROSMessenger 클래스 생성자
  ScoutROSMessenger::ScoutROSMessenger(ros::NodeHandle *nh)
      : scout_(nullptr), nh_(nh) {}

  // ScoutROSMessenger 클래스 생성자 (오버로드)
  ScoutROSMessenger::ScoutROSMessenger(ScoutRobot *scout, ros::NodeHandle *nh, bool is_scout_omni)
      : scout_(scout), nh_(nh), is_scout_omni(is_scout_omni) {}

  // ROS 토픽 구독 설정
  void ScoutROSMessenger::SetupSubscription()
  {
    // 오도메트리 발행자 설정
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
    status_publisher_ = nh_->advertise<scout_msgs::ScoutStatus>("/scout_status", 10);
    BMS_status_publisher_ = nh_->advertise<scout_msgs::ScoutBmsStatus>("/BMS_status", 10);

    // 명령어 구독자 설정
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this);
    light_cmd_subscriber_ = nh_->subscribe<scout_msgs::ScoutLightCmd>(
        "/scout_light_control", 5, &ScoutROSMessenger::LightCmdCallback, this);
  }

  // 이동 명령어 콜백 함수
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
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }
}


void ScoutROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                    double &angular)
{
    // 현재 시뮬레이션 모드에서의 이동 명령어를 반환합니다.
    std::lock_guard<std::mutex> guard(twist_mutex_);
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
}

void ScoutROSMessenger::LightCmdCallback(
    const scout_msgs::ScoutLightCmd::ConstPtr &msg)
{
    if (!simulated_robot_)
    {
        // 로봇이 실제로 움직이는 경우

        if (msg->enable_cmd_light_control)
        {
            // 조명 제어 명령이 활성화된 경우

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

            // 스카우트 로봇에 조명 명령을 설정합니다.
            scout_->SetLightCommand(cmd.front_light.mode, cmd.front_light.custom_value, cmd.rear_light.mode, cmd.rear_light.custom_value);
        }
        else
        {
            // 조명 제어 명령이 비활성화된 경우
            scout_->DisableLightControl();
        }
    }
    else
    {
        // 시뮬레이션 로봇인 경우 메시지를 출력합니다.
        std::cout << "simulated robot received light control cmd." << std::endl;
    }
}

void ScoutROSMessenger::PublishStateToROS()
{
    // 현재 ROS 시간을 얻어옵니다.
    current_time_ = ros::Time::now();

    // 시간 간격을 계산합니다.
    double dt = (current_time_ - last_time_).toSec();

    // 초기 실행 여부를 확인하는 변수를 정의하고 초기화합니다.
    static bool init_run = true;
    if (init_run)
    {
        // 최초 실행 시간을 설정하고 함수를 종료합니다.
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    // 로봇 상태와 액추에이터 상태를 얻어옵니다.
    auto robot_state = scout_->GetRobotState();
    auto actuator_state = scout_->GetActuatorState();

    // 스카우트 상태 메시지를 생성합니다.
    scout_msgs::ScoutStatus status_msg;
    scout_msgs::ScoutBmsStatus bms_status;

    // 메시지의 타임스탬프를 현재 시간으로 설정합니다.
    status_msg.header.stamp = current_time_;

    // 로봇의 선속도와 각속도를 메시지에 추가합니다.
    status_msg.linear_velocity = robot_state.motion_state.linear_velocity;
    status_msg.angular_velocity = robot_state.motion_state.angular_velocity;

    // 로봇의 기본 상태를 메시지에 추가합니다.
    status_msg.base_state = robot_state.system_state.vehicle_state;

    // 로봇의 제어 모드를 메시지에 추가합니다.
    status_msg.control_mode = robot_state.system_state.control_mode;

    // 로봇의 오류 코드를 메시지에 추가합니다.
    status_msg.fault_code = robot_state.system_state.error_code;

    // 로봇의 배터리 전압을 메시지에 추가합니다.
    status_msg.battery_voltage = robot_state.system_state.battery_voltage;

    // 로봇의 조명 제어 상태를 메시지에 추가합니다.
    status_msg.light_control_enabled = robot_state.light_state.enable_cmd_ctrl;

    // 로봇의 전면 조명 상태를 메시지에 추가합니다.
    status_msg.front_light_state.mode = robot_state.light_state.front_light.mode;
    status_msg.front_light_state.custom_value = robot_state.light_state.front_light.custom_value;

    // 로봇의 후면 조명 상태를 메시지에 추가합니다.
    status_msg.rear_light_state.mode = robot_state.light_state.rear_light.mode;
    status_msg.rear_light_state.custom_value = robot_state.light_state.rear_light.custom_value;

    // 프로토콜 버전에 따라 모터 및 드라이버 상태를 메시지에 추가합니다.
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

    // 배터리 관련 상태를 BMS 메시지에 추가합니다.
    // 주석 처리된 코드는 주석 해제 후 필요한 정보를 추가하면 됩니다.
    // bms_status.SOC = state.SOC;
    // bms_status.SOH = state.SOH;
    // bms_status.battery_voltage = state.bms_battery_voltage;
    // bms_status.battery_current = state.battery_current;
    // bms_status.battery_temperature = state.battery_temperature;
    // bms_status.Alarm_Status_1 = state.Alarm_Status_1;
    // bms_status.Alarm_Status_2 = state.Alarm_Status_2;
    // bms_status.Warning_Status_1 = state.Warning_Status_1;
    // bms_status.Warning_Status_2 = state.Warning_Status_2;

    // BMS 상태 메시지를 발행합니다.
    BMS_status_publisher_.publish(bms_status);

    // 스카우트 상태 메시지를 발행합니다.
    status_publisher_.publish(status_msg);

    // 오도메트리 및 TF 정보를 발행합니다.
    PublishOdometryToROS(robot_state.motion_state.linear_velocity, robot_state.motion_state.angular_velocity, dt);

    // 다음 통합을 위해 시간을 기록합니다.
    last_time_ = current_time_;
}

void ScoutROSMessenger::PublishSimStateToROS(double linear, double angular)
{
    // 현재 ROS 시간을 얻어옵니다.
    current_time_ = ros::Time::now();

    // 시간 간격을 계산합니다.
    double dt = (current_time_ - last_time_).toSec();

    // 초기 실행 여부를 확인하는 변수를 정의하고 초기화합니다.
    static bool init_run = true;
    if (init_run)
    {
        // 최초 실행 시간을 설정하고 함수를 종료합니다.
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    // 스카우트 상태 메시지를 생성합니다.
    scout_msgs::ScoutStatus status_msg;

    // 메시지의 타임스탬프를 현재 시간으로 설정합니다.
    status_msg.header.stamp = current_time_;

    // 선속도와 각속도 정보를 메시지에 추가합니다.
    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;

    // 기본 상태, 제어 모드, 오류 코드, 배터리 전압 및 조명 제어 상태를 메시지에 설정합니다.
    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;
    status_msg.light_control_enabled = false;

    // 스카우트 상태 메시지를 발행합니다.
    status_publisher_.publish(status_msg);

    // 오도메트리 및 TF 정보를 발행합니다.
    PublishOdometryToROS(linear, angular, dt);

    // 다음 통합을 위해 시간을 기록합니다.
    last_time_ = current_time_;
}

void ScoutROSMessenger::PublishOdometryToROS(double linear, double angular, double dt)
{
    // 수치 적분을 수행하여 자세의 추정값을 얻습니다.
    linear_speed_ = linear;
    angular_speed_ = angular;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    // 자세에 대한 쿼터니언을 생성합니다.
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // TF 변환 메시지를 발행합니다.
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    // TF 메시지를 발행합니다.
    if (pub_tf)
        tf_broadcaster_.sendTransform(tf_msg);

    // 오도메트리 메시지를 생성하고 발행합니다.
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    // 오도메트리 메시지를 발행합니다.
    odom_publisher_.publish(odom_msg);
}
