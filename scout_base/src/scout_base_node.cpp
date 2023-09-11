#include <memory> // <memory> 헤더를 포함합니다.

#include <ros/ros.h> // ROS 관련 헤더를 포함합니다.
#include <nav_msgs/Odometry.h> // 네비게이션 메시지 관련 헤더를 포함합니다.
#include <sensor_msgs/JointState.h> // 센서 메시지 관련 헤더를 포함합니다.
#include <tf/transform_broadcaster.h> // 변환 브로드캐스터 관련 헤더를 포함합니다.

#include "ugv_sdk/mobile_robot/scout_robot.hpp" // UGV SDK에서 스카우트 로봇 관련 헤더를 포함합니다.
#include "ugv_sdk/utilities/protocol_detector.hpp" // UGV SDK에서 프로토콜 감지 관련 헤더를 포함합니다.
#include "scout_base/scout_messenger.hpp" // 스카우트 메신저 관련 헤더를 포함합니다.

// 위 코드는 C++의 주요 라이브러리 및 ROS와 관련된 헤더를 가져옵니다.
using namespace westonrobot; // westonrobot 네임스페이스를 사용합니다.

std::unique_ptr<ScoutRobot> robot; // ScoutRobot 클래스의 고유 포인터를 선언합니다.

int main(int argc, char **argv) {
  // ROS 노드 설정
  ros::init(argc, argv, "scout_odom"); // ROS 노드를 초기화하고 "scout_odom"으로 이름을 설정합니다.
  ros::NodeHandle node(""), private_node("~"); // ROS 노드 핸들러를 생성합니다.

  // Scout Mini를 제어하는지 여부를 확인합니다.
  bool is_scout_mini = false;
  static bool is_scout_omni = false; // 스태틱 변수로 스카우트 오미니 여부를 설정합니다.

  // private_node.param<bool>("is_scout_mini", is_scout_mini, false);
  // private_node에서 "is_scout_mini" 매개변수를 가져와 is_scout_mini 변수에 저장합니다.

  private_node.getParam("is_scout_mini", is_scout_mini); // private_node에서 "is_scout_mini" 매개변수를 가져와 is_scout_mini 변수에 저장합니다.

  std::cout << "스카우트 미니로 작동 중: " << is_scout_mini << std::endl; // 현재 스카우트 미니로 작동 중인지를 출력합니다.
   private_node.getParam("is_scout_omni", is_scout_omni); // private_node에서 "is_scout_omni" 매개변수를 가져와 is_scout_omni 변수에 저장합니다.
  std::cout << "스카우트 오미니로 작동 중: " << is_scout_omni << std::endl; // 현재 스카우트 오미니로 작동 중인지를 출력합니다.

  // 프로토콜 버전 확인
  ProtocolDetector detector; // ProtocolDetector 객체를 생성합니다.
  try
  {
      detector.Connect("can0"); // "can0" 포트에 연결합니다.

      auto proto = detector.DetectProtocolVersion(5); // 프로토콜 버전을 감지합니다.

      if (is_scout_mini && is_scout_omni) { // 스카우트 미니 및 스카우트 오미니를 제어 중인 경우
          if (proto == ProtocolVersion::AGX_V1) {
              std::cout << "감지된 프로토콜: AGX_V1 오미니" << std::endl;
              robot = std::unique_ptr<ScoutMiniOmniRobot>(
                          new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
          } else if (proto == ProtocolVersion::AGX_V2) {
              std::cout << "감지된 프로토콜: AGX_V2 오미니" << std::endl;
              robot = std::unique_ptr<ScoutMiniOmniRobot>(
                          new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
          } else {
              std::cout << "감지된 프로토콜: 알 수 없음" << std::endl;
              return -1;
          }
      } else { // 스카우트 미니 또는 스카우트 오미니를 제어 중이지 않은 경우
          if (proto == ProtocolVersion::AGX_V1) {
              std::cout << "감지된 프로토콜: AGX_V1" << std::endl;
              robot = std::unique_ptr<ScoutRobot>(
                          new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
          } else if (proto == ProtocolVersion::AGX_V2) {
              std::cout << "감지된 프로토콜: AGX_V2" << std::endl;
              robot = std::unique_ptr<ScoutRobot>(
                          new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
          } else {
              std::cout << "감지된 프로토콜: 알 수 없음" << std::endl;
              return -1;
          }
      }

      if (robot == nullptr)
          std::cout << "로봇 객체 생성 실패" << std::endl;
  }
  catch (const std::exception error)
  {
      ROS_ERROR("CAN을 활성화하거나 CAN 포트가 존재하는지 확인하세요"); // 오류 메시지를 출력합니다.
      ros::shutdown(); // ROS를 종료합니다.
  }

  ScoutROSMessenger messenger(robot.get(), &node, is_scout_omni); // ScoutROSMessenger 객체를 생성하고 로봇 및 노드 정보를 전달합니다.
// 연결하기 전에 매개변수 가져오기
std::string port_name;
private_node.param<std::string>("port_name", port_name, std::string("can0")); // "port_name" 매개변수 값을 가져와서 port_name 변수에 저장합니다. 기본값은 "can0"입니다.
private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom")); // "odom_frame" 매개변수 값을 가져와서 messenger의 odom_frame_ 멤버 변수에 저장합니다. 기본값은 "odom"입니다.
private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link")); // "base_frame" 매개변수 값을 가져와서 messenger의 base_frame_ 멤버 변수에 저장합니다. 기본값은 "base_link"입니다.
private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false); // "simulated_robot" 매개변수 값을 가져와서 messenger의 simulated_robot_ 멤버 변수에 저장합니다. 기본값은 false입니다.
private_node.param<int>("control_rate", messenger.sim_control_rate_, 50); // "control_rate" 매개변수 값을 가져와서 messenger의 sim_control_rate_ 멤버 변수에 저장합니다. 기본값은 50입니다.
private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_, std::string("odom")); // "odom_topic_name" 매개변수 값을 가져와서 messenger의 odom_topic_name_ 멤버 변수에 저장합니다. 기본값은 "odom"입니다.
private_node.param<bool>("pub_tf", messenger.pub_tf, true); // "pub_tf" 매개변수 값을 가져와서 messenger의 pub_tf 멤버 변수에 저장합니다. 기본값은 true입니다.

if (!messenger.simulated_robot_) { // 만약 시뮬레이션 로봇이 아닌 경우
    // 로봇에 연결하고 ROS 구독 설정
    if (port_name.find("can") != std::string::npos) { // 포트 이름에 "can" 문자열이 포함되어 있는지 확인합니다.
        robot->Connect(port_name); // 로봇에 연결합니다.
        robot->EnableCommandedMode(); // 로봇의 명령 모드를 활성화합니다.
        ROS_INFO("로봇과 통신하기 위해 CAN 버스를 사용합니다.");
    } else {
        // robot->Connect(port_name, 115200); // 다른 통신 인터페이스를 지원하지 않음
        ROS_INFO("현재 CAN 버스 인터페이스만 지원됩니다.");
    }
}

messenger.SetupSubscription(); // ROS 구독 설정을 수행합니다.

// 로봇 상태를 50Hz로 게시하면서 트위스트 명령을 수신합니다.
ros::Rate rate(50); // 50Hz 주기로 반복하는 루프를 설정합니다.
while (ros::ok()) { // ROS가 정상 작동하는 동안 반복합니다.
    if (!messenger.simulated_robot_) { // 시뮬레이션 로봇이 아닌 경우
        messenger.PublishStateToROS(); // 로봇 상태를 ROS로 게시합니다.
    } else { // 시뮬레이션 로봇인 경우
        double linear, angular;
        messenger.GetCurrentMotionCmdForSim(linear, angular); // 시뮬레이션을 위한 현재 모션 명령을 가져옵니다.
        messenger.PublishSimStateToROS(linear, angular); // 시뮬레이션 로봇 상태를 ROS로 게시합니다.
    }
    ros::spinOnce(); // ROS 콜백 함수를 실행합니다.
    rate.sleep(); // 주기를 유지하기 위해 대기합니다.
}

return 0; // 프로그램을 종료합니다.
}
