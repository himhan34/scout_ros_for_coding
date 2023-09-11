/* 
 * scout_params.hpp
 * 
 * Created on: Sep 27, 2019 15:08
 * Description: 스카우트 로봇의 매개변수 정의
 * 
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef SCOUT_PARAMS_HPP
#define SCOUT_PARAMS_HPP

namespace westonrobot
{
    struct ScoutParams
    {
        /* 스카우트 로봇 매개변수 */
        static constexpr double max_steer_angle = 30.0; // 각도로 표시된 최대 스티어링 각도

        static constexpr double track = 0.58306;      // 미터 단위 (왼쪽 및 오른쪽 바퀴 간 거리)
        static constexpr double wheelbase = 0.498;    // 미터 단위 (전방 및 후방 바퀴 간 거리)
        static constexpr double wheel_radius = 0.165; // 미터 단위의 바퀴 반지름

        // 사용자 매뉴얼 v1.2.8 P18에서 가져옴
        // 최대 선속도: 1.5 m/s
        // 최대 각속도: 0.7853 rad/s
        static constexpr double max_linear_speed = 1.5;     // 미터/초로 표시된 최대 선형 속도
        static constexpr double max_angular_speed = 0.7853; // 라디안/초로 표시된 최대 각속도
        static constexpr double max_speed_cmd = 10.0;       // rad/s로 표시된 최대 속도 명령
    };
} // namespace westonrobot

#endif /* SCOUT_PARAMS_HPP */
