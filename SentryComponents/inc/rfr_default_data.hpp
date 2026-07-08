#ifndef SENTRY_COMPONENTS_RFR_DEFAULT_DATA_HPP_
#define SENTRY_COMPONENTS_RFR_DEFAULT_DATA_HPP_

#include "referee.hpp"
#include "rfr_official_pkgs.hpp"

namespace robot
{
    const hello_world::referee::CompStatusData kDefaultCompStatusData = {
        .game_type = 0,            // 未知类型比赛
        .game_progress = 4,        // 比赛中
        .stage_remain_time = 420,
        .sync_time_stamp = 0,
    };

    const hello_world::referee::TeamEventData kDefaultTeamEventData = {
        .restoration = 0,
        .reserved1 = 0,
        .supplier_area = 0,
        .small_power_rune = 0,
        .large_power_rune = 0,
        .central_highland = 0,
        .trapezoid_highland = 0,
        .dart_last_hit_us_time = 0,
        .dart_last_hit_us_type = 0,
        .center_buff_area = 0,
        .fortress_buff_area = 0,
        .outpost_buff_area = 0,
        .base_buff_area = 0,
        .reserved2 = 0,
    };

    const hello_world::referee::RobotBuffData kDefaultRobotBuffData = {
        .recovery_buff = 0,
        .cooling_buff = 0,
        .defence_buff = 0,
        .vulnerability_buff = 0,
        .attack_buff = 0,
        .remaining_energy = 0x31, // 默认剩余能量大于等于 50%
    };

    const hello_world::referee::RobotPerformanceData kDefaultRobotPerformanceData = {
        .robot_id = 7,                          // 红方哨兵机器人
        .robot_level = 10,
        .current_hp = 400,
        .maximum_hp = 400,
        .shooter_barrel_cooling_value = 80,
        .shooter_barrel_heat_limit = 400,
        .chassis_power_limit = 100,
        .power_management_gimbal_output = 1,
        .power_management_chassis_output = 1,
        .power_management_shooter_output = 1,
        .reserved = 0,
    };

    const hello_world::referee::RobotPosData kDefaultRobotPosData = {                        // 红方哨兵机器人
        .x = 0,
        .y = 0,
        .angle = 0,
    };

    const hello_world::referee::RobotResourceData kDefaultRobotResourceData = {
        .allowence_17mm = 300,          // 初始允许发弹量
        .allowence_42mm = 0,
        .remaining_coin = 3000,
        .fortress_allowence_17mm = 0,
    };

    const hello_world::referee::RobotRfidData kDefaultRobotRfidData = {
        .our_base = 0,
        .our_central_highland = 0,
        .opp_central_highland = 0,
        .our_trapezoid_highland = 0,
        .opp_trapezoid_highland = 0,
        .our_launch_front = 0,
        .our_launch_back = 0,
        .opp_launch_front = 0,
        .opp_launch_back = 0,
        .our_highland_bottom = 0,
        .our_highland_top = 0,
        .opp_highland_bottom = 0,
        .opp_highland_top = 0,
        .our_highway_bottom = 0,
        .our_highway_top = 0,
        .opp_highway_bottom = 0,
        .opp_highway_top = 0,
        .our_fortress = 0,
        .our_outpost = 0,
        .our_resource_1 = 0,
        .our_resource_2 = 0,
        .our_assembly_zone = 0,
        .opp_assembly_zone = 0,
        .central_boost = 0,
        .opp_fortress = 0,
        .opp_outpost = 0,
        .our_highway_tunnel_bottom = 0,
        .our_highway_tunnel_mid = 0,
        .our_highway_tunnel_top = 0,
        .our_highland_tunnel_bottom = 0,
        .our_highland_tunnel_mid = 0,
        .our_highland_tunnel_top = 0,
        .opp_highway_tunnel_bottom = 0,
        .opp_highway_tunnel_mid = 0,
        .opp_highway_tunnel_top = 0,
        .opp_highland_tunnel_bottom = 0,
        .opp_highland_tunnel_mid = 0,
        .opp_highland_tunnel_top = 0,
        .reserved = 0,
    };

    const hello_world::referee::RobotsGroundPosData kDefaultRobotsGroundPosData = 
    {
        .hero_x = 0,
        .hero_y = 0,
        .engineer_x = 0,
        .engineer_y = 0,
        .standard_3_x = 0,
        .standard_3_y = 0,
        .standard_4_x = 0,
        .standard_4_y = 0,
        .reserved_1 = 0,
        .reserved_2 = 0,
    };

    const hello_world::referee::RobotSentryDecisionData kDefaultRobotSentryDecisionData =
    {
        .allowance = 0,
        .remote_allowance = 0,
        .remote_hp = 0,
        .allow_free_resurrection = 0,
        .allow_redemption_resurrection = 0,
        .redemption_resurrection_cost = 1023,      // 设置一个很大的值等同认为无法直接复活
        .reserved1 = 0,
        .out_of_combat = 1,                         // 默认处于脱战状态
        .team_17mm_allowance = 0,
        .current_mode = 0,
        .allow_activate_power_rune = 0,
        .reserved2 = 0,
    };

    const hello_world::referee::CompRobotsHpData kDefaultCompRobotsHpData =
    {
        .ally_1_robot_HP = 200,
        .ally_2_robot_HP = 250,
        .ally_3_robot_HP = 150,
        .ally_4_robot_HP = 150,
        .reserved_1 = 0,
        .ally_7_robot_HP = 400,
        .ally_outpost_HP = 1500,
        .ally_base_HP = 5000,
    };

    const hello_world::referee::InterMapClientToRobotData kDefaultInterMapClientToRobotData = 
    {
        .target_position_x = 0,
        .target_position_y = 0,
        .cmd_keyboard = 0,
        .target_robot_id = 0,
        .cmd_source = 0,
    };

    const hello_world::referee::RobotPowerHeatData kDefaultRobotPowerHeatData =
    {
        // V1.7.0裁判系统不再读的到底盘电压、电流
        .reserved_1 = 0,
        .reserved_2 = 0,
        .reserved_3 = 0,
        .buffer_energy = 20,
        .shooter_17mm_1_barrel_heat = 0,
        .shooter_42mm_barrel_heat = 0,
    };

    const hello_world::referee::RobotShooterData kDefaultRobotShooterData = 
    {
        .bullet_type = 1u,
        .shooter_id = 1u,
        .launching_frequency = 0,
        .bullet_speed = 24.5f,
    };

    const hello_world::referee::RobotHurtData kDefaultRobotHurtData = 
    {
        .module_id = 0,
        .hp_deduction_reason = 2,
    };
    
    const hello_world::referee::InterSentryCmdData kDefaultInterSentryCmdData =
    {
        .is_to_revive = 1,
        .is_to_instant_revive = 1,
        .num_exchanging_projectiles = 0,
        .num_requests_remote_proj = 0,
        .num_requests_remote_hp = 0,
        .set_mode = 3,
        .set_activate_power_rune = 0,
        .reserved = 0,
    };

    const hello_world::referee::InterSentryDetectionData kDefaultInterSentryDetectionData = 
    {
        .stage_remain_time = 0,
        .flag = 0,
        .red_hero_x = 0,
        .red_hero_y = 0,
        .red_engineer_x = 0,
        .red_engineer_y = 0,
        .red_standard_3_x = 0,
        .red_standard_3_y = 0,
        .red_standard_4_x = 0,
        .red_standard_4_y = 0,
        .red_sentry_x = 0,
        .red_sentry_y = 0,
        .blue_hero_x = 0,
        .blue_hero_y = 0,
        .blue_engineer_x = 0,
        .blue_engineer_y = 0,
        .blue_standard_3_x = 0,
        .blue_standard_3_y = 0,
        .blue_standard_4_x = 0,
        .blue_standard_4_y = 0,
        .blue_sentry_x = 3,
        .blue_sentry_y = 0,
        // .other_robot_1_x = 0,
        // .other_robot_1_y = 0,
        // .other_robot_2_x = 0,
        // .other_robot_2_y = 0,
        // .other_robot_3_x = 0,
        // .other_robot_3_y = 0,
        // .other_robot_4_x = 0,
        // .other_robot_4_y = 0,
        // .other_robot_5_x = 0,
        // .other_robot_5_y = 0,
    };

    const hello_world::referee::InterRadarDetectionData kDefaultInterRadarDetectionData =
    {
        .hero_x = 0,
        .hero_y = 0,
        .engineer_x = 0,
        .engineer_y = 0,
        .standard_3_x = 0,
        .standard_3_y = 0,
        .standard_4_x = 0,
        .standard_4_y = 0,
        .sentry_x = 0,
        .sentry_y = 0,
        .outpost_status = 0,
    };
} // namespace robot

#endif /* SENTRY_COMPONENTS_RFR_DEFAULT_DATA_HPP_ */