#include "ins_rfr.hpp"

hw_rfr::CompStatusPackage unique_comp_status_pkg;
hw_rfr::TeamEventPackage unique_team_event_pkg;
hw_rfr::RobotPerformancePackage unique_robot_performance_pkg;
hw_rfr::RobotPosPackage unique_robot_pos_pkg;
hw_rfr::RobotResourcePackage unique_robot_resource_pkg;
hw_rfr::RobotRfidPackage unique_robot_rfid_pkg;
hw_rfr::RobotsGroundPosPackage unique_robots_ground_pos_pkg;
hw_rfr::RobotSentryDecisionPackage unique_robot_sentry_decision_pkg;
hw_rfr::CompRobotsHpPackage unique_comp_robots_hp_pkg;
hw_rfr::InterMapClientToRobotPackage unique_inter_map_client_to_robot_pkg;
hw_rfr::RobotPowerHeatPackage unique_robot_power_heat_pkg;
hw_rfr::RobotShooterPackage unique_robot_shooter_pkg;
hw_rfr::RobotHurtPackage unique_robot_hurt_pkg;
hw_rfr::InterRadarDetectionPackage unique_inter_radar_detection_pkg;
hw_rfr::RobotBuffPackage unique_robot_buff_pkg;

hw_rfr::InterSentryCmd unique_inter_sentry_cmd;
hw_rfr::InterSentryDetectionPackage unique_inter_sentry_detection_pkg;
hw_rfr::InterRobotTrajPackage unique_inter_robot_traj_pkg;

hw_rfr::Referee unique_referee;
bool is_rfr_inited = false;

hw_rfr::CompStatusPackage* GetRfrCompStatusPkgIns(void) { return &unique_comp_status_pkg; }
hw_rfr::TeamEventPackage* GetRfrTeamEventPkgIns(void) { return &unique_team_event_pkg; }
hw_rfr::RobotPerformancePackage* GetRfrRobotPerformancePkgIns(void) { return &unique_robot_performance_pkg; }
hw_rfr::RobotPosPackage* GetRfrRobotPosPkgIns(void) { return &unique_robot_pos_pkg; }
hw_rfr::RobotResourcePackage* GetRfrRobotResourcePkgIns(void) { return &unique_robot_resource_pkg; }
hw_rfr::RobotRfidPackage* GetRfrRobotRfidPkgIns(void) { return &unique_robot_rfid_pkg; }
hw_rfr::RobotsGroundPosPackage* GetRfrRobotsGroundPosPkgIns(void) { return &unique_robots_ground_pos_pkg; }
hw_rfr::RobotSentryDecisionPackage* GetRfrRobotSentryDecisionPkgIns(void) { return &unique_robot_sentry_decision_pkg; }
hw_rfr::CompRobotsHpPackage* GetRfrCompRobotsHpPkgIns(void) { return &unique_comp_robots_hp_pkg; }
hw_rfr::InterMapClientToRobotPackage* GetRfrInterMapClientToRobotPkgIns(void) { return &unique_inter_map_client_to_robot_pkg; }
hw_rfr::RobotPowerHeatPackage* GetRfrRobotPowerHeatPkgIns(void) { return &unique_robot_power_heat_pkg; }
hw_rfr::RobotShooterPackage* GetRfrRobotShooterPkgIns(void) { return &unique_robot_shooter_pkg; }
hw_rfr::RobotHurtPackage* GetRfrRobotHurtPkgIns(void) { return &unique_robot_hurt_pkg; }
hw_rfr::InterRadarDetectionPackage* GetRfrInterRadarDetectionPkgIns(void) { return &unique_inter_radar_detection_pkg; }
hw_rfr::RobotBuffPackage* GetRfrRobotBuffPkgIns(void) { return &unique_robot_buff_pkg; }

hw_rfr::InterSentryCmd* GetRfrInterSentryCmdIns(void) { return &unique_inter_sentry_cmd; }
hw_rfr::InterSentryDetectionPackage* GetRfrInterSentryDetectionPkgIns(void) { return &unique_inter_sentry_detection_pkg; }
hw_rfr::InterRobotTrajPackage* GetRfrInterRobotTrajPkgIns(void) { return &unique_inter_robot_traj_pkg; }

hw_rfr::Referee* GetRfrIns(void)
{
    if (!is_rfr_inited)
    {
        unique_referee.appendRxPkg(GetRfrCompStatusPkgIns());
        unique_referee.appendRxPkg(GetRfrTeamEventPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotPerformancePkgIns());
        unique_referee.appendRxPkg(GetRfrRobotPosPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotResourcePkgIns());
        unique_referee.appendRxPkg(GetRfrRobotRfidPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotsGroundPosPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotSentryDecisionPkgIns());
        unique_referee.appendRxPkg(GetRfrCompRobotsHpPkgIns());
        unique_referee.appendRxPkg(GetRfrInterMapClientToRobotPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotPowerHeatPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotShooterPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotHurtPkgIns());
        unique_referee.appendRxPkg(GetRfrInterRadarDetectionPkgIns());
        unique_referee.appendRxPkg(GetRfrRobotBuffPkgIns());
        is_rfr_inited = true;
    }
    return &unique_referee;
}