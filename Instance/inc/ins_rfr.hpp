#ifndef INS_RFR_HPP_
#define INS_RFR_HPP_

#include "referee.hpp"
#include "rfr_official_pkgs.hpp"

namespace hw_rfr = hello_world::referee;

hw_rfr::CompStatusPackage* GetRfrCompStatusPkgIns(void);
hw_rfr::TeamEventPackage* GetRfrTeamEventPkgIns(void);
hw_rfr::RobotPerformancePackage* GetRfrRobotPerformancePkgIns(void);
hw_rfr::RobotPosPackage* GetRfrRobotPosPkgIns(void);
hw_rfr::RobotResourcePackage* GetRfrRobotResourcePkgIns(void);
hw_rfr::RobotRfidPackage* GetRfrRobotRfidPkgIns(void);
hw_rfr::RobotsGroundPosPackage* GetRfrRobotsGroundPosPkgIns(void);
hw_rfr::RobotSentryDecisionPackage* GetRfrRobotSentryDecisionPkgIns(void);
hw_rfr::CompRobotsHpPackage* GetRfrCompRobotsHpPkgIns(void);
hw_rfr::InterMapClientToRobotPackage* GetRfrInterMapClientToRobotPkgIns(void);
hw_rfr::RobotPowerHeatPackage* GetRfrRobotPowerHeatPkgIns(void);
hw_rfr::RobotShooterPackage* GetRfrRobotShooterPkgIns(void);
hw_rfr::RobotHurtPackage* GetRfrRobotHurtPkgIns(void);
hw_rfr::InterRadarDetectionPackage* GetRfrInterRadarDetectionPkgIns(void);
hw_rfr::RobotBuffPackage* GetRfrRobotBuffPkgIns(void);

hw_rfr::InterSentryCmd* GetRfrInterSentryCmdIns(void);
hw_rfr::InterSentryDetectionPackage* GetRfrInterSentryDetectionPkgIns(void);
hw_rfr::InterRobotTrajPackage* GetRfrInterRobotTrajPkgIns(void);

hw_rfr::Referee* GetRfrIns(void);

#endif /* INS_RFR_HPP_ */