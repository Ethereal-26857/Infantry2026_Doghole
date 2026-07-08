#ifndef INS_ROBOT_HPP_
#define INS_ROBOT_HPP_

#include "robot.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "feed.hpp"
#include "fric.hpp"

namespace hw_module = hello_world::module;

robot::Robot* GetRobotIns(void);

robot::Chassis* GetChassisIns(void);
robot::Gimbal* GetGimbalIns(void);

hw_module::Feed* GetFeedIns(void);
hw_module::Fric* GetFricIns(void);

#endif /* INS_ROBOT_HPP_ */