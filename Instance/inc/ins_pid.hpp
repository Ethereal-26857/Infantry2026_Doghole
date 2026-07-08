#ifndef INS_PID_HPP_
#define INS_PID_HPP_

#include "pid.hpp"

namespace hw_pid = hello_world::pid;

/* chassis wheel */
hw_pid::MultiNodesPid* GetPidMotorWheelLeftFrontIns(void);
hw_pid::MultiNodesPid* GetPidMotorWheelLeftRearIns(void);
hw_pid::MultiNodesPid* GetPidMotorWheelRightRearIns(void);
hw_pid::MultiNodesPid* GetPidMotorWheelRightFrontIns(void);

hw_pid::MultiNodesPid* GetPidFollowOmegaIns(void);
hw_pid::MultiNodesPid* GetPidFollowSpeedOmegaIns(void);
hw_pid::MultiNodesPid* GetPidGyroIns(void);

/* gimbal */
hw_pid::MultiNodesPid* GetPidMotorGimbalPitchIns(void);
hw_pid::MultiNodesPid* GetPidMotorGimbalYawIns(void);
hw_pid::MultiNodesPid* GetPidMotorGimbalRollIns(void);

/* shooter */
hw_pid::MultiNodesPid* GetPidMotorShooterFeedIns(void);
hw_pid::MultiNodesPid *GetPidMotorShooterLeftFricIns(void);
hw_pid::MultiNodesPid *GetPidMotorShooterRightFricIns(void);
hw_pid::MultiNodesPid *GetPidFricSpeedSynIns(void);

#endif /* INS_PID_HPP_ */