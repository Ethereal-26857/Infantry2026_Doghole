#ifndef INSTANCE_INS_MOTOR_HPP_
#define INSTANCE_INS_MOTOR_HPP_

#include "motor.hpp"

namespace hw_motor = hello_world::motor;

/* chassis wheel */
hw_motor::Motor* GetMotorWheelLeftFrontIns(void);
hw_motor::Motor* GetMotorWheelLeftRearIns(void);
hw_motor::Motor* GetMotorWheelRightRearIns(void);
hw_motor::Motor* GetMotorWheelRightFrontIns(void);

/* gimbal */
hw_motor::DaMiao* GetMotorGimbalPitchIns(void);
hw_motor::DaMiao* GetMotorGimbalYawIns(void);
hw_motor::DaMiao* GetMotorGimbalRollIns(void);

/* shooter */
hw_motor::Motor* GetMotorShooterFeedIns(void);
hw_motor::Motor* GetMotorShooterFricLeftIns(void);
hw_motor::Motor* GetMotorShooterFricRightIns(void);

#endif /* INSTANCE_INS_MOTOR_HPP_ */