#include "ins_motor.hpp"

/* chassis wheel */
const hw_motor::OptionalParams kMotorParamsWheelLeftFront = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegInfToPosInf,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = true,
    .angle_offset = 0.0f,
    .ex_redu_rat = 268.0f / 17.0f,
};

const hw_motor::OptionalParams kMotorParamsWheelLeftRear = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegInfToPosInf,
    .dir = hw_motor::Dir::kDirRev,
    .remove_build_in_reducer = true,
    .angle_offset = 0.0f,
    .ex_redu_rat = 268.0f / 17.0f,
};

const hw_motor::OptionalParams kMotorParamsWheelRightRear = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegInfToPosInf,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = true,
    .angle_offset = 0.0f,
    .ex_redu_rat = 268.0f / 17.0f,
};

const hw_motor::OptionalParams kMotorParamsWheelRightFront = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegInfToPosInf,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = true,
    .angle_offset = 0.0f,
    .ex_redu_rat = 268.0f / 17.0f,
};


/* gimbal */
const hw_motor::OptionalParams kMotorParamsGimbalPitch = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = false,
    .angle_offset = 0.0785f,
    .ex_redu_rat = 1.0f,
};

const hw_motor::OptionalParams kMotorParamsGimbalYaw = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = false,
    .angle_offset = 0.3286f,
    .ex_redu_rat = 1.0f,
};

const hw_motor::OptionalParams kMotorParamsGimbalRoll = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = false,
    .angle_offset = 0.0f,
    .ex_redu_rat = 1.0f,
};

/* shooter */
const hw_motor::OptionalParams kMotorParamsShooterFeed = {
    .input_type = hw_motor::InputType::kTorq,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::Dir::kDirFwd,
    .remove_build_in_reducer = false,
    .angle_offset = 0.0f,
    .ex_redu_rat = 1.0f,
};

const hw_motor::OptionalParams KMotorParamsShooterFricLeft = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirRev,
    .remove_build_in_reducer = true,
    .angle_offset = 0,
    .ex_redu_rat = 1,
};

const hw_motor::OptionalParams KMotorParamsShooterFricRight = {
    .input_type = hw_motor::InputType::kRaw,
    .angle_range = hw_motor::AngleRange::kNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    .remove_build_in_reducer = true,
    .angle_offset = 0,
    .ex_redu_rat = 1,
};

enum MotorID {
  kMotorIdWheelLeftFront = 2U,
  kMotorIdWheelLeftRear = 3U,
  kMotorIdWheelRightRear = 4U,
  kMotorIdWheelRightFront = 1U,
  kMotorIdGimbalPitch = 2U,
  kMotorIdGimbalYaw = 3U,
  kMotorIdGimbalRoll = 4U,
  kMotorIdShooterFeed = 3U,
  kMotorIdShooterFricLeft = 1U,
  kMotorIdShooterFricRight = 2U,
};

/* chassis wheel */
static hw_motor::M3508 unique_motor_wheel_left_front = hw_motor::M3508(kMotorIdWheelLeftFront, kMotorParamsWheelLeftFront);
static hw_motor::M3508 unique_motor_wheel_left_rear = hw_motor::M3508(kMotorIdWheelLeftRear, kMotorParamsWheelLeftRear);
static hw_motor::M3508 unique_motor_wheel_right_rear = hw_motor::M3508(kMotorIdWheelRightRear, kMotorParamsWheelRightRear);
static hw_motor::M3508 unique_motor_wheel_right_front = hw_motor::M3508(kMotorIdWheelRightFront, kMotorParamsWheelRightFront);

/* gimbal */
static hw_motor::DM_J4310 unique_motor_gimbal_pitch = hw_motor::DM_J4310(kMotorIdGimbalPitch, kMotorParamsGimbalPitch);
static hw_motor::DM_J4310 unique_motor_gimbal_yaw = hw_motor::DM_J4310(kMotorIdGimbalYaw, kMotorParamsGimbalYaw);
static hw_motor::DM_J4340 unique_motor_gimbal_roll = hw_motor::DM_J4340(kMotorIdGimbalRoll, kMotorParamsGimbalRoll);

/* shooter */
static hw_motor::M2006 unique_motor_shooter_feed = hw_motor::M2006(kMotorIdShooterFeed, kMotorParamsShooterFeed);
static hw_motor::M3508 unique_motor_shooter_fric_left = hw_motor::M3508(kMotorIdShooterFricLeft, KMotorParamsShooterFricLeft);
static hw_motor::M3508 unique_motor_shooter_fric_right = hw_motor::M3508(kMotorIdShooterFricRight, KMotorParamsShooterFricRight);

/* chassis wheel */
hw_motor::Motor* GetMotorWheelLeftFrontIns(void) { return &unique_motor_wheel_left_front; }
hw_motor::Motor* GetMotorWheelLeftRearIns(void) { return &unique_motor_wheel_left_rear; }
hw_motor::Motor* GetMotorWheelRightFrontIns(void) { return &unique_motor_wheel_right_front; }
hw_motor::Motor* GetMotorWheelRightRearIns(void) { return &unique_motor_wheel_right_rear; }

/* gimbal */
hw_motor::DaMiao* GetMotorGimbalPitchIns(void) { return &unique_motor_gimbal_pitch; }
hw_motor::DaMiao* GetMotorGimbalYawIns(void) { return &unique_motor_gimbal_yaw; }
hw_motor::DaMiao* GetMotorGimbalRollIns(void) { return &unique_motor_gimbal_roll; }

/* shooter */
hw_motor::Motor* GetMotorShooterFeedIns(void) { return &unique_motor_shooter_feed; }
hw_motor::Motor* GetMotorShooterFricLeftIns(void) { return &unique_motor_shooter_fric_left; }
hw_motor::Motor* GetMotorShooterFricRightIns(void) { return &unique_motor_shooter_fric_right; }