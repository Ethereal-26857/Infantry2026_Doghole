#include "ins_pid.hpp"

#include "motor.hpp"

const float kMaxPidOutputMottorWheel = 16384.0f;  // 轮电机采用3508

const float kMaxPidOutputFollowOmega = 6.0f;
const float kMaxPidOutputGyro = 10.0f;

const float kMaxPidOutputMottorGimbalPitch = 7.0f;  // 云台 pitch 轴电机采用DM-J4310
const float kMaxPidOutputMottorGimbalYaw = 7.0f;    // 云台 yaw 轴电机采用DM-J4310
const float kMaxPidOutputMottorGimbalRoll = 28.0f;  // 云台 roll 轴电机采用DM-J4340

const float kMaxPidOutputShooterFeed = 2.5f;
const float kMaxPidOutputShooterFric = 16384.0f;
const float kMaxPidOutputFricSpeedSyn = 100.0f;

const hw_pid::OutLimit kOutLimitMotorWheel = hw_pid::OutLimit(true, -kMaxPidOutputMottorWheel, kMaxPidOutputMottorWheel);

const hw_pid::OutLimit kOutLimitFollowOmega = hw_pid::OutLimit(true, -kMaxPidOutputFollowOmega, kMaxPidOutputFollowOmega);
const hw_pid::OutLimit kOutLimitGyro = hw_pid::OutLimit(true, 0.0f, kMaxPidOutputGyro);

const hw_pid::OutLimit kOutLimitMotorGimbalPitch = hw_pid::OutLimit(true, -kMaxPidOutputMottorGimbalPitch, kMaxPidOutputMottorGimbalPitch);
const hw_pid::OutLimit kOutLimitMotorGimbalYaw = hw_pid::OutLimit(true, -kMaxPidOutputMottorGimbalYaw, kMaxPidOutputMottorGimbalYaw);
const hw_pid::OutLimit kOutLimitMotorGimbalRoll = hw_pid::OutLimit(true, -kMaxPidOutputMottorGimbalRoll, kMaxPidOutputMottorGimbalRoll);

const hw_pid::OutLimit kOutLimitShooterFeed = hw_pid::OutLimit(true, -kMaxPidOutputShooterFeed, kMaxPidOutputShooterFeed);
const hw_pid::OutLimit kOutLimitShooterFric = hw_pid::OutLimit(true, -kMaxPidOutputShooterFric, kMaxPidOutputShooterFric);
const hw_pid::OutLimit kOutLimitFricSpeedSyn = hw_pid::OutLimit(true, -kMaxPidOutputFricSpeedSyn, kMaxPidOutputFricSpeedSyn);

/* chassis wheel */
const hw_pid::MultiNodesPid::ParamsList kPidParamsMotorWheel = {
    {
        .auto_reset = true,
        .kp = 1600.0f,
        .ki = 0.0f,
        .kd = 0.0f,

        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(true, -0.1, 0.1, 0.1),
        .dead_band = hw_pid::DeadBand(true, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -40.0f, 40.0f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(true, -10.0f, 10.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(true),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(true, 0.5f),
        .out_limit = kOutLimitMotorWheel,
    },
};

/* chassis wheel follow / gyro */
const hw_pid::MultiNodesPid::ParamsList kPidParamsFollowOmega = {
    {
        .auto_reset = true,
        .kp = 7.3f,
        .ki = 0,
        .kd = 0,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(true, PI * 2.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -3000.0f, 3000.0f),  ///< 积分抗风up优化器
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = kOutLimitFollowOmega,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsFollowSpeedOmega = {
    {
        .auto_reset = true,
        .kp = 9.2,
        .ki = 0,
        .kd = 0,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(true, PI / 2.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -3000.0f, 3000.0f),  ///< 积分抗风up优化器
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = kOutLimitFollowOmega,
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsGyro = {
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -10.0f, 10.0f),  ///< 积分抗风up优化器
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(true),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = kOutLimitGyro,
    },
};

/* gimbal */
const hw_pid::MultiNodesPid::ParamsList kPidParamsMotorGimbalPitch = {
    {
        .auto_reset = true,
        .kp = 19.5f,  // 18
        .ki = 0.00f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.9),
        .dead_band = hw_pid::DeadBand(false, -0.0001f, 0.0001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -1.0f, 1.0f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(true, 0.3f, 0.3f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(true),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = hw_pid::OutLimit(true, -12.0f, 12.0f),
    },
    // {
    //     .auto_reset = true,
    //     .kp = 1.4f,
    //     .ki = 0.0f,
    //     .kd = 0.0f,
    //     .max_interval_ms = 5,
    //     .setpoint_ramping = hw_pid::SetpointRamping(true, -0.1, 0.1, 0.9),
    //     .dead_band = hw_pid::DeadBand(false, -0.2f, 0.2f),
    //     .period_sub = hw_pid::PeriodSub(false, 0.0f),
    //     .inte_anti_windup = hw_pid::InteAntiWindup(false, -1.0f, 1.0f),
    //     .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
    //     .inte_separation = hw_pid::InteSeparation(false, -0.3f, 0.3f),
    //     .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
    //     .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
    //     .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
    //     .out_limit = kOutLimitMotorGimbalPitch,
    // },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsMotorGimbalYaw = {
    {
        .auto_reset = true,
        .kp = 28.5f,  // 24
        .ki = 0.00f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(true, -0.1, 0.1, 0.9),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -1.0f, 1.0f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(true, 0.3f, 0.3f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = hw_pid::OutLimit(true, -22.0f, 22.0f),
    },
    // {
    //     .auto_reset = true,
    //     .kp = 1.8f,
    //     .ki = 0.0f,
    //     .kd = 0.0f,
    //     .max_interval_ms = 5,
    //     .setpoint_ramping = hw_pid::SetpointRamping(true, -0.1, 0.1, 0.9),
    //     .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
    //     .period_sub = hw_pid::PeriodSub(false, 0.0f),
    //     .inte_anti_windup = hw_pid::InteAntiWindup(false, -1.0f, 1.0f),
    //     .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
    //     .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
    //     .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
    //     .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
    //     .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
    //     .out_limit = kOutLimitMotorGimbalYaw,
    // },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsMotorGimbalRoll = {
    {
        .auto_reset = true,
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.9),
        .dead_band = hw_pid::DeadBand(false, -0.0001f, 0.0001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -1.0f, 1.0f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(true, 0.3f, 0.3f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(true),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = hw_pid::OutLimit(true, -28.0f, 28.0f),
    },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsMotorShooterFeed = {
    {
        .auto_reset = false,  ///< 是否自动清零
        .kp = 33.0f,          // 38.0
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(true, 2.0f * PI),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -0.1f, 0.1f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = hw_pid::OutLimit(true, -60.0f, 60.0f),
    },
    {
        .auto_reset = false,  ///< 是否自动清零
        .kp = 1.0f,           // 1.2f
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(true, -0.1f, 0.1f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(true),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(true, 0.5f),
        // .out_limit = hw_pid::OutLimit(true, -3.0f, 3.0f),
        .out_limit = kOutLimitShooterFeed,
    }};

const hw_pid::MultiNodesPid::ParamsList kPidParamsMotorShooterFric = {
    {
        .auto_reset = false,  ///< 是否自动清零
        .kp = 120.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(true, -0.1, 0.1, 0.9),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -0.1f, 0.1f),
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = kOutLimitShooterFric,
    },
};

// 摩擦轮速度同步 PID
// NOTE: 暂时感觉没什么用，可能影响弹道，具体的参数需要在测弹道的时候测试
const hw_pid::MultiNodesPid::ParamsList kPidParamsFricSpeedSyn = {
    {
        .auto_reset = true,  ///< 是否自动清零
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 5,
        .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.9),
        .dead_band = hw_pid::DeadBand(false, -0.001f, 0.001f),
        .period_sub = hw_pid::PeriodSub(false, 0.0f),
        .inte_anti_windup = hw_pid::InteAntiWindup(false, -10.0f, 10.0f),  ///< 积分抗风up优化器
        .inte_changing_rate = hw_pid::InteChangingRate(false, 0.0f, 0.001f),
        .inte_separation = hw_pid::InteSeparation(false, 0.0f, 0.0f),
        .inte_trapezoidal = hw_pid::InteTrapezoidal(false),
        .diff_filter = hw_pid::DiffFilter(false, -0.1f, 0.1f, 0.5f),
        .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
        .out_limit = kOutLimitFricSpeedSyn,
    },
};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kCascade;

/* chassis wheel */
hw_pid::MultiNodesPid unique_pid_motor_wheel_left_front = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorWheel, kPidParamsMotorWheel);
hw_pid::MultiNodesPid unique_pid_motor_wheel_left_rear = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorWheel, kPidParamsMotorWheel);
hw_pid::MultiNodesPid unique_pid_motor_wheel_right_rear = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorWheel, kPidParamsMotorWheel);
hw_pid::MultiNodesPid unique_pid_motor_wheel_right_front = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorWheel, kPidParamsMotorWheel);

hw_pid::MultiNodesPid unique_pid_follow_omega(kPidTypeCascade, kOutLimitFollowOmega, kPidParamsFollowOmega);
hw_pid::MultiNodesPid unique_pid_follow_speed_omega(kPidTypeCascade, kOutLimitFollowOmega, kPidParamsFollowSpeedOmega);
hw_pid::MultiNodesPid unique_pid_gyro(kPidTypeCascade, kOutLimitGyro, kPidParamsGyro);

/* gimbal */
hw_pid::MultiNodesPid unique_pid_motor_gimbal_pitch = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorGimbalPitch, kPidParamsMotorGimbalPitch);
hw_pid::MultiNodesPid unique_pid_motor_gimbal_yaw = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorGimbalYaw, kPidParamsMotorGimbalYaw);
hw_pid::MultiNodesPid unique_pid_motor_gimbal_roll = hw_pid::MultiNodesPid(kPidTypeCascade, kOutLimitMotorGimbalRoll, kPidParamsMotorGimbalRoll);

/* shooter */
hw_pid::MultiNodesPid unique_pid_motor_shooter_feed(kPidTypeCascade, kOutLimitShooterFeed, kPidParamsMotorShooterFeed);
hw_pid::MultiNodesPid unique_pid_motor_shooter_left_fric(kPidTypeCascade, kOutLimitShooterFric, kPidParamsMotorShooterFric);
hw_pid::MultiNodesPid unique_pid_motor_shooter_right_fric(kPidTypeCascade, kOutLimitShooterFric, kPidParamsMotorShooterFric);
hw_pid::MultiNodesPid unique_pid_fric_speed_syn(kPidTypeCascade, kOutLimitFricSpeedSyn, kPidParamsFricSpeedSyn);

/* chassis wheel */
hw_pid::MultiNodesPid *GetPidMotorWheelLeftFrontIns() { return &unique_pid_motor_wheel_left_front; }
hw_pid::MultiNodesPid *GetPidMotorWheelLeftRearIns() { return &unique_pid_motor_wheel_left_rear; }
hw_pid::MultiNodesPid *GetPidMotorWheelRightRearIns() { return &unique_pid_motor_wheel_right_rear; }
hw_pid::MultiNodesPid *GetPidMotorWheelRightFrontIns() { return &unique_pid_motor_wheel_right_front; }

hw_pid::MultiNodesPid *GetPidFollowOmegaIns() { return &unique_pid_follow_omega; };
hw_pid::MultiNodesPid *GetPidFollowSpeedOmegaIns() { return &unique_pid_follow_speed_omega; };
hw_pid::MultiNodesPid *GetPidGyroIns() { return &unique_pid_gyro; };

/* gimbal */
hw_pid::MultiNodesPid *GetPidMotorGimbalPitchIns() { return &unique_pid_motor_gimbal_pitch; }
hw_pid::MultiNodesPid *GetPidMotorGimbalYawIns() { return &unique_pid_motor_gimbal_yaw; }
hw_pid::MultiNodesPid *GetPidMotorGimbalRollIns() { return &unique_pid_motor_gimbal_roll; }

/* shooter */
hw_pid::MultiNodesPid *GetPidMotorShooterFeedIns() { return &unique_pid_motor_shooter_feed; }
hw_pid::MultiNodesPid *GetPidMotorShooterLeftFricIns() { return &unique_pid_motor_shooter_left_fric; }
hw_pid::MultiNodesPid *GetPidMotorShooterRightFricIns() { return &unique_pid_motor_shooter_right_fric; }
hw_pid::MultiNodesPid *GetPidFricSpeedSynIns() { return &unique_pid_fric_speed_syn; }