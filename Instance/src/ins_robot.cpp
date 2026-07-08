#include "ins_all.hpp"

const robot::Chassis::Config kChassisConfig = {
    .norm_trans_vel = 3.0f,                  ///< 正常模式下(超电没开启）最大平移速度，单位：m/s
    .norm_rot_spd = 8.0f,                    ///< 正常模式下（超电没开启）最大旋转速度，单位: rad/s
    .max_trans_vel = 3.0f,                   ///< 最大行驶速度，单位：m/s
    .max_rot_spd = 8.0f,                     ///< 最大旋转速度，单位：rad/s
    .min_follow_spd = 0.5f,                  ///< 最小随动速度，单位：m/s
    .ffd_kp_gyro = 0.0188f,                  ///< 小陀螺前馈增益，单位：无
    .ffd_kp_vel = 0.001f,                    ///< 平移速度前馈增益，单位：无
    .real_intilthole_ang = 0.486145430783f,  // 斜狗洞实际底盘朝向角，单位：rad
    .real_insgthole_ang = 0.0f,              // 直狗洞实际底盘朝向角，单位：rad
};

const robot::Gimbal::Config kGimbalConfig = {
    .GimbalPitch = {
        .pid_spd_kp = 2.1,             ///< 速度环kp
        .sensitivity_normal = 0.002f,  ///< Pitch轴手动模式下的灵敏度，单位：rad/ms
        .sensitivity_search = 0.001f,  ///< Pitch轴巡航模式下的灵敏度，单位：rad/ms
        .max_ang_motor = 0.80f,        ///< Pitch轴机械最大角度，单位：rad
        .min_ang_motor = -0.224f,      ///< Pitch轴机械最小角度，单位：rad
        .max_ang_outpost = 0.0f,       ///< Pitch轴前哨模式下的最大角度，单位：rad
        .min_ang_outpost = -0.0f,      ///< Pitch轴前哨模式下的最小角度，单位：rad
        .max_ang_search = 0.00f,       ///< Pitch轴巡逻模式下的最大角度，单位：rad
        .min_ang_search = -0.18f,      ///< Pitch轴巡逻模式下的最小角度，单位：rad
        .fix_ang_search = -2.0f,       ///< 云台巡逻模式下的固定俯仰角度，单位：Deg
        .fix_ang_outpost = 25.0f,      ///< 云台前哨站模式下的固定俯仰角度，单位：Deg
    },
    .GimbalYaw = {
        .pid_spd_kp = 1.8,             ///< 速度环kp
        .sensitivity_normal = 0.003f,  ///< Yaw轴手动模式下的灵敏度，单位：rad/ms
        .sensitivity_search = 0.002f,  ///< Yaw轴巡航模式下的灵敏度，单位：rad/ms
        .max_ang_motor = 0.72f,        ///< Yaw轴机械最大角度，单位：rad
        .min_ang_motor = -0.588f,      ///< Yaw轴机械最小角度，单位：rad
        .max_ang_outpost = 0.68f,      ///< Yaw轴前哨模式下的最大角度，单位：rad
        .min_ang_outpost = -0.58f,     ///< Yaw轴前哨模式下的最小角度，单位：rad
        .max_ang_search = 0.68f,       ///< Yaw轴巡逻模式下的最大角度，单位：rad
        .min_ang_search = -0.58f,      ///< Yaw轴巡逻模式下的最小角度，单位：rad
    },
};

const hw_module::Feed::Config kFeedConfig = {
    .ang_ref_offset = PI / 15,  // 15  //12
    .ang_per_blt = PI / 5,
    .heat_per_blt = 10.0f,
    //.stuck_curr_thre = 10.0f,
    .resurrection_pos_err = 5.0 / 180 * PI,
    // .stuck_duration_thre = 200,
    // .hold_duration_thre = 100,
    .default_trigger_interval = 33,
    .default_safe_num_blt = 5.0f,
    .stuck_detection =
        {
            // 暂时仍旧使用电流判断，晚点尝试角度判断
            .detection_by = hw_module::feed_impl::FeedStuckDetectionBy::kAngle,
            // .ang_hold_duration_thre = 100,
            .ang_diff_thre = 0.1,
            // .ang_beyond_thre = 4.0 / 180.0 * PI,
            .curr_thre = 9.0f,
            .time_thre = 100,
        },
    // .opt_prevent_stuck =
    // {
    //     // 这个是只建议42mm大弹丸开启的
    //     .is_prevent_stuck = false,
    //     .back_ang = 0,
    // }
    // .stuck_rollback = {
    //     .rollback_after_trigger = false,
    //     .back_ang = 24.0f / 180.0f * PI,
    // }
    .stuck_handle =
        {
            .method = hw_module::feed_impl::FeedStuckBackMethod::kFixedAng,
            .back_ang = PI / 5 * 2,
        },
    .stuck_prevent =
        {
            .enabled = false,
            .back_ang = 0.0f,
        },
};

const hw_module::Fric::Config kFricConfig = {
    .fric_num = 2,                        ///< 摩擦轮数量, >=2, <=10, 无默认值
    .default_spd_ref = 610.0f,            ///< 摩擦轮期望速度预设值, >0, 无默认值, rad/s
    .default_spd_ref_backward = -100.0f,  ///< 摩擦轮反转目标速度, <0, 默认值 -100 rad/s, 反转模式是为了将卡在摩擦轮中间的弹丸回退出来，转速不易过快
    .stuck_curr_thre = 30.0f,             ///< 用于判断摩擦轮堵转的电流阈值, >0, 默认值 14 A
    .spd_delta_thre = 5.0f,               ///< 用于判断摩擦轮速度保持恒定的阈值, >0, 默认值 10 rad/s
    .spd_err_thre = 5.0f,                 ///< 用于判断摩擦轮速度跟上期望转速的阈值, >0, 默认值 5 rad/s
    .spd_stop_thre = 100.0f,              ///< 摩擦轮Stop模式，转速小于该阈值后，停止控制电机, >0, 默认值 100 rad/s
    .resurrection_duration = 1,           ///< 复活模式持续时间, >0, 默认值 2000 ms
    .switch_input_thre = 10000.0f,           ///< 启动和急停时的控制输入阈值，>0 ，无默认值
    .opt_spd_same_pid_enabled = false,    ///< 是否使用双摩擦轮同速PID(期望为0，反馈输入为两轮差速，输出分别正负作用到两个电机上)
    .opt_blt_spd_cl = {
        .is_enabled = true,               ///< 是否开启弹速闭环
        .min_reasonable_blt_spd = 18.0f,  ///< 最小合理弹丸速度, >0, 无默认值, m/s, 小于该值认为裁判系统反馈数据错误
        .max_reasonable_blt_spd = 28.0f,  ///< 最大合理弹丸速度, >0, 无默认值, m/s, 大于该值认为裁判系统反馈数据错误
        .min_target_blt_spd = 22.60f,     ///< 弹丸速度期望值区间下限, >0, 无默认值, m/s
        .max_target_blt_spd = 23.10f,     ///< 弹丸速度期望值区间上限, >0, 无默认值, m/s
        .spd_gradient = 5.0f,             ///< 摩擦轮转速调整梯度, >=0, 默认值 5 rad/s
    }};

static bool is_robot_inited = false;
robot::Robot unique_robot = robot::Robot();

static bool is_chassis_inited = false;
robot::Chassis unique_chassis = robot::Chassis(kChassisConfig);

static bool is_gimbal_inited = false;
robot::Gimbal unique_gimbal = robot::Gimbal(kGimbalConfig);

static bool is_feed_inited = false;
hw_module::Feed unique_feed = hw_module::Feed(kFeedConfig);

static bool is_fric_inited = false;
hw_module::Fric unique_fric = hw_module::Fric(kFricConfig);

robot::Robot *GetRobotIns(void)
{
  if (!is_robot_inited) {
    // main 组件指针注册
    // 主要模块状态机组件指针
    unique_robot.registerChassis(GetChassisIns());
    unique_robot.registerGimbal(GetGimbalIns());
    unique_robot.registerFeed(GetFeedIns());
    unique_robot.registerFric(GetFricIns());

    // 无通信功能的组件指针
    // unique_robot.registerBuzzer(GetBuzzerIns());
    unique_robot.registerImu(GetImuIns());

    // 只接受数据的组件指针
    unique_robot.registerRc(GetRcIns());

    // 发送接收数据的组件指针
    // chassis
    unique_robot.registerCap(GetCapIns());
    unique_robot.registerMotorWheels(GetMotorWheelLeftFrontIns(), robot::Chassis::kWheelMotorIdxLeftFront);
    unique_robot.registerMotorWheels(GetMotorWheelLeftRearIns(), robot::Chassis::kWheelMotorIdxLeftRear);
    unique_robot.registerMotorWheels(GetMotorWheelRightRearIns(), robot::Chassis::kWheelMotorIdxRightRear);
    unique_robot.registerMotorWheels(GetMotorWheelRightFrontIns(), robot::Chassis::kWheelMotorIdxRightFront);

    // gimbal
    unique_robot.registerMotorGimbal(GetMotorGimbalPitchIns(), robot::Gimbal::kJointIdxGimbalPitch);
    unique_robot.registerMotorGimbal(GetMotorGimbalYawIns(), robot::Gimbal::kJointIdxGimbalYaw);
    unique_robot.registerMotorGimbal(GetMotorGimbalRollIns(), robot::Gimbal::kJointIdxGimbalRoll);

    // shooter
    unique_robot.registerMotorFeed(GetMotorShooterFeedIns());
    unique_robot.registerMotorFric(GetMotorShooterFricLeftIns(), 0);
    unique_robot.registerMotorFric(GetMotorShooterFricRightIns(), 1);

    unique_robot.registerVision(GetVisionIns());

    unique_robot.registerUiMgr(GetUiMgrIns());
    unique_robot.registerUiDrawer(GetUiDrawerIns());

    // rfr
    unique_robot.registerRfr(GetRfrIns());

    unique_robot.registerRfrCompStatusPkg(GetRfrCompStatusPkgIns());
    unique_robot.registerRfrTeamEventPkg(GetRfrTeamEventPkgIns());
    unique_robot.registerRfrRobotPerformancePkg(GetRfrRobotPerformancePkgIns());
    unique_robot.registerRfrRobotPosPkg(GetRfrRobotPosPkgIns());
    unique_robot.registerRfrRobotResourcePkg(GetRfrRobotResourcePkgIns());
    unique_robot.registerRfrRobotRfidPkg(GetRfrRobotRfidPkgIns());
    unique_robot.registerRfrRobotsGroundPosPkg(GetRfrRobotsGroundPosPkgIns());
    unique_robot.registerRfrRobotSentryDecisionPkg(GetRfrRobotSentryDecisionPkgIns());
    unique_robot.registerRfrCompRobotsHpPkg(GetRfrCompRobotsHpPkgIns());
    unique_robot.registerRfrInterMapClientToRobotPkg(GetRfrInterMapClientToRobotPkgIns());
    unique_robot.registerRfrRobotPowerHeatPkg(GetRfrRobotPowerHeatPkgIns());
    unique_robot.registerRfrRobotShooterPkg(GetRfrRobotShooterPkgIns());
    unique_robot.registerRfrRobotHurtPkg(GetRfrRobotHurtPkgIns());
    unique_robot.registerRfrRobotBuffPkg(GetRfrRobotBuffPkgIns());
    unique_robot.registerRfrInterRadarDetectionPkg(GetRfrInterRadarDetectionPkgIns());

    unique_robot.registerRfrInterSentryCmd(GetRfrInterSentryCmdIns());
    unique_robot.registerRfrInterSentryDetectionPkg(GetRfrInterSentryDetectionPkgIns());
    unique_robot.registerRfrInterRobotTrajPkg(GetRfrInterRobotTrajPkgIns());
    is_robot_inited = true;
  }
  return &unique_robot;
};

robot::Chassis *GetChassisIns(void)
{
  if (!is_chassis_inited) {
    unique_chassis.registerIkSolver(GetChassisIkSolverIns());

    unique_chassis.registerPid(GetPidMotorWheelLeftFrontIns(), robot::Chassis::kPidIdxWheelMotorLeftFront);
    unique_chassis.registerPid(GetPidMotorWheelLeftRearIns(), robot::Chassis::kPidIdxWheelMotorLeftRear);
    unique_chassis.registerPid(GetPidMotorWheelRightRearIns(), robot::Chassis::kPidIdxWheelMotorRightRear);
    unique_chassis.registerPid(GetPidMotorWheelRightFrontIns(), robot::Chassis::kPidIdxWheelMotorRightFront);

    unique_chassis.registerPid(GetPidFollowOmegaIns(), robot::Chassis::kPidIdxFollowOmega);
    unique_chassis.registerPid(GetPidFollowSpeedOmegaIns(), robot::Chassis::kPidIdxFollowSpeedOmega);
    unique_chassis.registerPid(GetPidGyroIns(), robot::Chassis::kPidIdxGyro);

    unique_chassis.registerPwrLimiter(GetPwrLimiterIns());

    unique_chassis.registerCap(GetCapIns());

    unique_chassis.registerWheelMotor(GetMotorWheelLeftFrontIns(), robot::Chassis::kWheelMotorIdxLeftFront);
    unique_chassis.registerWheelMotor(GetMotorWheelLeftRearIns(), robot::Chassis::kWheelMotorIdxLeftRear);
    unique_chassis.registerWheelMotor(GetMotorWheelRightRearIns(), robot::Chassis::kWheelMotorIdxRightRear);
    unique_chassis.registerWheelMotor(GetMotorWheelRightFrontIns(), robot::Chassis::kWheelMotorIdxRightFront);

    is_chassis_inited = true;
  }
  return &unique_chassis;
};

robot::Gimbal *GetGimbalIns(void)
{
  if (!is_gimbal_inited) {
    unique_gimbal.registerPid(GetPidMotorGimbalPitchIns(), robot::Gimbal::kJointIdxGimbalPitch);
    unique_gimbal.registerPid(GetPidMotorGimbalYawIns(), robot::Gimbal::kJointIdxGimbalYaw);
    unique_gimbal.registerPid(GetPidMotorGimbalRollIns(), robot::Gimbal::kJointIdxGimbalRoll);

    unique_gimbal.registerMotor(GetMotorGimbalPitchIns(), robot::Gimbal::kJointIdxGimbalPitch);
    unique_gimbal.registerMotor(GetMotorGimbalYawIns(), robot::Gimbal::kJointIdxGimbalYaw);
    unique_gimbal.registerMotor(GetMotorGimbalRollIns(), robot::Gimbal::kJointIdxGimbalRoll);

    is_gimbal_inited = true;
  }
  return &unique_gimbal;
};

hw_module::Feed *GetFeedIns()
{
  if (!is_feed_inited) {
    unique_feed.registerMotorFeed(GetMotorShooterFeedIns());
    unique_feed.registerPidFeed(GetPidMotorShooterFeedIns());
    is_feed_inited = true;
  }
  return &unique_feed;
}

hw_module::Fric *GetFricIns()
{
  if (!is_fric_inited) {
    unique_fric.registerMotor(GetMotorShooterFricLeftIns());
    unique_fric.registerMotor(GetMotorShooterFricRightIns());

    unique_fric.registerPid(GetPidMotorShooterLeftFricIns());
    unique_fric.registerPid(GetPidMotorShooterRightFricIns());
    // unique_fric.registerPid(GetPidFricSpeedSynIns());
  }
  return &unique_fric;
}