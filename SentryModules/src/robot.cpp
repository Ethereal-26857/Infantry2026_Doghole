#include "robot.hpp"

// DEBUG:
bool is_handled = false;
hello_world::referee::InterMapClientToRobotData Intermaprobot_data = {0.0f, 0.0f, 0, 0, 0};

namespace robot
{

#pragma region 数据更新

// 状态机主要接口函数
void Robot::update()
{
  updateData();
  updatePwrState();
}

void Robot::updateData()
{
  work_tick_ = getCurrentTickMs();

  updateImuData();
  updateRcData();
  updateRfrData();
  updateModuleState();
}

void Robot::updateImuData()
{
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isOffsetCalcFinished()) {
    is_imu_caled_offset_ = true;
  }
}

void Robot::updateRcData()
{
  HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);
  last_SL_ = now_SL_;
  last_SR_ = now_SR_;
  now_SL_ = rc_ptr_->rc_l_switch();
  now_SR_ = rc_ptr_->rc_r_switch();

  // 控制源自动切换：键鼠活动 → KB，拨杆变化 → RC
  if (rc_ptr_->isUsingKeyboardMouse()) {
    manual_ctrl_src_ = hello_world::module::ManualCtrlSrc::kKb;
  } else if (rc_ptr_->isRcSwitchChanged()) {
    manual_ctrl_src_ = hello_world::module::ManualCtrlSrc::kRc;
  }
}

void Robot::updateRfrData()
{
  HW_ASSERT(rfr_ptr_ != nullptr, "RFR pointer is null", rfr_ptr_);

  Chassis::RfrData chassis_rfr_data;
  Feed::RfrInputData feed_rfr_data;
  Fric::RfrInputData fric_rfr_data;

  static RobotPerformancePkg::Data rpp_data = kDefaultRobotPerformanceData;
  static RobotPowerHeatPkg::Data rph_data = kDefaultRobotPowerHeatData;
  static RobotShooterPkg::Data rsp_data = kDefaultRobotShooterData;
  static RobotBuffPkg::Data rbuf_data = kDefaultRobotBuffData;

  if (!rfr_ptr_->isOffline()) {
    rpp_data = rfr_robot_performance_pkg_ptr_->getData();
    rph_data = rfr_robot_power_heat_pkg_ptr_->getData();
    rsp_data = rfr_robot_shooter_pkg_ptr_->getData();
    rbuf_data = rfr_robot_buff_pkg_ptr_->getData();

    rfr_inter_radar_detection_pkg_ptr_->setReceiverId(rpp_data.robot_id + 2);
    rfr_inter_radar_detection_pkg_ptr_->setSenderId(rpp_data.robot_id);

    robot_id_ = rpp_data.robot_id;
  }

  // chassis
  chassis_rfr_data.is_pwr_on = rpp_data.power_management_chassis_output;
  chassis_rfr_data.pwr_limit = rpp_data.chassis_power_limit;
  chassis_rfr_data.pwr_buffer = rph_data.buffer_energy;
  chassis_rfr_data.current_hp = rpp_data.current_hp;
  chassis_rfr_data.current_remaining_energy = rbuf_data.remaining_energy;
  chassis_ptr_->setRfrData(chassis_rfr_data);

  // shooter
  feed_rfr_data.is_rfr_on = !rfr_ptr_->isOffline();
  feed_rfr_data.is_power_on = rpp_data.power_management_shooter_output;
  feed_rfr_data.heat_limit = rpp_data.shooter_barrel_heat_limit;
  feed_rfr_data.heat = rph_data.shooter_17mm_1_barrel_heat;
  feed_rfr_data.heat_cooling_ps = rpp_data.shooter_barrel_cooling_value;

  fric_rfr_data.is_power_on = rpp_data.power_management_shooter_output;
  fric_rfr_data.bullet_spd = rsp_data.bullet_speed;

  if (!rfr_robot_shooter_pkg_ptr_->isHandled()) {
    feed_rfr_data.is_new_bullet_shot = true;
    fric_rfr_data.is_new_bullet_shot = true;

    last_shoot_speed = shoot_speed;
    shoot_speed = shoot_speed * 0.5 + rfr_robot_shooter_pkg_ptr_->getData().bullet_speed * 0.5;

    rfr_robot_shooter_pkg_ptr_->setHandled();
  } else {
    feed_rfr_data.is_new_bullet_shot = false;
    fric_rfr_data.is_new_bullet_shot = false;
  }

  feed_ptr_->updateRfrData(feed_rfr_data);
  fric_ptr_->updateRfrData(fric_rfr_data);
}

void Robot::updatePwrState()
{
  PwrState pre_state = pwr_state_;
  PwrState next_state = pre_state;
  if (pre_state == PwrState::kDead) {
    // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
    next_state = PwrState::kResurrection;
  } else if (pre_state == PwrState::kResurrection) {
    // 复活状态下，等待IMU校准完成
    if (is_imu_caled_offset_ && work_tick_ > 1000) {
      next_state = PwrState::kWorking;
    }
  } else if (pre_state == PwrState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态下，认为是死亡状态
    next_state = PwrState::kDead;
  }

  // 应急开关，左右拨杆同时拨到最上面，切到死亡状态，所有模块失能
  if (now_SL_ == RcSwitchState::kUp && now_SR_ == RcSwitchState::kUp) {
    next_state = PwrState::kDead;
  }

  setPwrState(next_state);
}

void Robot::updateModuleState()
{
  feed_ptr_->setFricStatus(fric_ptr_->getStatus());
  chassis_ptr_->setImuAng(imu_ptr_->yaw());
}

#pragma endregion

#pragma region 执行任务

void Robot::run()
{
  if (pwr_state_ == PwrState::kDead) {
    runOnDead();
  } else if (pwr_state_ == PwrState::kResurrection) {
    runOnResurrection();
  } else if (pwr_state_ == PwrState::kWorking) {
    runOnWorking();
  } else {
    runOnDead();
  }

  setCommData();
  sendCommData();
}

void Robot::runOnDead()
{
  resetDataOnDead();

  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->standby();

  gimbal_ptr_->update();
  gimbal_ptr_->standby();

  feed_ptr_->update();
  feed_ptr_->standby();
  fric_ptr_->update();
  fric_ptr_->standby();
}

void Robot::runOnResurrection()
{
  resetDataOnResurrection();

  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->standby();

  gimbal_ptr_->update();
  gimbal_ptr_->standby();

  feed_ptr_->update();
  feed_ptr_->standby();
  fric_ptr_->update();
  fric_ptr_->standby();
}

void Robot::runOnWorking()
{
  genModulesCmd();

  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  chassis_ptr_->update();
  if (is_chassis_standby_) {
    chassis_ptr_->standby();
  } else {
    chassis_ptr_->run();
  }

  gimbal_ptr_->update();
  if (is_gimbal_standby_) {
    gimbal_ptr_->standby();
  } else {
    gimbal_ptr_->run();
  }

  feed_ptr_->update();
  if (is_feed_standby_) {
    feed_ptr_->standby();
  } else {
    feed_ptr_->run();
  }

  fric_ptr_->update();
  if (is_fric_standby_) {
    fric_ptr_->standby();
  } else {
    fric_ptr_->run();
  }
}

void Robot::standby()
{
  HW_ASSERT(chassis_ptr_ != nullptr, "Chassis FSM pointer is null", chassis_ptr_);
  chassis_ptr_->update();
  chassis_ptr_->standby();

  gimbal_ptr_->update();
  gimbal_ptr_->standby();

  feed_ptr_->update();
  feed_ptr_->standby();
  fric_ptr_->update();
  fric_ptr_->standby();
}
#pragma endregion

#pragma region 生成控制指令

void Robot::genModulesCmd()
{
  HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);

  if (manual_ctrl_src_ == hello_world::module::ManualCtrlSrc::kKb) {
    genModulesCmdFromKb();
    return;
  }

  // ===== RC 遥控器模式（用户后续自行填充）=====
  // 暂时先全部写死，调试用
  is_chassis_standby_ = true;
  is_gimbal_standby_ = true;
  is_feed_standby_ = true;
  is_fric_standby_ = true;

  if (now_SL_ == RcSwitchState::kUp)
  {
    if (now_SR_ == RcSwitchState::kUp) 
    //左上右上
    {
      // 应急开关，左右拨杆同时拨到最上面，切到死亡状态，所有模块失能
      setPwrState(PwrState::kDead);
    }
    else if (now_SR_ == RcSwitchState::kMid) 
    //左上右中
    {

    }
    else 
    //左上右下
    {

    }

  }
  else if (now_SL_ == RcSwitchState::kMid) 
  {
    if (now_SR_ == RcSwitchState::kUp) 
    //左中右上
    {

    }
    else if (now_SR_ == RcSwitchState::kMid) 
    //左中右中
    {

    }
    else 
    //左中右下
    {

    }

  }
  else
  {
    if (now_SR_ == RcSwitchState::kUp) 
    //左下右上
    {

    }
    else if (now_SR_ == RcSwitchState::kMid) 
    //左下右中
    {

    }
    else 
    //左下右下
    {

    }

  }

  // float chassis_config_trans_vel = chassis_ptr_->getConfigNormTransVel();
  // Chassis::Cmd chassis_cmd = {0.0f, 0.0f, 0.0f};
  // Gimbal::Cmd gimbal_cmd = {0.0f, 0.0f};

  // // 默认：手动控制
  // gimbal_ptr_->setGimbalJointWorkingMode(GimbalJointWorkingMode::kManual);
  // chassis_ptr_->setWorkingMode(Chassis::WorkingMode::kFollow);

  // GimbalRcCmd(gimbal_cmd);
  // ChassisRcCmd(chassis_cmd, chassis_config_trans_vel);
  // chassis_ptr_->setNormCmd(chassis_cmd);
  // gimbal_ptr_->setNormCmdDelta(gimbal_cmd);

  // // Vision 自瞄：检测到目标时切入自瞄模式
  // if (!vision_ptr_->isOffline() && vision_ptr_->getIsEnemyDetected() &&
  //     gimbal_ptr_->isVisCmdInRange(vision_ptr_->getPitchPos(), vision_ptr_->getYawPos())) {
  //   gimbal_ptr_->setGimbalJointWorkingMode(GimbalJointWorkingMode::kAutoAim);
  //   gimbal_ptr_->setVisCmd(vision_ptr_->getPitchPos(), vision_ptr_->getYawPos());
  //   feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kAuto);
  //   feed_ptr_->setVisionShootFlag(vision_ptr_->getShootFlag());
  // } else {
  //   feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kManual);
  //   // SR 拨杆控制手动发弹
  //   if (now_SR_ == RcSwitchState::kDown) {
  //     feed_ptr_->setManualShootFlag(true);
  //   } else {
  //     feed_ptr_->setManualShootFlag(false);
  //   }
  // }

  // // 摩擦轮：比赛中开启，否则停止
  // if (rfr_comp_status_pkg_ptr_->getData().game_progress == static_cast<uint8_t>(hello_world::referee::CompStage::kOngoing)) {
  //   fric_ptr_->setWorkingMode(hello_world::module::Fric::WorkingMode::kShoot);
  // } else {
  //   fric_ptr_->setWorkingMode(hello_world::module::Fric::WorkingMode::kStop);
  // }

  // feed_ptr_->setTriggerLimit(true, true, 3, 1000 / 20);
}

void Robot::genModulesCmdFromKb()
{
  HW_ASSERT(rc_ptr_ != nullptr, "RC pointer is null", rc_ptr_);

  // 激活所有模块（退出 standby）
  is_chassis_standby_ = false;
  is_gimbal_standby_ = false;
  is_feed_standby_ = false;
  is_fric_standby_ = false;

  float chassis_config_trans_vel = chassis_ptr_->getConfigNormTransVel();

  // ===== 底盘运动控制：WASD =====
  Chassis::Cmd chassis_cmd = {0.0f, 0.0f, 0.0f};
  chassis_cmd.v_x = (int8_t)rc_ptr_->key_W() - (int8_t)rc_ptr_->key_S();
  chassis_cmd.v_y = (int8_t)rc_ptr_->key_D() - (int8_t)rc_ptr_->key_A();
  chassis_cmd.v_x = hello_world::Bound(chassis_cmd.v_x * chassis_config_trans_vel,
                                       -chassis_config_trans_vel, chassis_config_trans_vel);
  chassis_cmd.v_y = hello_world::Bound(chassis_cmd.v_y * chassis_config_trans_vel,
                                       -chassis_config_trans_vel, chassis_config_trans_vel);
  chassis_ptr_->setNormCmd(chassis_cmd);

  // ===== 底盘工作模式：Q/E =====
  if (rc_ptr_->key_Q()) {
    chassis_ptr_->setWorkingMode(Chassis::WorkingMode::kFastGyro);
  } else if (rc_ptr_->key_E()) {
    chassis_ptr_->setWorkingMode(Chassis::WorkingMode::kFollow);
  }

  // ===== 行进方向控制：Z / Ctrl+Z =====
  static bool last_key_Z = false;
  bool key_Z_now = rc_ptr_->key_Z();
  if (key_Z_now && !last_key_Z && !rc_ptr_->key_CTRL()) {
    is_straight_ = !is_straight_;
  }
  if (rc_ptr_->key_CTRL() && key_Z_now) {
    is_straight_ = true;
  }
  last_key_Z = key_Z_now;
  // 将 is_straight_ 标志传递给底盘 (通过 working mode 或其他方式)
  chassis_ptr_->setSentryInhole(is_straight_ ? 0 : 1);

  // ===== 云台工作模式：Q=搜索, 鼠标右键=自瞄, X=自瞄+自动射击, 默认=手动 =====
  if (rc_ptr_->key_X()) {
    gimbal_ptr_->setGimbalJointWorkingMode(GimbalJointWorkingMode::kAutoAim);
    feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kAuto);
  } else if (rc_ptr_->mouse_r_btn()) {
    gimbal_ptr_->setGimbalJointWorkingMode(GimbalJointWorkingMode::kAutoAim);
    feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kManual);
  } else if (rc_ptr_->key_Q()) {
    gimbal_ptr_->setGimbalJointWorkingMode(GimbalJointWorkingMode::kSearch);
    feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kManual);
  } else {
    gimbal_ptr_->setGimbalJointWorkingMode(GimbalJointWorkingMode::kManual);
    feed_ptr_->setCtrlMode(hello_world::module::CtrlMode::kManual);
  }

  // ===== 云台指令：自瞄听视觉，手动/搜索用鼠标 =====
  if (gimbal_ptr_->getGimbalJointWorkingMode() == GimbalJointWorkingMode::kAutoAim) {
    // 自瞄模式：优先使用视觉目标角度
    if (!vision_ptr_->isOffline() && vision_ptr_->getIsEnemyDetected()
        && gimbal_ptr_->isVisCmdInRange(vision_ptr_->getPitchPos(), vision_ptr_->getYawPos())) {
      gimbal_ptr_->setVisCmd(vision_ptr_->getPitchPos(), vision_ptr_->getYawPos());
      if (feed_ptr_->getCtrlMode() == hello_world::module::CtrlMode::kAuto) {
        feed_ptr_->setVisionShootFlag(vision_ptr_->getShootFlag());
      }
    }
    // 视觉无效时，鼠标作为 fallback（Gimbal 内部 calcJointAngRef 自动降级）
    Gimbal::Cmd gimbal_fallback = {0.0f, 0.0f};
    gimbal_fallback.gimbal_pitch = hello_world::Bound(0.01f * rc_ptr_->mouse_y(), -1.0f, 1.0f);
    gimbal_fallback.gimbal_yaw   = hello_world::Bound(-0.01f * rc_ptr_->mouse_x(), -1.0f, 1.0f);
    gimbal_ptr_->setNormCmdDelta(gimbal_fallback);
  } else {
    // 手动/搜索模式：鼠标直接控制云台
    Gimbal::Cmd gimbal_cmd = {0.0f, 0.0f};
    gimbal_cmd.gimbal_pitch = hello_world::Bound(0.01f * rc_ptr_->mouse_y(), -1.0f, 1.0f);
    gimbal_cmd.gimbal_yaw   = hello_world::Bound(-0.01f * rc_ptr_->mouse_x(), -1.0f, 1.0f);
    gimbal_ptr_->setNormCmdDelta(gimbal_cmd);
  }

  // ===== 射击控制 =====
  // 鼠标左键 → 手动单发
  if (rc_ptr_->mouse_l_btn()) {
    feed_ptr_->setManualShootFlag(true);
  }

  // ===== 超电：Shift =====
  chassis_ptr_->setUseCapFlag(rc_ptr_->key_SHIFT());

  // ===== 摩擦轮 =====
  fric_ptr_->setWorkingMode(hello_world::module::Fric::WorkingMode::kShoot);

  // ===== 拨弹频率限制 =====
  feed_ptr_->setTriggerLimit(true, true, 3, 1000 / 20);
}

#pragma endregion

#pragma region 数据重置函数

void Robot::reset()
{
  pwr_state_ = PwrState::kDead;       ///< 电源状态
  last_pwr_state_ = PwrState::kDead;  ///< 上一电源状态

  now_SL_ = RcSwitchState::kErr;
  now_SR_ = RcSwitchState::kErr;
  last_SL_ = RcSwitchState::kErr;
  last_SR_ = RcSwitchState::kErr;

}

void Robot::resetDataOnDead()
{
  pwr_state_ = PwrState::kDead;       ///< 电源状态
  last_pwr_state_ = PwrState::kDead;  ///< 上一电源状态

  now_SL_ = RcSwitchState::kErr;
  now_SR_ = RcSwitchState::kErr;
  last_SL_ = RcSwitchState::kErr;
  last_SR_ = RcSwitchState::kErr;
}

void Robot::resetDataOnResurrection()
{
}

#pragma endregion

#pragma region 通信数据设置函数

void Robot::setCommData()
{
  // 更新 Vision TX 数据
  vision_ptr_->setMotionState(0,
      gimbal_ptr_->getJointGimbalPitchAng(), 0, 0,
      gimbal_ptr_->getJointGimbalYawAng(), 0, 0);
  vision_ptr_->setBulletSpeed(shoot_speed);

  // 哨兵复活指令
  rfr_inter_sentry_cmd_ptr_->setSenderId(robot_id_);
  rfr_inter_sentry_cmd_ptr_->getData().is_to_revive = 1;

  // 更新 UI 绘制数据
  setUiDrawerData();
}

void Robot::setUiDrawerData()
{
  if (ui_drawer_ptr_ == nullptr) return;

  // Chassis 状态
  ui_drawer_ptr_->setChassisWorkState(chassis_ptr_->getPwrState());
  ui_drawer_ptr_->setChassisCtrlMode(
      gimbal_ptr_->getGimbalJointWorkingMode() == GimbalJointWorkingMode::kAutoAim
          ? hello_world::module::CtrlMode::kAuto
          : hello_world::module::CtrlMode::kManual);
  ui_drawer_ptr_->setChassisManualCtrlSrc(manual_ctrl_src_);
  ui_drawer_ptr_->setChassisWorkingMode(chassis_ptr_->getWorkingMode());
  ui_drawer_ptr_->setChassisHeadDir(0.0f);  // theta_i2r from IMU if available

  // Gimbal 状态
  ui_drawer_ptr_->setGimbalWorkState(gimbal_ptr_->getPwrState());
  ui_drawer_ptr_->setGimbalCtrlMode(
      gimbal_ptr_->getGimbalJointWorkingMode() == GimbalJointWorkingMode::kAutoAim
          ? hello_world::module::CtrlMode::kAuto
          : hello_world::module::CtrlMode::kManual);
  ui_drawer_ptr_->setGimbalWorkingMode(gimbal_ptr_->getGimbalJointWorkingMode());
  ui_drawer_ptr_->setGimbalJointAngPitchFdb(gimbal_ptr_->getJointGimbalPitchAng());
  ui_drawer_ptr_->setGimbalJointAngYawFdb(gimbal_ptr_->getJointGimbalYawAng());

  // Shooter 相关
  ui_drawer_ptr_->setHeat(0.0f);            // from rfr data
  ui_drawer_ptr_->setHeatLimit(100.0f);     // from rfr data
  ui_drawer_ptr_->setBulletNum(feed_ptr_->getTriggerCnt());
  ui_drawer_ptr_->setFeedStuckFlag(
      feed_ptr_->getStuckStatus() != hello_world::module::feed_impl::FeedStuckStatus::kNone);
  ui_drawer_ptr_->setFricStuckFlag(!fric_ptr_->getStatus());

  // Vision
  if (vision_ptr_ != nullptr && !vision_ptr_->isOffline()) {
    ui_drawer_ptr_->setisvisionvalid(vision_ptr_->getIsEnemyDetected());
    if (vision_ptr_->getIsEnemyDetected()) {
      ui_drawer_ptr_->setVisTgtX(static_cast<uint16_t>(vision_ptr_->getVtmX()), true);
      ui_drawer_ptr_->setVisTgtY(static_cast<uint16_t>(vision_ptr_->getVtmY()), true);
    }
  }

  // Cap
  ui_drawer_ptr_->setCapPwrPercent(0.0f);       // from cap data
  ui_drawer_ptr_->setBufferCapPwrPercent(0.0f); // from cap data

  // 其他
  ui_drawer_ptr_->setIsStraight(is_straight_);
  ui_drawer_ptr_->setNavigateFlag(false);

  // 刷新图形
  ui_drawer_ptr_->updateDynamicUi();
}

#pragma endregion

#pragma region 通信数据发送函数

void Robot::sendCommData()
{
  sendCanData();
  sendUsartData();
  sendVisionData();
}

void Robot::sendCanData()
{
  // 降低通信频率不然可能出现严重丢包

  // 发送频率500Hz
  if (work_tick_ % 2 == 0) {
    /* chassis */
    sendWheelsMotorData();

    /* shooter */
    sendFeedMotorData();
    sendFricMotorData();

    /* gimbal */
    sendGimbalYawMotorData();
  } else {
    /* gimbal */
    sendGimbalPitchMotorData();
    HW_ASSERT(gimbal_motor_ptr_[Gimbal::kJointIdxGimbalRoll] != nullptr, "Gimbal roll motor pointer is null");
    gimbal_motor_ptr_[Gimbal::kJointIdxGimbalRoll]->setNeedToTransmit();
  }

  // super cap
  if (work_tick_ % 10 == 0) {
    sendCapData();
  }
}

void Robot::sendWheelsMotorData()
{
  for (size_t i = 0; i < Chassis::kWheelMotorNum; i++) {
    HW_ASSERT(wheel_motor_ptr_[i] != nullptr, "Wheel motor pointer %d is null", i);
    wheel_motor_ptr_[i]->setNeedToTransmit();
  }
}

void Robot::sendGimbalYawMotorData()
{
  HW_ASSERT(gimbal_motor_ptr_[Gimbal::kJointIdxGimbalYaw] != nullptr, "Gimbal yaw motor pointer is null");
  gimbal_motor_ptr_[Gimbal::kJointIdxGimbalYaw]->setNeedToTransmit();
}

void Robot::sendGimbalPitchMotorData()
{
  HW_ASSERT(gimbal_motor_ptr_[Gimbal::kJointIdxGimbalPitch] != nullptr, "Gimbal pitch motor pointer is null");
  gimbal_motor_ptr_[Gimbal::kJointIdxGimbalPitch]->setNeedToTransmit();
}

void Robot::sendFeedMotorData()
{
  HW_ASSERT(feed_motor_ptr_ != nullptr, "Feed motor pointer is null", feed_motor_ptr_);
  feed_motor_ptr_->setNeedToTransmit();
}

void Robot::sendFricMotorData()
{
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(fric_motor_ptr_[i] != nullptr, "Fric motor pointer %d is null", i);
    fric_motor_ptr_[i]->setNeedToTransmit();
  }
}

void Robot::sendCapData()
{
  cap_ptr_->setNeedToTransmit();
}

void Robot::sendUsartData()
{
  sendRfrData();
}

void Robot::sendRfrData()
{
  if (work_tick_ % 100 == 0) {
    rfr_ptr_->setTxPkg(rfr_inter_sentry_cmd_ptr_);
  } else if (work_tick_ % 1000 == 1) {
    rfr_ptr_->setTxPkg(rfr_inter_robot_traj_pkg_ptr_);
  }
  rfr_ptr_->setNeedToTransmit();

  // UI 图形数据发送（每 200ms 刷新一次，降低裁判系统负载）
  if (work_tick_ % 200 == 50 && ui_mgr_ptr_ != nullptr) {
    ui_mgr_ptr_->refresh();
    ui_mgr_ptr_->setNeedToTransmit();
  }
}

void Robot::sendVisionData()
{
  // Vision 通过 UART TX mgr 发送，由 comm_task 周期调用 encode
}

#pragma endregion

#pragma region 工具函数
void Robot::GimbalRcCmd(Gimbal::Cmd &cmd)
{
  cmd.gimbal_pitch = hello_world::Bound(rc_ptr_->rc_rv(), -1.0f, 1.0f);
  cmd.gimbal_yaw = hello_world::Bound(-1.0f * rc_ptr_->rc_rh(), -1.0f, 1.0f);
  gimbal_ptr_->setNormCmdDelta(cmd);
}

void Robot::ChassisRcCmd(Chassis::Cmd &cmd, float v_max)
{
  cmd.v_x = hello_world::Bound(rc_ptr_->rc_rv() * v_max, -v_max, v_max);
  cmd.v_y = hello_world::Bound(rc_ptr_->rc_rh() * -1.0f * v_max, -v_max, v_max);
  chassis_ptr_->setNormCmd(cmd);
}

#pragma endregion

#pragma region 注册函数
void Robot::registerChassis(Chassis *ptr)
{
  HW_ASSERT(ptr != nullptr, "Chassis FSM pointer is null", ptr);
  chassis_ptr_ = ptr;
}

void Robot::registerGimbal(Gimbal *ptr)
{
  HW_ASSERT(ptr != nullptr, "Gimbal FSM pointer is null", ptr);
  gimbal_ptr_ = ptr;
}

void Robot::registerFeed(Feed *ptr)
{
  HW_ASSERT(ptr != nullptr, "Feed FSM pointer is null", ptr);
  feed_ptr_ = ptr;
}

void Robot::registerFric(Fric *ptr)
{
  HW_ASSERT(ptr != nullptr, "Fric FSM pointer is null", ptr);
  fric_ptr_ = ptr;
}

void Robot::registerBuzzer(Buzzer *ptr)
{
  HW_ASSERT(ptr != nullptr, "Buzzer pointer is null", ptr);
  buzzer_ptr_ = ptr;
}

void Robot::registerImu(Imu *ptr)
{
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
}

void Robot::registerMotorWheels(Motor *dev_ptr, uint8_t idx)
{
  HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
  HW_ASSERT(idx >= 0 && idx < Chassis::kWheelMotorNum, "Motor index is out of range", idx);
  wheel_motor_ptr_[idx] = dev_ptr;
}

void Robot::registerMotorGimbal(Motor *dev_ptr, uint8_t idx)
{
  HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
  HW_ASSERT(idx >= 0 && idx < Gimbal::kJointNum, "Motor index is out of range", idx);
  gimbal_motor_ptr_[idx] = dev_ptr;
}

void Robot::registerMotorFeed(Motor *dev_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
  feed_motor_ptr_ = dev_ptr;
}

void Robot::registerMotorFric(Motor *dev_ptr, uint8_t idx)
{
  HW_ASSERT(dev_ptr != nullptr, "Motor pointer is null", dev_ptr);
  HW_ASSERT(idx >= 0 && idx < 2, "Motor index is out of range", idx);
  fric_motor_ptr_[idx] = dev_ptr;
}

void Robot::registerCap(Cap *dev_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "Cap pointer is null", dev_ptr);
  cap_ptr_ = dev_ptr;
}

void Robot::registerRc(DT7 *ptr)
{
  HW_ASSERT(ptr != nullptr, "RC pointer is null", ptr);
  rc_ptr_ = ptr;
}

void Robot::registerVision(hello_world::vision::Vision *ptr)
{
  HW_ASSERT(ptr != nullptr, "Vision pointer is null", ptr);
  vision_ptr_ = ptr;
}

void Robot::registerUiMgr(UiMgr *ptr)
{
  HW_ASSERT(ptr != nullptr, "UiMgr pointer is null", ptr);
  ui_mgr_ptr_ = ptr;
}

void Robot::registerUiDrawer(UiDrawer *ptr)
{
  HW_ASSERT(ptr != nullptr, "UiDrawer pointer is null", ptr);
  ui_drawer_ptr_ = ptr;
}

void Robot::registerRfr(Referee *dev_ptr)
{
  HW_ASSERT(dev_ptr != nullptr, "Referee pointer is null", dev_ptr);
  rfr_ptr_ = dev_ptr;
}

void Robot::registerRfrCompStatusPkg(CompStatusPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr CompStatusPkg pointer is null", ptr);
  rfr_comp_status_pkg_ptr_ = ptr;
}

void Robot::registerRfrTeamEventPkg(TeamEventPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr TeamEventPkg pointer is null", ptr);
  rfr_team_event_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotPerformancePkg(RobotPerformancePkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotPerformancePkg pointer is null", ptr);
  rfr_robot_performance_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotPosPkg(RobotPosPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotPosPkg pointer is null", ptr);
  rfr_robot_pos_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotResourcePkg(RobotResourcePkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotResourcePkg pointer is null", ptr);
  rfr_robot_resource_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotRfidPkg(RobotRfidPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotRfidPkg pointer is null", ptr);
  rfr_robot_rfid_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotsGroundPosPkg(RobotsGroundPosPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotsGroundPosPkg pointer is null", ptr);
  rfr_robots_ground_pos_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotSentryDecisionPkg(RobotSentryDecisionPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotSentryDecisionPkg pointer is null", ptr);
  rfr_robot_sentry_decision_pkg_ptr_ = ptr;
}

void Robot::registerRfrCompRobotsHpPkg(CompRobotsHpPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr CompRobotsHpPkg pointer is null", ptr);
  rfr_comp_robots_hp_pkg_ptr_ = ptr;
}

void Robot::registerRfrInterMapClientToRobotPkg(InterMapClientToRobotPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr InterMapClientToRobotPkg pointer is null", ptr);
  rfr_inter_map_client_to_robot_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotPowerHeatPkg(RobotPowerHeatPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfe RobotPowerHeatPkg pointer is null", ptr);
  rfr_robot_power_heat_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotShooterPkg(RobotShooterPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotShootDataPkg pointer is null", ptr);
  rfr_robot_shooter_pkg_ptr_ = ptr;
}

void Robot::registerRfrRobotHurtPkg(RobotHurtPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotHurtPkg pointer is null", ptr);
  rfr_robot_hurt_pkg_ptr_ = ptr;
}

void Robot::registerRfrInterSentryCmd(InterSentryCmd *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr InterSentryCmd pointer is null", ptr);
  rfr_inter_sentry_cmd_ptr_ = ptr;
}

void Robot::registerRfrRobotBuffPkg(RobotBuffPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr RobotBuffPkg pointer is null", ptr);
  rfr_robot_buff_pkg_ptr_ = ptr;
}

void Robot::registerRfrInterSentryDetectionPkg(InterSentryDetectionPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr InterSentryDetection pointer is null", ptr);
  rfr_inter_sentry_detection_pkg_ptr_ = ptr;
}

void Robot::registerRfrInterRadarDetectionPkg(InterRadarDetectionPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr InterRadarDetection pointer is null", ptr);
  rfr_inter_radar_detection_pkg_ptr_ = ptr;
}

void Robot::registerRfrInterRobotTrajPkg(InterRobotTrajPkg *ptr)
{
  HW_ASSERT(ptr != nullptr, " Rfr InterRobotTraj pointer is null", ptr);
  rfr_inter_robot_traj_pkg_ptr_ = ptr;
}

#pragma endregion

}  // namespace robot