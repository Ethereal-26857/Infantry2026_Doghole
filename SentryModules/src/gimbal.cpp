#include "gimbal.hpp"

// DEBUG:
static float debug_ref[3] = {-1.0f, -1.0f, -1.0f};
static float debug_fdb[3] = {0.0f};
static float debug_out[3] = {0.0f};
static float debug_err[3] = {0.0f};
float debug_torq[3] = {0.0f};

float debug_vision_yaw = 0.0f;
float debug_yaw_deta = 0.0f;
float debug_yaw_fdb = 0.0f;

namespace robot
{
#pragma region 数据更新

void Gimbal::update()
{
  updateData();
  updatePwrState();
};

void Gimbal::updateData()
{
  work_tick_ = getCurrentTickMs();

  updateMotorData();
  updateIsPwrOn();
  updateJointData();
};

void Gimbal::updatePwrState()
{
  // 无论任何状态，断电意味着切到死亡状态
  if (!is_pwr_on_) {
    setPwrState(PwrState::kDead);
    return;
  }

  PwrState current_state = getPwrState();
  PwrState next_state = current_state;
  if (current_state == PwrState::kDead) {
    // 死亡状态下，如果上电，则切换到复活状态
    if (is_pwr_on_) {
      next_state = PwrState::kResurrection;
    }
  } else if (current_state == PwrState::kResurrection) {
    if (is_any_motor_pwr_on_) {
      next_state = PwrState::kWorking;
    }
  } else if (current_state == PwrState::kWorking) {
  } else {
    next_state = PwrState::kDead;
  }
  setPwrState(next_state);
};

void Gimbal::updateMotorData()
{
  Motor *motor_ptr = nullptr;
  JointIdx motor_idxs[kJointNum] = {kJointIdxGimbalPitch, kJointIdxGimbalYaw, kJointIdxGimbalRoll};

  bool is_any_motor_pwr_on = false;
  bool is_all_motor_pwr_on = true;

  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx motor_idx = motor_idxs[i];
    motor_ptr = motor_ptr_[motor_idx];
    HW_ASSERT(motor_ptr != nullptr, "pointer to motor is nullptr");
    if (motor_ptr->isOffline()) {
      motor_ang_fdb_[motor_idx] = 0.0f;
      motor_spd_fdb_[motor_idx] = 0.0f;
      is_all_motor_pwr_on = false;
    } else {
      last_motor_spd_fdb_[motor_idx] = motor_spd_fdb_[motor_idx];
      motor_ang_fdb_[motor_idx] = motor_ptr->angle();
      motor_spd_fdb_[motor_idx] = motor_ptr->vel();

      is_any_motor_pwr_on = true;

      // DEBUG:
      debug_torq[i] = motor_ptr->torq();
    }
  }

  is_any_motor_pwr_on_ = is_any_motor_pwr_on;
  is_all_motor_pwr_on_ = is_all_motor_pwr_on;
};

void Gimbal::updateIsPwrOn()
{
  is_pwr_on_ = is_any_motor_pwr_on_ || is_rfr_pwr_on_;
};

void Gimbal::updateJointData()
{
  joint_ang_fdb_[kJointIdxGimbalPitch] = motor_ang_fdb_[kJointIdxGimbalPitch];
  joint_ang_fdb_[kJointIdxGimbalYaw] = motor_ang_fdb_[kJointIdxGimbalYaw];
  joint_ang_fdb_[kJointIdxGimbalRoll] = motor_ang_fdb_[kJointIdxGimbalRoll];

  joint_spd_fdb_[kJointIdxGimbalPitch] = motor_spd_fdb_[kJointIdxGimbalPitch];
  joint_spd_fdb_[kJointIdxGimbalYaw] = motor_spd_fdb_[kJointIdxGimbalYaw];
  joint_spd_fdb_[kJointIdxGimbalRoll] = motor_spd_fdb_[kJointIdxGimbalRoll];
}

#pragma endregion

#pragma region 执行任务

void Gimbal::run()
{
  PwrState current_state = getPwrState();
  if (current_state == PwrState::kDead) {
    runOnDead();
  } else if (current_state == PwrState::kResurrection) {
    runOnResurrection();
  } else if (current_state == PwrState::kWorking) {
    runOnWorking();
  } else {
    runOnDead();
  }
};

void Gimbal::runOnDead()
{
  resetDataOnDead();
  setCommData(false);
};

void Gimbal::runOnResurrection()
{
  resetDataOnResurrection();
  setCommData(false);
};

void Gimbal::runOnWorking()
{
  calcJointAngRef();
  calcJointTorRef();
  setCommData(true);
};

void Gimbal::standby()
{
  calcJointAngRefOnStandby();
  setCommData(false);
};

void Gimbal::calcJointAngRefOnStandby()
{
  JointIdx joint_idxs[kJointNum] = {kJointIdxGimbalPitch, kJointIdxGimbalYaw};
  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    joint_ang_ref_[joint_idx] = joint_ang_fdb_[joint_idx];
    last_joint_ang_ref_[joint_idx] = joint_ang_fdb_[joint_idx];
  }
};

void Gimbal::calcJointAngRef()
{
  if (gimbal_joint_last_working_mode_ != gimbal_joint_working_mode_) {
    last_joint_ang_ref_[kJointIdxGimbalPitch] = joint_ang_fdb_[kJointIdxGimbalPitch];
    last_joint_ang_ref_[kJointIdxGimbalYaw] = joint_ang_fdb_[kJointIdxGimbalYaw];
    last_joint_ang_ref_[kJointIdxGimbalRoll] = joint_ang_fdb_[kJointIdxGimbalRoll];
  } else {
    last_joint_ang_ref_[kJointIdxGimbalPitch] = joint_ang_ref_[kJointIdxGimbalPitch];
    last_joint_ang_ref_[kJointIdxGimbalYaw] = joint_ang_ref_[kJointIdxGimbalYaw];
    last_joint_ang_ref_[kJointIdxGimbalRoll] = joint_ang_ref_[kJointIdxGimbalRoll];
  }

  if ((gimbal_joint_last_working_mode_ == GimbalJointWorkingMode::kOutpost || gimbal_joint_last_working_mode_ == GimbalJointWorkingMode::kBuff) &&
      gimbal_joint_working_mode_ == GimbalJointWorkingMode::kManual) {
    // 退出前哨/Buff模式，重置云台角度
    calcGimbalJointAngRefReset();
  }

  // 云台模式
  if (gimbal_joint_working_mode_ == GimbalJointWorkingMode::kManual) {
    calcGimbalJointAngRefManualMode();
  } else if (gimbal_joint_working_mode_ == GimbalJointWorkingMode::kAutoAim) {
    calcGimbalJointAngRefAutoAimMode();
  } else if (gimbal_joint_working_mode_ == GimbalJointWorkingMode::kSearch) {
    calcGimbalJointAngRefSearchMode();
  } else if (gimbal_joint_working_mode_ == GimbalJointWorkingMode::kOutpost) {
    calcGimbalJointAngRefOutpostMode();
  } else if (gimbal_joint_working_mode_ == GimbalJointWorkingMode::kInhole) {
    calcGimbalJointAngRefInholeMode();
  } else if (gimbal_joint_working_mode_ == GimbalJointWorkingMode::kBuff) {
    calcGimbalJointAngRefBuffMode();
  }

  gimbal_joint_last_working_mode_ = gimbal_joint_working_mode_;
};

void Gimbal::calcGimbalJointAngRefAutoAimMode()
{
  float tmp_gimbal_pitch_ang_ref = vis_cmd_.gimbal_pitch;
  float tmp_gimbal_yaw_ang_ref = vis_cmd_.gimbal_yaw;

  tmp_gimbal_pitch_ang_ref = hello_world::Bound(tmp_gimbal_pitch_ang_ref, cfg_.GimbalPitch.min_ang_motor, cfg_.GimbalPitch.max_ang_motor);
  tmp_gimbal_yaw_ang_ref = hello_world::Bound(tmp_gimbal_yaw_ang_ref, cfg_.GimbalYaw.min_ang_motor, cfg_.GimbalYaw.max_ang_motor);

  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::AngleNormRad(tmp_gimbal_pitch_ang_ref);
  joint_ang_ref_[kJointIdxGimbalYaw] = hello_world::AngleNormRad(tmp_gimbal_yaw_ang_ref);
};

void Gimbal::calcGimbalJointAngRefReset()
{
  last_joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::Deg2Rad(0.0f);
  last_joint_ang_ref_[kJointIdxGimbalYaw] = 0.0f;
  last_joint_ang_ref_[kJointIdxGimbalRoll] = 0.0f;
  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::Deg2Rad(0.0f);
  joint_ang_ref_[kJointIdxGimbalYaw] = 0.0f;
  joint_ang_ref_[kJointIdxGimbalRoll] = 0.0f;
};

void Gimbal::calcGimbalJointAngRefManualMode()
{
  // 手动模式下，云台听从遥控器指令
  float sensitivity_pitch = cfg_.GimbalPitch.sensitivity_normal;
  float sensitivity_yaw = cfg_.GimbalYaw.sensitivity_normal;

  // Pitch 轴限位
  bool is_pitch_ang_too_large = motor_ang_fdb_[kJointIdxGimbalPitch] > cfg_.GimbalPitch.max_ang_motor - 0.05f;
  bool is_pitch_ang_too_small = motor_ang_fdb_[kJointIdxGimbalPitch] < cfg_.GimbalPitch.min_ang_motor + 0.05f;

  // Yaw 轴限位
  bool is_yaw_ang_too_large = motor_ang_fdb_[kJointIdxGimbalYaw] > cfg_.GimbalYaw.max_ang_motor - 0.05f;
  bool is_yaw_ang_too_small = motor_ang_fdb_[kJointIdxGimbalYaw] < cfg_.GimbalYaw.min_ang_motor + 0.05f;

  float tmp_gimbal_pitch_ang_ref = last_joint_ang_ref_[kJointIdxGimbalPitch];
  float tmp_gimbal_yaw_ang_ref = last_joint_ang_ref_[kJointIdxGimbalYaw];

  if (is_pitch_ang_too_large && norm_cmd_delta_.gimbal_pitch > 0.0f) {
  } else if (is_pitch_ang_too_small && norm_cmd_delta_.gimbal_pitch < 0.0f) {
  } else {
    tmp_gimbal_pitch_ang_ref += norm_cmd_delta_.gimbal_pitch * sensitivity_pitch;
  }

  if (is_yaw_ang_too_large && norm_cmd_delta_.gimbal_yaw > 0.0f) {
  } else if (is_yaw_ang_too_small && norm_cmd_delta_.gimbal_yaw < 0.0f) {
  } else {
    tmp_gimbal_yaw_ang_ref += norm_cmd_delta_.gimbal_yaw * sensitivity_yaw;
  }

  tmp_gimbal_pitch_ang_ref = hello_world::Bound(tmp_gimbal_pitch_ang_ref, cfg_.GimbalPitch.min_ang_motor, cfg_.GimbalPitch.max_ang_motor);
  tmp_gimbal_yaw_ang_ref = hello_world::Bound(tmp_gimbal_yaw_ang_ref, cfg_.GimbalYaw.min_ang_motor, cfg_.GimbalYaw.max_ang_motor);

  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::AngleNormRad(tmp_gimbal_pitch_ang_ref);
  joint_ang_ref_[kJointIdxGimbalYaw] = hello_world::AngleNormRad(tmp_gimbal_yaw_ang_ref);
};

void Gimbal::calcGimbalJointAngRefSearchMode()
{
  GimbalYawSearch(cfg_.GimbalYaw.min_ang_search, cfg_.GimbalYaw.max_ang_search);
  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::Deg2Rad(cfg_.GimbalPitch.fix_ang_search);
};

void Gimbal::calcGimbalJointAngRefOutpostMode()
{
  GimbalYawSearch(cfg_.GimbalYaw.min_ang_outpost, cfg_.GimbalYaw.max_ang_outpost);
  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::Deg2Rad(cfg_.GimbalPitch.fix_ang_outpost);
};

void Gimbal::calcGimbalJointAngRefInholeMode()
{
  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::Deg2Rad(cfg_.GimbalPitch.fix_ang_search);
  joint_ang_ref_[kJointIdxGimbalYaw] = hello_world::Deg2Rad(0.0f);
}

void Gimbal::calcGimbalJointAngRefBuffMode()
{
  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::Deg2Rad(20.7f);
  joint_ang_ref_[kJointIdxGimbalYaw] = hello_world::Deg2Rad(0.0f);
}

void Gimbal::calcJointTorRef()
{
  calcGimbalJointTorRef();
};

void Gimbal::calcGimbalJointTorRef()
{
  JointIdx joint_idxs[kJointNum] = {kJointIdxGimbalPitch, kJointIdxGimbalYaw, kJointIdxGimbalRoll};
  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];

    float fdb = joint_ang_fdb_[joint_idx];
    float ref = joint_ang_ref_[joint_idx];

    Pid *pid_ptr = pid_ptr_[joint_idx];
    HW_ASSERT(pid_ptr != nullptr, "pointer to pid is nullptr");

    if (joint_idx == kJointIdxGimbalPitch) {
      float ffd = 0.45f;
      if (ref >= fdb + 0.05f) {
        ffd = 0.60f;
      }
      pid_ptr->calc(&ref, &fdb, &ffd, &joint_input_ref_[joint_idx]);
    } else if (joint_idx == kJointIdxGimbalYaw) {
      float ffd = 0.0f;  // ##TODO 每次更改辫子位置后重测
      if (ref > fdb + 0.05f) {
        if (fdb <= -0.43f) {
          ffd = 0.045f;
        } else if (fdb <= -0.26f) {
          ffd = 0.10f;
        } else if (fdb <= 0.18f) {
          ffd = 0.16f;
        }
      } else if (ref < fdb - 0.05f) {
        if (fdb >= 0.36f) {
          ffd = -0.13f;
        } else if (fdb >= 0.004f) {
          ffd = -0.27f;
        } else if (fdb >= -0.40f) {
          ffd = -0.29f;
        }
      }
      pid_ptr->calc(&ref, &fdb, &ffd, &joint_input_ref_[joint_idx]);
    } else if (joint_idx == kJointIdxGimbalRoll) {
      float ffd = 0.0f;
      pid_ptr->calc(&ref, &fdb, &ffd, &joint_input_ref_[joint_idx]);
    }

    // DEBUG:
    debug_fdb[i] = fdb;
    debug_ref[i] = ref;
    debug_out[i] = joint_input_ref_[joint_idx];
    debug_err[i] = debug_ref[i] - debug_fdb[i];
  }
}

#pragma endregion

#pragma region 数据重置

void Gimbal::reset()
{
  pwr_state_ = PwrState::kDead;  ///< 工作状态

  // 在 runOnWorking 函数中更新的数据
  gimbal_joint_working_mode_ = GimbalJointWorkingMode::kManual;       ///< 云台工作模式
  gimbal_joint_last_working_mode_ = GimbalJointWorkingMode::kManual;  ///< 云台上一工作模式

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_));            ///< Joint 关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_));            ///< Joint 关节速度反馈值
  memset(last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一控制周期关节角度期望值
  memset(joint_input_ref_, 0, sizeof(joint_input_ref_));        ///< 控制指令，基于关节力矩

  // 从电机中拿到的数据
  is_any_motor_pwr_on_ = false;  ///< 是否有电机上电
  is_all_motor_pwr_on_ = false;  ///< 是否所有电机上电

  memset(motor_ang_fdb_, 0, sizeof(motor_ang_fdb_));            ///< 电机角度反馈值
  memset(motor_spd_fdb_, 0, sizeof(motor_spd_fdb_));            ///< 电机速度反馈值
  memset(last_motor_spd_fdb_, 0, sizeof(last_motor_spd_fdb_));  ///< 上一次的电机速度反馈值

  resetPids();
}

void Gimbal::resetDataOnDead()
{
  // 在 runOnWorking 函数种更新的数据
  gimbal_joint_working_mode_ = GimbalJointWorkingMode::kManual;       ///< 云台工作模式
  gimbal_joint_last_working_mode_ = GimbalJointWorkingMode::kManual;  ///< 云台上一工作模式

  memset(joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(joint_ang_fdb_, 0, sizeof(joint_ang_fdb_));            ///< Joint 关节角度反馈值
  memset(joint_spd_fdb_, 0, sizeof(joint_spd_fdb_));            ///< Joint 关节速度反馈值
  memset(last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一控制周期关节角度期望值
  memset(joint_input_ref_, 0, sizeof(joint_input_ref_));

  resetPids();  ///< 控制指令，基于关节力矩
};

void Gimbal::resetDataOnResurrection()
{
  // 在 runOnWorking 函数种更新的数据
  gimbal_joint_working_mode_ = GimbalJointWorkingMode::kManual;       ///< 云台工作模式
  gimbal_joint_last_working_mode_ = GimbalJointWorkingMode::kManual;  ///< 云台上一工作模式

  JointIdx joint_idxs[kJointNum] = {kJointIdxGimbalPitch, kJointIdxGimbalYaw};
  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    joint_ang_ref_[joint_idx] = joint_ang_fdb_[joint_idx];
    last_joint_ang_ref_[joint_idx] = joint_ang_fdb_[joint_idx];
    memset(joint_input_ref_, 0, sizeof(joint_input_ref_));
  }
};

void Gimbal::resetPids()
{
  for (size_t i = 0; i < kJointNum; i++) {
    Pid *pid_ptr = pid_ptr_[i];
    HW_ASSERT(pid_ptr != nullptr, "pointer to pid is nullptr");
    pid_ptr->reset();
  }
};

#pragma endregion

#pragma region 通讯数据设置

void Gimbal::setCommData(bool working_flag)
{
  JointIdx joint_idxs[kJointNum] = {kJointIdxGimbalPitch, kJointIdxGimbalYaw};
  for (size_t i = 0; i < kJointNum; i++) {
    JointIdx joint_idx = joint_idxs[i];
    Motor *motor_ptr = motor_ptr_[joint_idx];
    HW_ASSERT(motor_ptr != nullptr, "pointer to motor %d is nullptr", joint_idx);

    if (working_flag && (!motor_ptr->isOffline())) {
      if (joint_idx == kJointIdxGimbalPitch) {
        motor_ptr->setMitInput(0, joint_input_ref_[joint_idx], 0, cfg_.GimbalPitch.pid_spd_kp, 0);
      } else if (joint_idx == kJointIdxGimbalYaw) {
        motor_ptr->setMitInput(0, joint_input_ref_[joint_idx], 0, cfg_.GimbalYaw.pid_spd_kp, 0);
      } else if (joint_idx == kJointIdxGimbalRoll) {
        motor_ptr->setMitInput(0, joint_input_ref_[joint_idx], 0, 0, 0);
      }
    } else {
      pid_ptr_[joint_idx]->reset();
      motor_ptr->setInput(0.0f);
    }
  }
};

#pragma endregion

#pragma region 注册函数

void Gimbal::registerMotor(Motor *ptr, JointIdx idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "joint index %d is out of range", idx);
  motor_ptr_[idx] = ptr;
};

void Gimbal::registerPid(Pid *ptr, JointIdx idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kJointNum, "joint index %d is out of range", idx);
  pid_ptr_[idx] = ptr;
};

#pragma endregion

#pragma region 工具函数

void Gimbal::GimbalYawSearch(float min, float max)
{
  float tmp_gimbal_yaw_ang_ref = last_joint_ang_ref_[kJointIdxGimbalYaw];
  float sensitivity_yaw = cfg_.GimbalYaw.sensitivity_search;

  // Yaw 轴限位
  bool is_yaw_ang_too_large = motor_ang_fdb_[kJointIdxGimbalYaw] > max - 0.05f;
  bool is_yaw_ang_too_small = motor_ang_fdb_[kJointIdxGimbalYaw] < min + 0.05f;

  if (is_yaw_ang_too_large && search_gimbal_yaw_move_dir_ == kDirPos) {
    search_gimbal_yaw_move_dir_ = kDirNeg;
  } else if (is_yaw_ang_too_small && search_gimbal_yaw_move_dir_ == kDirNeg) {
    search_gimbal_yaw_move_dir_ = kDirPos;
  }

  tmp_gimbal_yaw_ang_ref += static_cast<float>(search_gimbal_yaw_move_dir_) * sensitivity_yaw;
  joint_ang_ref_[kJointIdxGimbalYaw] = hello_world::AngleNormRad(tmp_gimbal_yaw_ang_ref);
}

void Gimbal::GimbalPitchSearch(float min, float max)
{
  float tmp_gimbal_pitch_ang_ref = last_joint_ang_ref_[kJointIdxGimbalPitch];
  float sensitivity_pitch = cfg_.GimbalPitch.sensitivity_search;

  // Pitch 轴限位
  bool is_pitch_ang_too_large = motor_ang_fdb_[kJointIdxGimbalPitch] > max - 0.02f;
  bool is_pitch_ang_too_small = motor_ang_fdb_[kJointIdxGimbalPitch] < min + 0.02f;

  if (is_pitch_ang_too_large && search_gimbal_pitch_move_dir_ == kDirPos) {
    search_gimbal_pitch_move_dir_ = kDirNeg;
  } else if (is_pitch_ang_too_small && search_gimbal_pitch_move_dir_ == kDirNeg) {
    search_gimbal_pitch_move_dir_ = kDirPos;
  }

  tmp_gimbal_pitch_ang_ref += static_cast<float>(search_gimbal_pitch_move_dir_) * sensitivity_pitch * 3;
  joint_ang_ref_[kJointIdxGimbalPitch] = hello_world::AngleNormRad(tmp_gimbal_pitch_ang_ref);
}

Gimbal::VisionCmdStatus Gimbal::isVisCmdVaild(float pitch, float yaw)
{
  if (pitch > cfg_.GimbalPitch.max_ang_motor || pitch < cfg_.GimbalPitch.min_ang_motor) {
    return kPitchOutOfRange;
  }
  if (yaw > cfg_.GimbalYaw.max_ang_motor) {
    return kYawBigerThanRange;
  } else if (yaw < cfg_.GimbalYaw.min_ang_motor) {
    return kYawSmallerThanRange;
  }
  return kVaild;
}

bool Gimbal::isVisCmdInRange(float pitch, float yaw)
{
  if (pitch > cfg_.GimbalPitch.max_ang_motor || pitch < cfg_.GimbalPitch.min_ang_motor) {
    return false;
  }
  if (yaw > cfg_.GimbalYaw.max_ang_motor || yaw < cfg_.GimbalYaw.min_ang_motor) {
    return false;
  }
  return true;
}

#pragma endregion

}  // namespace robot
