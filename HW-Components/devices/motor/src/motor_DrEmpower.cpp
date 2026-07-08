/**
 *******************************************************************************
 * @file      : motor_DrEmpower.cpp
 * @brief     : 大然电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025-03-10      Vamper          1. 完成正式版
 *******************************************************************************
 * @attention :
 * 1. 请先查看 motor_base.hpp 中的注意事项
 * 2. 电机ID范围为 1 ~ 63
 * 3. 目前仅可用输出类型 InputType::kTorq
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_DrEmpower.hpp"

#include <cstring>

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const MotorBaseInfo kDrEmpowerMotorBaseInfo{
    .raw_input_lim = kInvalidValue,
    .torq_input_lim = 5.0f,
    .curr_input_lim = kInvalidValue,
    .torq_const = kInvalidValue,
    .redu_rat = 1.0f,
    .angle_rat = kInvalidValue,
    .vel_rat = kInvalidValue,
    .curr_rat = kInvalidValue,
    .torq_rat = kInvalidValue,
    .cross_0_value = static_cast<uint16_t>(kInvalidValue),
    .raw_mapping_type = RawMappingType::kTorq,
};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

HW_OPTIMIZE_O2_START

DrEmpower::DrEmpower(uint8_t id, const OptionalParams &opt)
    : Motor(opt.offline_tick_thres)
{
#pragma region
  HW_ASSERT(1 <= id && id <= 63, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev, "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kTorq, "Error input type: %d",
            opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0, "Error external reduction ration: %f",
            opt.ex_redu_rat);
  HW_ASSERT(opt.max_torq_input_lim > 0, "Error max torque input limit: %f",
            opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0, "Error max current input limit: %f",
            opt.max_curr_input_lim);
#pragma endregion

  motor_info_ = kDrEmpowerMotorBaseInfo;
  motor_info_.tx_id = ((id << 5) + 0x1d);
  motor_info_.rx_id = ((id << 5) + 0x01);
  motor_info_.rx_ids = {motor_info_.rx_id, (uint32_t)((id << 5) + 0x03),
                        (uint32_t)((id << 5) + 0x05),
                        (uint32_t)((id << 5) + 0x07)};
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }

  motor_info_.torq_input_lim = max_torq_input;
}

bool DrEmpower::decode(size_t len, const uint8_t *data, uint32_t rx_id)
{
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion
  if (len != 8) {
    decode_fail_cnt_++;
    return false;
  }
  float actual_ang = 0;
  memcpy(&actual_ang, data, sizeof(float));
  actual_angle_ = actual_ang / 360.0f * 2 * PI;
  angle_ = normAngle(actual_angle_ - motor_info_.angle_offset);
  vel_ = (int16_t)((data[5] << 8) + data[4]) * 0.01f / 60 * 2 * PI;
  torq_ = (int16_t)((data[7] << 8) + data[6]) * 0.01f;
  angle_ = angle_ * motor_info_.dir;
  vel_ = vel_ * motor_info_.dir;
  torq_ = torq_ * motor_info_.dir;

  oc_.update();

  decode_success_cnt_++;
  is_update_ = true;
  if (update_cb_) {
    update_cb_();
  }
  return true;
}

bool DrEmpower::encode(size_t &len, uint8_t *data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
  HW_ASSERT(len, "len is nullptr");
#pragma endregion

  if (len != 8) {
    encode_fail_cnt_++;
    return false;
  }
  if (is_enable_) {
    motor_info_.tx_id = ((motor_info_.id << 5) + 0x1f);
    unsigned short param_address = 22001;
    int8_t param_type = 3;
    uint32_t value = 1;
    data[0] = (uint8_t)(param_address);
    data[1] = (uint8_t)(param_address >> 8);
    data[2] = (uint8_t)(param_type);
    data[3] = (uint8_t)(param_type >> 8);
    data[4] = (uint8_t)(value);
    data[5] = (uint8_t)(value >> 8);
    data[6] = (uint8_t)(value >> 16);
    data[7] = (uint8_t)(value >> 24);
    is_enable_ = false;
  } else {
    motor_info_.tx_id = ((motor_info_.id << 5) + 0x1d);
    int u16_input_mode, s16_ramp_rate;
    u16_input_mode = 1;
    s16_ramp_rate = 0;
    torq_input_ = torq_input_ * motor_info_.dir;
    memcpy(data, &torq_input_, sizeof(torq_input_));
    data[4] = s16_ramp_rate;
    data[5] = s16_ramp_rate >> 8;
    data[6] = u16_input_mode;
    data[7] = u16_input_mode >> 8;
  }
  encode_success_cnt_++;
  return true;
}

Status DrEmpower::setInput(float input)
{
  switch (motor_info_.input_type) {
    case InputType::kTorq:
      torq_input_ = hello_world::Bound(input, motor_info_.torq_input_lim,
                                       -motor_info_.torq_input_lim);
      if (fabsf(torq_input_) < fabsf(input)) {
        return Status::kInputValueOverflow;
      } else {
        return Status::kOk;
      }
      break;
    default:
      return Status::kInputTypeError;
  }
}

Status DrEmpower::set_input_type(InputType input_type)
{
  if (input_type == InputType::kTorq) {
    motor_info_.input_type = input_type;
    return Status::kOk;
  } else {
    return Status::kInputTypeError;
  }
}

void DrEmpower::init(uint8_t id, const OptionalParams &opt)
{
#pragma region
  HW_ASSERT(1 <= id && id <= 63, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev, "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kTorq, "Error input type: %d",
            opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0, "Error external reduction ration: %f",
            opt.ex_redu_rat);
  HW_ASSERT(opt.max_torq_input_lim > 0, "Error max torque input limit: %f",
            opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0, "Error max current input limit: %f",
            opt.max_curr_input_lim);
#pragma endregion

  motor_info_ = kDrEmpowerMotorBaseInfo;

  motor_info_.tx_id = (id << 5) + 0x1d;
  motor_info_.rx_id = (id << 5) + 0x01;
  motor_info_.rx_ids = {motor_info_.rx_id, (uint32_t)((id << 5) + 0x03),
                        (uint32_t)((id << 5) + 0x05),
                        (uint32_t)((id << 5) + 0x07)};
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }
  motor_info_.torq_input_lim = max_torq_input;
  motor_info_.curr_input_lim = max_curr_input;

  angle_ = 0.0f;
  vel_ = 0.0f;
  vel_raw_ = 0.0f;
  torq_ = 0.0f;
  curr_ = 0.0f;
  round_ = 0.0f;
  actual_angle_ = 0.0f;

  is_update_ = false;
  update_cb_ = nullptr;
  oc_.init(opt.offline_tick_thres);

  raw_input_ = 0;
  torq_input_ = 0;
  curr_input_ = 0;

  td_ptr_ = nullptr;

  decode_success_cnt_ = 0;
  decode_fail_cnt_ = 0;
  encode_success_cnt_ = 0;
  encode_fail_cnt_ = 0;
  transmit_success_cnt_ = 0;

  is_enable_ = false;
}

HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world