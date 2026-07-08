/**
 *******************************************************************************
 * @file              :motor_JC2804.cpp
 * @brief             :JC2804电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025-03-18      <Vamper>        1。<note>
 *******************************************************************************
 * @attention :
 * 1. 请先查看 motor_base.hpp 中的注意事项
 * 2. 电机ID范围为1-63
 * 3. 目前仅可用输出类型InputType::kTorq
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_JC2804.hpp"

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

static const MotorBaseInfo kJC2804MotorBaseInfo{
    .raw_input_lim = kInvalidValue,
    .torq_input_lim = 5.0f,
    .curr_input_lim = kInvalidValue,
    .torq_const = 0.68f,
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

JC2804::JC2804(uint8_t id, const OptionalParams &opt)
    : Motor(opt.offline_tick_thres)
{
#pragma region
  HW_ASSERT(1 <= id && id <= 127, "Error id: %d", id);
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

  motor_info_ = kJC2804MotorBaseInfo;
  motor_info_.tx_id = (0x600 + id);
  motor_info_.rx_id = (0x580 + id);
  motor_info_.rx_ids = {motor_info_.rx_id};
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  motor_info_.torq_input_lim = PI;
}

JC2804 &JC2804::operator=(const JC2804 &other)
{
  if (this != &other) {
    Motor::operator=(other);
  }
  return *this;
}

JC2804::JC2804(JC2804 &&other) : Motor(std::move(other)) {}

JC2804 &JC2804::operator=(JC2804 &&other)
{
  if (this != &other) {
    Motor::operator=(std::move(other));
  }
  return *this;
}

bool JC2804::decode(size_t len, const uint8_t *data, uint32_t rx_id)
{
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion
  if (len != 8) {
    decode_fail_cnt_++;
    return false;
  }

  if (data[0] == 0x60 && data[1] == 0x00 && data[2] == 0x23 &&
      data[3] == 0x00 && data[4] == 0x00 && data[5] == 0x00 &&
      data[6] == 0x00 && data[7] == 0x00) {
    oc_.update();

    decode_success_cnt_++;
    is_update_ = true;
    if (update_cb_) {
      update_cb_();
    }
    return true;
  } else if (data[0] == 0x43 && data[1] == 0x00 && data[2] == 0x08 &&
             data[3] == 0x00) {
    int32_t angle_raw = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) |
                        data[7];
    angle_ = static_cast<float>(angle_raw) / 100.0f;
    oc_.update();

    decode_success_cnt_++;
    is_update_ = true;
    if (update_cb_) {
      update_cb_();
    }
    return true;
  } else {
    return false;
  }
}

bool JC2804::encode(size_t &len, uint8_t *data)
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

  if (is_get_angle_ == false) {
    angle_encode(len, data);
  } else if (is_get_angle_ == true) {
    get_angle_encode(len, data);
  }

  encode_success_cnt_++;

  return true;
}

void JC2804::angle_encode(size_t &len, uint8_t *data)
{
  motor_info_.tx_id = (0x600 + motor_info_.id);
  int32_t torq_input = (int32_t)(torq_input_ / PI * 180 * 100);

  data[0] = 0x23;
  data[1] = 0x00;
  data[2] = 0x23;
  data[3] = 0x00;
  data[4] = torq_input >> 24 & 0xFF;
  data[5] = torq_input >> 16 & 0xFF;
  data[6] = torq_input >> 8 & 0xFF;
  data[7] = torq_input & 0xFF;
}

void JC2804::get_angle_encode(size_t &len, uint8_t *data)
{
  motor_info_.tx_id = (0x600 + motor_info_.id);
  data[0] = 0x43;
  data[1] = 0x00;
  data[2] = 0x08;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
  is_get_angle_ = false;
}

Status JC2804::setInput(float input)
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

Status JC2804::set_input_type(InputType input_type)
{
  if (input_type == InputType::kTorq) {
    motor_info_.input_type = input_type;
    return Status::kOk;
  } else {
    return Status::kInputTypeError;
  }
}

void JC2804::init(uint8_t id, const OptionalParams &opt)
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

  motor_info_ = kJC2804MotorBaseInfo;

  motor_info_ = kJC2804MotorBaseInfo;
  motor_info_.tx_id = (0x600 + id);
  motor_info_.rx_id = (0x580 + id);
  motor_info_.rx_ids = {motor_info_.rx_id};
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  motor_info_.torq_input_lim = PI;

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
}

HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world