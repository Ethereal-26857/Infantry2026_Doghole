/**
 *******************************************************************************
 * @file      : bus_servo.cpp
 * @brief     : 总线舵机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025-04-07      Jinletian       1. 完成测试版
 *******************************************************************************
 * @attention :
 * 1. 总线舵机依赖 uart 口 5V 供电，使用时请确保供电正常。
 * 2. 该类依赖串口接收管理器 UartRxMgr，使用前请确保 UartRxMgr 按要求配置于初始化，其中
 *    串口波特率设置为 115200 bps，字长 8 Bits(include Parity)，无校验，停止位 1。
 *    串口接收管理器中 buf_len 设置为 kBusServoRxDataMaxLen + 1，max_process_data_len
 *    设置为 kBusServoRxDataMaxLen，eof_type 设置为 EofType::kIdle
 *    串口发送管理器中 buf_len 建议设置不小于 kBusServoTxDataMaxLen。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "bus_servo.hpp"

#include <cstdio>

#include "tick.hpp"

namespace hello_world
{
namespace servo
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
HW_OPTIMIZE_O2_START

bool BusServo::encode(size_t &len, uint8_t *data)
{
  if (len == 0 || data == nullptr) {
    encode_fail_cnt_++;
    return false;
  }

  if (info_.auto_enable &&
      last_msg_type_ == SendMsgType::kEnable) {
    // 防止自动使能吃掉第一帧控制指令
    tx_data_.msg_type = SendMsgType::kCtrl;
  }

  if (tx_data_.msg_type == SendMsgType::kCtrl &&
      info_.auto_enable &&
      !status_.enable) {
    // 自动使能
    tx_data_.msg_type = SendMsgType::kEnable;
  }

  switch (tx_data_.msg_type) {
    case SendMsgType::kCtrl:
      if (len < kCtrlDataLen) {
        return false;
      }
      len = snprintf(reinterpret_cast<char *>(data), kCtrlDataLen, "#%03dP%04dT%04d!", info_.id, tx_data_.ang_ref, tx_data_.move_time);
      break;
    case SendMsgType::kGetAngle:
      if (len < kCmdDataLen) {
        return false;
      }
      len = snprintf(reinterpret_cast<char *>(data), kCmdDataLen, "#%03dPRAD!", info_.id);
      break;
    case SendMsgType::kDisable:
      if (len < kCmdDataLen) {
        return false;
      }
      len = snprintf(reinterpret_cast<char *>(data), kCmdDataLen, "#%03dPULK!", info_.id);
      break;
    case SendMsgType::kEnable:
      if (len < kCmdDataLen) {
        return false;
      }
      len = snprintf(reinterpret_cast<char *>(data), kCmdDataLen, "#%03dPULR!", info_.id);
      break;
  }

  last_msg_type_ = tx_data_.msg_type;
  tx_data_.msg_type = SendMsgType::kGetAngle;  // 默认获取角度
  memcpy(tx_data_buffer_, data, len);
  return true;
}

bool BusServo::decode(size_t len, const uint8_t *data, uint32_t rx_id)
{
  if (len == 0 || data == nullptr) {
    decode_fail_cnt_++;
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    rx_result_ = processByte(data[i]);
    switch (rx_result_) {
      case RxResult::kOk: {
        oc_.update();
        is_updated_ = true;
        if (update_cb_) {
          update_cb_();
        }
        decode_success_cnt_++;
        return true;
      }
      case RxResult::kErrNoDataInput:
      case RxResult::kErrTooLongData:
      case RxResult::kErrWrongId: {
        decode_fail_cnt_++;
        break;
      }
      default: {
        break;
      }
    }
  }

  return false;
}

/* Private function definitions ----------------------------------------------*/

BusServo::RxResult BusServo::processByte(uint8_t byte)
{
  RxResult result = RxResult::kErrNoDataInput;

  switch (rx_status_) {
    case RxStatus::kWaitingHeader: {
      resetDecodeProgress(true);
      if (byte == '#') {
        rx_data_buffer_[rx_data_buffer_idx_++] = byte;
        rx_status_ = RxStatus::kWaitingTail;
        result = RxResult::kHandlingWaitTail;
      } else {
        result = RxResult::kErrNoDataInput;
      }
      break;
    }
    case RxStatus::kWaitingTail: {
      if (byte == '!') {
        rx_data_buffer_[rx_data_buffer_idx_++] = byte;
        bool decode_result = decodeRxData();
        resetDecodeProgress();
        if (decode_result) {
          result = RxResult::kOk;
        } else {
          result = RxResult::kErrWrongId;
        }
      } else if (rx_data_buffer_idx_ < kBusServoRxDataMaxLen) {
        rx_data_buffer_[rx_data_buffer_idx_++] = byte;
        result = RxResult::kHandlingWaitTail;
      } else {
        resetDecodeProgress(true);
        result = RxResult::kErrTooLongData;
      }
      break;
    }
  }
  return result;
}

void BusServo::resetDecodeProgress(bool keep_rx_buffer)
{
  rx_status_ = RxStatus::kWaitingHeader;
  if (!keep_rx_buffer) {
    memset(rx_data_buffer_, 0, sizeof(rx_data_buffer_));
  }
  rx_data_buffer_idx_ = 0;
}

bool BusServo::decodeRxData(void)
{
  if (rx_data_buffer_[1] == 'O' && rx_data_buffer_[2] == 'K') {
    // 反馈信息为 OK
    if (last_msg_type_ == SendMsgType::kEnable) {
      status_.enable = true;
    } else if (last_msg_type_ == SendMsgType::kDisable) {
      status_.enable = false;
    }
    return true;
  }

  if (str2uint(rx_data_buffer_ + 1, 3) != info_.id) {
    // ID 不匹配，丢弃数据
    return false;
  }

  /* 位置数据 */
  if (rx_data_buffer_idx_ == 10) {
    status_.ang_fdb = str2uint(rx_data_buffer_ + 5, 4);
    status_.ang_fdb_deg =
        static_cast<float>(status_.ang_fdb - 1500) / 2000 * static_cast<float>(info_.angle_range);
    status_.ang_fdb_rad = Deg2Rad(status_.ang_fdb_deg);
  }
  return true;
}

uint16_t BusServo::str2uint(const uint8_t *str, size_t len)
{
  uint16_t value = 0;
  for (size_t i = 0; i < len; i++) {
    value = value * 10 + (str[i] - '0');
  }
  return value;
}

HW_OPTIMIZE_O2_END
}  // namespace servo
}  // namespace hello_world
