/**
 *******************************************************************************
 * @file      : VTM_Receiver.cpp
 * @brief     : RoboMaster 裁判系统相机图传模块接收端类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2025-04-19      Jinletian       1. 完成测试版
 *******************************************************************************
 * @attention :
 *  该类依赖串口接收管理器 UartRxMgr，使用前请确保 UartRxMgr 按要求配置于初始化，其中
 *  串口波特率设置为 921600，字长 8 Bits(include Parity)，无校验，停止位 1，只接收。
 *  串口接收管理器中 buf_len 设置为 VTM::kRcRxDataLen_ + 1 即 22
 *  max_process_data_len 设置为 VTM::kRcRxDataLen_ 即 21
 *  eof_type 设置为 EofType::kIdle
 *  发送端与接收端对频连接后，发送端每间隔 14ms 发送一帧 21 字节的数据
 *  接收端连接选手客户端后，图传链路同时在该串口发送数据，可能会造成 decode_fail_cnt_ 增加，
 *  其中：键鼠包速率约 30Hz，自定义控制器包速率约 30Hz。用户可以另注册referee进行解包。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "VTM_Receiver.hpp"

#include <cstring>

#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace remote_control
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint16_t kCrc16Init = 0xffff;
static const uint16_t kCrc16Tab[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

VTM &VTM::operator=(const VTM &other)
{
  if (this != &other) {
    rc_lv_ = other.rc_lv_;
    rc_lh_ = other.rc_lh_;
    rc_rv_ = other.rc_rv_;
    rc_rh_ = other.rc_rh_;
    rc_wheel_ = other.rc_wheel_;

    trigger_ = other.trigger_;
    pause_btn_ = other.pause_btn_;
    custom_l_btn_ = other.custom_l_btn_;
    custom_r_btn_ = other.custom_r_btn_;

    rc_switch_ = other.rc_switch_;
    last_rc_switch_ = other.last_rc_switch_;

    mouse_l_btn_ = other.mouse_l_btn_;
    mouse_r_btn_ = other.mouse_r_btn_;
    mouse_m_btn_ = other.mouse_m_btn_;
    mouse_x_ = other.mouse_x_;
    mouse_y_ = other.mouse_y_;
    mouse_z_ = other.mouse_z_;

    key_ = other.key_;

    raw_rx_frame_ = other.raw_rx_frame_;

    oc_ = other.oc_;

    is_updated_ = other.is_updated_;
    update_cb_ = other.update_cb_;
    rx_ids_ = other.rx_ids_;

    decode_success_cnt_ = other.decode_success_cnt_;
    decode_fail_cnt_ = other.decode_fail_cnt_;
    crc_fail_cnt_ = other.crc_fail_cnt_;
  }

  return *this;
}

VTM::VTM(VTM &&other)
{
  rc_lv_ = other.rc_lv_;
  rc_lh_ = other.rc_lh_;
  rc_rv_ = other.rc_rv_;
  rc_rh_ = other.rc_rh_;
  rc_wheel_ = other.rc_wheel_;

  trigger_ = other.trigger_;
  pause_btn_ = other.pause_btn_;
  custom_l_btn_ = other.custom_l_btn_;
  custom_r_btn_ = other.custom_r_btn_;

  rc_switch_ = other.rc_switch_;
  last_rc_switch_ = other.last_rc_switch_;

  mouse_l_btn_ = other.mouse_l_btn_;
  mouse_r_btn_ = other.mouse_r_btn_;
  mouse_m_btn_ = other.mouse_m_btn_;
  mouse_x_ = other.mouse_x_;
  mouse_y_ = other.mouse_y_;
  mouse_z_ = other.mouse_z_;

  key_ = other.key_;

  raw_rx_frame_ = other.raw_rx_frame_;

  oc_ = std::move(other.oc_);

  is_updated_ = other.is_updated_;
  update_cb_ = other.update_cb_;
  rx_ids_ = other.rx_ids_;

  decode_success_cnt_ = other.decode_success_cnt_;
  decode_fail_cnt_ = other.decode_fail_cnt_;
  crc_fail_cnt_ = other.crc_fail_cnt_;
}

VTM &VTM::operator=(VTM &&other)
{
  if (this != &other) {
    rc_lv_ = other.rc_lv_;
    rc_lh_ = other.rc_lh_;
    rc_rv_ = other.rc_rv_;
    rc_rh_ = other.rc_rh_;
    rc_wheel_ = other.rc_wheel_;

    trigger_ = other.trigger_;
    pause_btn_ = other.pause_btn_;
    custom_l_btn_ = other.custom_l_btn_;
    custom_r_btn_ = other.custom_r_btn_;

    rc_switch_ = other.rc_switch_;
    last_rc_switch_ = other.last_rc_switch_;

    mouse_l_btn_ = other.mouse_l_btn_;
    mouse_r_btn_ = other.mouse_r_btn_;
    mouse_m_btn_ = other.mouse_m_btn_;
    mouse_x_ = other.mouse_x_;
    mouse_y_ = other.mouse_y_;
    mouse_z_ = other.mouse_z_;

    key_ = other.key_;

    raw_rx_frame_ = other.raw_rx_frame_;

    oc_ = std::move(other.oc_);

    is_updated_ = other.is_updated_;
    update_cb_ = other.update_cb_;
    rx_ids_ = other.rx_ids_;

    decode_success_cnt_ = other.decode_success_cnt_;
    decode_fail_cnt_ = other.decode_fail_cnt_;
    crc_fail_cnt_ = other.crc_fail_cnt_;
  }

  return *this;
}

bool VTM::decode(size_t len, const uint8_t *data, uint32_t rx_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data != nullptr, "Error data pointer");
#pragma endregion

  if (len != kRcRxDataLen_) {
    decode_fail_cnt_++;
    return false;
  }

  memcpy(&raw_rx_frame_, data, kRcRxDataLen_);

  if (raw_rx_frame_.sof_1 != 0xA9 || raw_rx_frame_.sof_2 != 0x53) {
    /* 帧头不匹配 */
    decode_fail_cnt_++;
    return false;
  }

  if (getCrc16CheckNum(data, kRcRxDataLen_ - 2, kCrc16Init) !=
      raw_rx_frame_.crc16) {
    /* CRC 校验失败 */
    decode_fail_cnt_++;
    crc_fail_cnt_++;
    return false;
  }

  rc_rh_ = Bound(-1.0, 1.0, (raw_rx_frame_.ch_0 - kRcOffset_) * kRcRatio_);
  rc_rv_ = Bound(-1.0, 1.0, (raw_rx_frame_.ch_1 - kRcOffset_) * kRcRatio_);
  rc_lh_ = Bound(-1.0, 1.0, (raw_rx_frame_.ch_2 - kRcOffset_) * kRcRatio_);
  rc_lv_ = Bound(-1.0, 1.0, (raw_rx_frame_.ch_3 - kRcOffset_) * kRcRatio_);

  last_rc_switch_ = rc_switch_;
  rc_switch_ = static_cast<SwitchState>(raw_rx_frame_.mode_sw);
  pause_btn_ = raw_rx_frame_.pause;
  custom_l_btn_ = raw_rx_frame_.fn_1;
  custom_r_btn_ = raw_rx_frame_.fn_2;
  rc_wheel_ = Bound(-1.0, 1.0, (raw_rx_frame_.wheel - kRcOffset_) * kRcRatio_);
  trigger_ = raw_rx_frame_.trigger;
  mouse_x_ = raw_rx_frame_.mouse_x;
  mouse_y_ = raw_rx_frame_.mouse_y;
  mouse_z_ = raw_rx_frame_.mouse_z;
  mouse_l_btn_ = raw_rx_frame_.mouse_left;
  mouse_r_btn_ = raw_rx_frame_.mouse_right;
  mouse_m_btn_ = raw_rx_frame_.mouse_middle;
  key_.data = raw_rx_frame_.key;

  oc_.update();

  decode_success_cnt_++;
  is_updated_ = true;
  if (update_cb_) {
    update_cb_();
  }
  return true;
}
/* Private function definitions ----------------------------------------------*/

inline uint16_t VTM::getCrc16CheckNum(const uint8_t *head, uint16_t length, uint16_t crc16)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(head != nullptr, "Error data pointer");
#pragma endregion
  uint8_t data;
  while (length--) {
    data = *head++;
    crc16 = ((uint16_t)(crc16) >> 8) ^
            kCrc16Tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
  }

  return crc16;
}

HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world
