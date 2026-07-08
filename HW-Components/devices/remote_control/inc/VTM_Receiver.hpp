/**
 *******************************************************************************
 * @file      : VTM_Receiver.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REMOTE_CONTROL_VTM_HPP_
#define HW_COMPONENTS_DEVICES_REMOTE_CONTROL_VTM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "offline_checker.hpp"
#include "receiver.hpp"
#include "system.hpp"

namespace hello_world
{
namespace remote_control
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum class VTMSwitchState : uint8_t {
  kLeft = 0u,
  kMid = 1u,
  kRight = 2u,
  kErr = 3u,
};

class VTM : public comm::Receiver
{
 public:
  typedef VTMSwitchState SwitchState;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  VTM(void) = default;
  /**
   * @brief       构造函数
   * @param        offline_tick_thres: 离线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  VTM(uint32_t offline_tick_thres) : oc_(offline_tick_thres) {}
  VTM(const VTM &) = default;
  VTM &operator=(const VTM &other);
  VTM(VTM &&other);
  VTM &operator=(VTM &&other);

  virtual ~VTM(void) = default;

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return 0u; }

  virtual const RxIds &rxIds(void) const override { return rx_ids_; }

  /**
   * @brief       将接收到的数据解包
   * @param        len: 数据长度
   * @param        data: 接收到的数据
   * @param        rx_id: 接收 ID
   * @retval       解包成功返回 true，否则返回 false
   * @note        None
   */
  virtual bool decode(size_t len, const uint8_t *data, uint32_t rx_id) override;

  virtual bool isUpdate(void) const override { return is_updated_; }

  virtual void clearUpdateFlag(void) override { is_updated_ = false; }

  /**
   * @brief       注册更新回调函数
   * @param        cb: 回调函数指针，在 decode 函数解码成功后被调用，不使用时传入
   *               nullptr
   * @retval       None
   * @note        如注测封装好的看门狗刷新函数
   *
   */
  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  /* 配置方法 */

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param        offline_tick_thres: 离线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  void init(uint32_t offline_tick_thres) { oc_.init(offline_tick_thres); }

  /* 数据修改与获取 */

  float rc_lv(void) const { return rc_lv_; }
  float rc_lh(void) const { return rc_lh_; }
  float rc_rv(void) const { return rc_rv_; }
  float rc_rh(void) const { return rc_rh_; }
  float rc_wheel(void) const { return rc_wheel_; }

  SwitchState rc_switch(void) const { return rc_switch_; }
  SwitchState last_rc_switch(void) const { return last_rc_switch_; }

  bool trigger(bool reset = false)
  {
    bool tmp = trigger_;
    if (reset) {
      trigger_ = false;
    }
    return tmp;
  }
  bool pause_btn(bool reset = false)
  {
    bool tmp = pause_btn_;
    if (reset) {
      pause_btn_ = false;
    }
    return tmp;
  }
  bool custom_l_btn(bool reset = false)
  {
    bool tmp = custom_l_btn_;
    if (reset) {
      custom_l_btn_ = false;
    }
    return tmp;
  }
  bool custom_r_btn(bool reset = false)
  {
    bool tmp = custom_r_btn_;
    if (reset) {
      custom_r_btn_ = false;
    }
    return tmp;
  }

  int16_t mouse_x(void) const { return mouse_x_; }
  int16_t mouse_y(void) const { return mouse_y_; }
  int16_t mouse_z(void) const { return mouse_z_; }

  bool mouse_l_btn(bool reset = false)
  {
    bool tmp = mouse_l_btn_;
    if (reset) {
      mouse_l_btn_ = false;
    }
    return tmp;
  }
  bool mouse_r_btn(bool reset = false)
  {
    bool tmp = mouse_r_btn_;
    if (reset) {
      mouse_r_btn_ = false;
    }
    return tmp;
  }
  bool key_W(bool reset = false)
  {
    bool tmp = key_.W;
    if (reset) {
      key_.W = false;
    }
    return tmp;
  }
  bool key_S(bool reset = false)
  {
    bool tmp = key_.S;
    if (reset) {
      key_.S = false;
    }
    return tmp;
  }
  bool key_A(bool reset = false)
  {
    bool tmp = key_.A;
    if (reset) {
      key_.A = false;
    }
    return tmp;
  }
  bool key_D(bool reset = false)
  {
    bool tmp = key_.D;
    if (reset) {
      key_.D = false;
    }
    return tmp;
  }
  bool key_SHIFT(bool reset = false)
  {
    bool tmp = key_.SHIFT;
    if (reset) {
      key_.SHIFT = false;
    }
    return tmp;
  }
  bool key_CTRL(bool reset = false)
  {
    bool tmp = key_.CTRL;
    if (reset) {
      key_.CTRL = false;
    }
    return tmp;
  }
  bool key_Q(bool reset = false)
  {
    bool tmp = key_.Q;
    if (reset) {
      key_.Q = false;
    }
    return tmp;
  }
  bool key_E(bool reset = false)
  {
    bool tmp = key_.E;
    if (reset) {
      key_.E = false;
    }
    return tmp;
  }
  bool key_R(bool reset = false)
  {
    bool tmp = key_.R;
    if (reset) {
      key_.R = false;
    }
    return tmp;
  }
  bool key_F(bool reset = false)
  {
    bool tmp = key_.F;
    if (reset) {
      key_.F = false;
    }
    return tmp;
  }
  bool key_G(bool reset = false)
  {
    bool tmp = key_.G;
    if (reset) {
      key_.G = false;
    }
    return tmp;
  }
  bool key_Z(bool reset = false)
  {
    bool tmp = key_.Z;
    if (reset) {
      key_.Z = false;
    }
    return tmp;
  }
  bool key_X(bool reset = false)
  {
    bool tmp = key_.X;
    if (reset) {
      key_.X = false;
    }
    return tmp;
  }
  bool key_C(bool reset = false)
  {
    bool tmp = key_.C;
    if (reset) {
      key_.C = false;
    }
    return tmp;
  }
  bool key_V(bool reset = false)
  {
    bool tmp = key_.V;
    if (reset) {
      key_.V = false;
    }
    return tmp;
  }
  bool key_B(bool reset = false)
  {
    bool tmp = key_.B;
    if (reset) {
      key_.B = false;
    }
    return tmp;
  }

  bool isKeyboardPressed(void) const { return key_.data != 0u; }
  bool isMousePressed(void) const { return mouse_l_btn_ || mouse_r_btn_ || mouse_m_btn_; }
  bool isMouseMoved(void) const
  {
    return mouse_x_ != 0 || mouse_y_ != 0 || mouse_z_ != 0;
  }
  bool isUsingKeyboardMouse(void) const
  {
    return isKeyboardPressed() || isMousePressed() || isMouseMoved();
  }
  bool isRcSwitchChanged(void) const
  {
    return (rc_switch_ != last_rc_switch_ &&
            last_rc_switch_ != SwitchState::kErr);
  }
  bool isRcMoved(void) const
  {
    return (rc_lv_ != 0) || (rc_lh_ != 0) || (rc_rv_ != 0) ||
           (rc_rh_ != 0) || (rc_wheel_ != 0);
  }
  bool isBtnPressed(void) const
  {
    return trigger_ || pause_btn_ || custom_l_btn_ || custom_r_btn_;
  }
  bool isUsingRc(void) const { return isRcSwitchChanged() || isRcMoved() || isBtnPressed(); }

  bool isOffline(void) { return oc_.isOffline(); }

  static const uint8_t kRcRxDataLen_ = 21u;  ///* 遥控器接收数据长度

 private:
  
  static inline uint16_t getCrc16CheckNum(const uint8_t *head, uint16_t length, uint16_t crc16);

  /* 遥控器数据 */

  float rc_lv_ = 0.0f;     ///* 遥控器左摇杆竖直值，[-1, 1]
  float rc_lh_ = 0.0f;     ///* 遥控器左摇杆水平值，[-1, 1]
  float rc_rv_ = 0.0f;     ///* 遥控器右摇杆竖直值，[-1, 1]
  float rc_rh_ = 0.0f;     ///* 遥控器右摇杆水平值，[-1, 1]
  float rc_wheel_ = 0.0f;  ///* 遥控器拨轮值，[-1, 1]

  bool trigger_ = false;       ///* 遥控器扳机
  bool pause_btn_ = false;     ///* 暂停按键
  bool custom_l_btn_ = false;  ///* 自定义按键（左）
  bool custom_r_btn_ = false;  ///* 自定义按键（右）

  SwitchState rc_switch_ = SwitchState::kErr;  ///* 遥控器挡位值
  /** 上一次接收到的遥控器挡位值 */
  SwitchState last_rc_switch_ = SwitchState::kErr;

  /* 键鼠数据 */

  bool mouse_l_btn_ = false;  ///* 鼠标左键是否按下
  bool mouse_r_btn_ = false;  ///* 鼠标右键是否按下
  bool mouse_m_btn_ = false;  ///* 鼠标中键是否按下
  int16_t mouse_x_ = 0u;      ///* 鼠标x轴数值
  int16_t mouse_y_ = 0u;      ///* 鼠标y轴数值
  int16_t mouse_z_ = 0u;      ///* 鼠标z轴数值

  union {
    uint16_t data = 0u;
    struct {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    };
  } key_;  ///* 键盘按键

  struct __packed RawRxFrame {
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    uint16_t key;
    uint16_t crc16;
  } raw_rx_frame_; ///* 原始数据

  OfflineChecker oc_ = OfflineChecker(100u);

  bool is_updated_ = false;  ///* 是否有新数据更新
  pUpdateCallback update_cb_ = nullptr;
  RxIds rx_ids_ = {0u};  ///* 接收端 ID 列表

  /** 接收状态统计 */

  uint32_t decode_success_cnt_ = 0u;  ///* 解码成功次数
  uint32_t decode_fail_cnt_ = 0u;     ///* 解码失败次数
  uint32_t crc_fail_cnt_ = 0u;        ///* CRC 校验失败次数

  static constexpr uint16_t kRcOffset_ = 1024u;
  static constexpr float kRcRatio_ = 1.0f / 660;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REMOTE_CONTROL_VTM_HPP_ */
