/**
 *******************************************************************************
 * @file       : motor_LingKong.hpp
 * @brief      : 瓴控电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025-04-12      Jinletian       1.整合不同型号
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~4，其中 1~4 为同一条发送报文
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq 和 InputType::kCurr
 *  4. 电机为一发一收的通信方式，不给电机发送指令时电机不会反馈数据
 *  5. 可额外获得的数据为电机温度 temp
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_LINGKONG_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_LINGKONG_HPP_
/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

#pragma region 瓴控基类
class LingKong : public Motor
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  LingKong(void) = default;
  /**
   * @brief       瓴控电机初始化
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit LingKong(
      uint8_t id, const OptionalParams& opt = OptionalParams());
  LingKong(const LingKong&) = default;
  LingKong& operator=(const LingKong& other) = default;
  LingKong(LingKong&& other) = default;
  LingKong& operator=(LingKong&& other) = default;

  virtual ~LingKong(void) = default;

  /* 重载方法 */

  /**
   * @brief       将电调发回的 CAN 报文进行解包
   * @param        len: 报文长度
   * @param        data: 电调发回的 CAN 报文
   * @param        rx_id: 接收 ID
   * @retval       是否解包成功
   * @note        请前判断 rx_id 是否符合再进行解码
   */
  virtual bool decode(size_t len, const uint8_t* data, uint32_t rx_id) override;

  /**
   * @brief       将要发给电调的期望输值编码为对应的 CAN 报文
   * @param        len: 传入缓冲区长度（必须为 8），返回报文长度
   * @param        data: 将要发出的 CAN 报文
   * @retval       是否编码成功
   * @note        请前判断 tx_id 是否符合再进行解码
   */
  virtual bool encode(size_t& len, uint8_t* data) override;

  /* 允许相同 ID 的发送端共同编译报文 */
  virtual bool allowSharedId(void) const override { return true; }

  /* 配置方法 */

  /**
   * @brief       MG6012E-i36 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams());

  /* 数据修改与获取 */

  uint8_t temp(void) const { return temp_; }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~4
   * @retval       对应的发送 ID
   * @note        None
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 4, "Error id: %d", id);
#pragma endregion

    return kTx1_4_ + id;
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~4
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 4, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 protected:
  /* 电机状态 */

  uint8_t temp_ = 0;  ///* 电机温度

  static constexpr uint32_t kTx1_4_ = 0x280;
  static constexpr uint32_t kRx0_ = 0x140;
};

static const MotorBaseInfo kMG6012Ei36MotorBaseInfo = {
    .raw_input_lim = 700,
    .torq_input_lim = 45.1f,
    .curr_input_lim = 11.279f,
    .torq_const = 45.1 / 11.279,
    .redu_rat = 1.0f,
    .angle_rat = 2 * PI / 0xFFFF,
    .vel_rat = PI / 180 / 36,
    .curr_rat = kInvalidValue,
    .torq_rat = 1.0f / 15.5440f,
    .cross_0_value = 0xFFFFU,
    .raw_mapping_type = RawMappingType::kTorq,
};
#pragma endregion

#pragma region MG6012E-i36
class MG6012Ei36 : public LingKong
{
 public:

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  MG6012Ei36(void) = default;

  /**
   * @brief       MG6012E-i36 初始化
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit MG6012Ei36(uint8_t id, const OptionalParams& opt = OptionalParams())
      : LingKong(id, opt)
  {
    motor_info_ = kMG6012Ei36MotorBaseInfo;
  }
  MG6012Ei36(const MG6012Ei36&) = default;
  MG6012Ei36& operator=(const MG6012Ei36& other) = default;
  MG6012Ei36(MG6012Ei36&& other) = default;
  MG6012Ei36& operator=(MG6012Ei36&& other) = default;

  virtual ~MG6012Ei36(void) = default;

  /**
   * @brief       MG6012E-i36 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams())
  {
    motor_info_ = kMG6012Ei36MotorBaseInfo;
    LingKong::init(id, opt);
  }
};
#pragma endregion

#pragma region MG8016E
static const MotorBaseInfo kMG8016EMotorBaseInfo = {
  .raw_input_lim = 2000,
  .torq_input_lim = 37.0f,
  .curr_input_lim = 33.0f,
  .torq_const = 0.24f * 6,
  .redu_rat = 1.0f,  ///* MG8016E 角度反馈会换算为输出端，且减速器（6:1）难以拆卸
  .angle_rat = 2 * PI / 0xFFFF,
  .vel_rat = PI / 180 / 6,
  .curr_rat = 32.0f / 2000,
  .torq_rat = kInvalidValue,
  .cross_0_value = 0xFFFFU,
  .raw_mapping_type = RawMappingType::kCurr,
};

class MG8016E : public LingKong
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  MG8016E(void) = default;

  /**
   * @brief       MG8016E 初始化
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit MG8016E(uint8_t id, const OptionalParams& opt = OptionalParams())
      : LingKong(id, opt)
  {
    motor_info_ = kMG8016EMotorBaseInfo;
  }
  MG8016E(const MG8016E&) = default;
  MG8016E& operator=(const MG8016E& other) = default;
  MG8016E(MG8016E&& other) = default;
  MG8016E& operator=(MG8016E&& other) = default;

  virtual ~MG8016E(void) = default;

  /**
   * @brief       MG8016E 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams())
  {
    motor_info_ = kMG8016EMotorBaseInfo;
    LingKong::init(id, opt);
  }
};
#pragma endregion

#pragma region MF9025v2
static const MotorBaseInfo kMF9025v2MotorBaseInfo = {
  .raw_input_lim = 2000,
  .torq_input_lim = 11.0f,
  .curr_input_lim = 33.0f,
  .torq_const = 0.32f,
  .redu_rat = 1.0f,
  .angle_rat = 2 * PI / 0xFFFF,
  .vel_rat = PI / 180,
  .curr_rat = 32.0f / 2000,
  .torq_rat = kInvalidValue,
  .cross_0_value = 0xFFFFU,
  .raw_mapping_type = RawMappingType::kCurr,
};

class MF9025v2 : public LingKong
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  MF9025v2(void) = default;

  /**
   * @brief       MF9025v2 初始化
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit MF9025v2(uint8_t id, const OptionalParams& opt = OptionalParams())
      : LingKong(id, opt)
  {
    motor_info_ = kMF9025v2MotorBaseInfo;
  }
  MF9025v2(const MF9025v2&) = default;
  MF9025v2& operator=(const MF9025v2& other) = default;
  MF9025v2(MF9025v2&& other) = default;
  MF9025v2& operator=(MF9025v2&& other) = default;

  virtual ~MF9025v2(void) = default;

  /**
   * @brief       MG6012E-i36 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams())
  {
    motor_info_ = kMF9025v2MotorBaseInfo;
    LingKong::init(id, opt);
  }
};
#pragma endregion

#pragma region MG5010E-i36
static const MotorBaseInfo kMG5010Ei36MotorBaseInfo = {
  .raw_input_lim = 2000,
  .torq_input_lim = 25.0f,
  .curr_input_lim = 6.25f,
  .torq_const = 4.0,        // 待确认
  .redu_rat = 1.0f,
  .angle_rat = 2 * PI / 0xFFFF,
  .vel_rat = PI / 180 / 36,
  .curr_rat = 4.65 / 1050,
  .torq_rat = kInvalidValue,
  .cross_0_value = 0xFFFFU,
  .raw_mapping_type = RawMappingType::kCurr,
};

class MG5010Ei36 : public LingKong
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  MG5010Ei36(void) = default;

  /**
   * @brief       MG5010E-i36 初始化
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit MG5010Ei36(uint8_t id, const OptionalParams& opt = OptionalParams())
      : LingKong(id, opt)
  {
    motor_info_ = kMG5010Ei36MotorBaseInfo;
  }
  MG5010Ei36(const MG5010Ei36&) = default;
  MG5010Ei36& operator=(const MG5010Ei36& other) = default;
  MG5010Ei36(MG5010Ei36&& other) = default;
  MG5010Ei36& operator=(MG5010Ei36&& other) = default;

  virtual ~MG5010Ei36(void) = default;

  /**
   * @brief       MG5010E-i36 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams())
  {
    motor_info_ = kMG5010Ei36MotorBaseInfo;
    LingKong::init(id, opt);
  }
};
#pragma endregion

#pragma region MG4005-i10
static const MotorBaseInfo kMG4005i10MotorBaseInfo = {
  .raw_input_lim = 1000,
  .torq_input_lim = 3.0f,
  .curr_input_lim = 8.0f,
  .torq_const = 0.39f,
  .redu_rat = 10.0f,  // 减速比为 10:1，单编码器，无法保存零点
  .angle_rat = 2 * PI / 0xFFFF,
  .vel_rat = PI / 180,
  .curr_rat = kInvalidValue,
  .torq_rat = 3.0 / 1000.0,
  .cross_0_value = 0xFFFFU,
  .raw_mapping_type = RawMappingType::kTorq,
};

class MG4005i10 : public LingKong
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  MG4005i10(void) = default;

  /**
   * @brief       MG4005i10 初始化
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit MG4005i10(uint8_t id, const OptionalParams& opt = OptionalParams())
      : LingKong(id, opt)
  {
    motor_info_ = kMG4005i10MotorBaseInfo;
  }
  MG4005i10(const MG4005i10&) = default;
  MG4005i10& operator=(const MG4005i10& other) = default;
  MG4005i10(MG4005i10&& other) = default;
  MG4005i10& operator=(MG4005i10&& other) = default;

  virtual ~MG4005i10(void) = default;

  /**
   * @brief       MG4005i10 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~4
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams())
  {
    motor_info_ = kMG4005i10MotorBaseInfo;
    LingKong::init(id, opt);
  }
};
#pragma endregion

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_LINGKONG_HPP_ */
