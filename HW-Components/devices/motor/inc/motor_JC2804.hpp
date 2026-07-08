/**
 *******************************************************************************
 * @file              :motor_JC2804.hpp
 * @brief             :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.0.0      2025-03-10      <Vamper>        1。<note>
 *******************************************************************************
 * @attention :
 * 1. 请先查看 motor_base.hpp 中的注意事项
 * 2. 电机ID范围为1-127
 * 3. 目前仅可用输出类型InputType::kTorq
 * 4. 由于电机的逆天设计，请设置为Torq输入的同时直接在INPUT内填入目标角度值
 * 5. 使用前请通过USB2CAN工具设置电机ID，上电后不再校准与自动闭环使能
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_JC2804_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_JC2804_HPP_
/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

#pragma region JC2804基类
class JC2804 : public Motor
{
 public:
  JC2804(void) = default;
  explicit JC2804(uint8_t id, const OptionalParams &opt = OptionalParams());
  JC2804(const JC2804 &) = default;
  JC2804 &operator=(const JC2804 &other);
  JC2804(JC2804 &&other);
  JC2804 &operator=(JC2804 &&other);
  virtual ~JC2804(void) = default;

  /* 配置方法 */

  /**
   * @brief       JC2804 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，0x01 ~ 0x7F
   * @param        opt: 电机可选配置参数
   * @param        auto_enable: 电机自动使能
   * @retval       None
   * @note         V0.0需要手动进入上位机设置ID
   *
   */
  void init(uint8_t id, const OptionalParams &opt = OptionalParams());

  /**
   * @brief      使能JC2804，令其可以发送回调数据
   * @param       None
   * @retval      None
   * @note        None
   */

  /* 方法重载 */

  /**
   * @brief       将电调发回的 CAN 报文进行解包
   * @param        len: 报文长度
   * @param        data: 电调发回的 CAN 报文
   * @retval       是否解包成功
   * @note        请前判断 rx_id 是否符合再进行解码
   */
  virtual bool decode(size_t len, const uint8_t *data, uint32_t rx_id) override;

  /**
   * @brief       将要发给电调的期望输值编码为对应的 CAN 报文
   * @param        len: 传入缓冲区长度（必须为 8），返回报文长度
   * @param        data: 将要发出的 CAN 报文
   * @retval       是否编码成功
   * @note        请前判断 tx_id 是否符合再进行解码
   */
  virtual bool encode(size_t &len, uint8_t *data) override;

  /**
   * @brief       设定发给电调的期望值
   * @param        input: 发给电调的期望值
   * @retval       设定状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   * @note        1. 期望值的物理意义与电机当前的输入类型有关，可使用
   * get_input_type 方法查看
   *              2.
   * 设定的期望值会自动被限制到允许的范围内，当前实际的设定值可以通过 getInput
   * 方法查看
   */
  virtual Status setInput(float input) override;

  /**
   * @brief       设置点击的输入类型
   * @param        input_type: 期望输入类型，可选值为：
   *   @arg        InputType::kTorq: 目前仅支持力矩输入
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设置成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        None
   */
  virtual Status set_input_type(InputType input_type) override;

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~7
   * @param        input_type: 电机输入类型
   * @retval       对应的发送 ID
   * @note        None
   */
  static uint32_t GetTxId(uint8_t id, InputType input_type)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
    HW_ASSERT(input_type == InputType::kTorq, "Error input type: %d",
              input_type);
#pragma endregion
    return ((id << 5) + 0x1d);
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~7
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
#pragma endregion

    return ((id << 5) + 0x01);
  }
  /**
   * @brief   对于角度控制encode
   **/
  void angle_encode(size_t &len, uint8_t *data);
  /**
   * @brief   对于读取角度encode
   **/
  void get_angle_encode(size_t &len, uint8_t *data);

  void get_angle_flag(void)
  {
    is_get_angle_ = true;
    return;
  }

 protected:
  bool is_get_angle_ = false;
};
}  // namespace motor
}  // namespace hello_world
#endif /*HW_COMPONENTS_DEVICES_MOTOR_MOTOR_JC2804_HPP_ */