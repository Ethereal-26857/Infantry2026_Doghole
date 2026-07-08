/**
 *******************************************************************************
 * @file      : bus_servo.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_SERVO_BUS_SERVO_HPP
#define HW_COMPONENTS_DEVICES_SERVO_BUS_SERVO_HPP

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "base.hpp"
#include "offline_checker.hpp"
#include "receiver.hpp"
#include "system.hpp"
#include "transmitter.hpp"

#include <cstring>

namespace hello_world
{
namespace servo
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

const size_t kBusServoRxDataMaxLen = 10u;  ///< 总线舵机最大反馈帧长度
const size_t kBusServoTxDataMaxLen = 16u;  ///< 总线舵机最大发送帧长度

/* Exported types ------------------------------------------------------------*/
HW_OPTIMIZE_O2_START

enum class ServoAngleRange : uint16_t {
  k180 = 180,  ///< 180 度舵机
  k270 = 270,  ///< 270 度舵机
};

struct BusServoInfo : public MemMgr {
  ServoAngleRange angle_range = ServoAngleRange::k180;  ///< 舵机角度范围
  uint16_t id = 0;                                      ///< 舵机 ID
  bool auto_enable = true;                              ///< 是否自动恢复力矩
  uint32_t offline_tick_thres = 200;                    ///< 掉线阈值，单位：ms
};

class BusServo : public comm::Receiver, public comm::Transmitter
{
 public:
  typedef BusServoInfo Info;

  enum class RxStatus : uint8_t {
    kWaitingHeader,  ///< 等待帧头
    kWaitingTail,    ///< 等待帧尾
  };

  enum class RxResult : uint8_t {
    kErrNoDataInput,    ///< 未开始接收数据，一直等不到帧头
    kHandlingWaitTail,  ///< 处理中-等待帧尾
    kErrTooLongData,    ///< 数据长度过长
    kErrWrongId,        ///< ID 错误
    kOk,                ///< 校验通过，解包成功
  };

  enum class SendMsgType : uint8_t {
    kGetAngle,  ///< 获取舵机角度
    kCtrl,      ///< 控制舵机角度
    kDisable,   ///< 释放扭力
    kEnable,    ///< 恢复扭力
  };

  BusServo() = default;
  explicit BusServo(Info info)
      : info_(info), oc_(info.offline_tick_thres) {}
  BusServo(const BusServo &) = default;
  BusServo &operator=(const BusServo &other) = default;
  BusServo(BusServo &&other) = default;
  BusServo &operator=(BusServo &&other) = default;

  virtual ~BusServo(void) = default;

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return 0u; }

  virtual const RxIds &rxIds(void) const override { return rx_ids_; }

  /**
   * @brief 解码输入数据并处理
   * @param len 输入数据的长度
   * @param data 指向输入数据的指针
   * @param rx_id 接收 ID
   * @return true 如果成功解码并处理了至少一个数据帧
   * @return false 如果输入数据为空或长度为零，或者解码失败
   */
  virtual bool decode(size_t len, const uint8_t *data, uint32_t rx_id) override;

  virtual bool isUpdate(void) const override { return is_updated_; }

  virtual void clearUpdateFlag(void) override { is_updated_ = false; }

  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  virtual uint32_t txId(void) const override { return 0u; }

  /**
   * @brief 编码数据并存储到输出缓冲区
   * @param len 输出参数，表示编码后的数据长度
   * @param data 输出参数，指向存储编码后数据的缓冲区
   * @return true 如果编码成功
   * @return false 如果数据长度不足
   */
  virtual bool encode(size_t &len, uint8_t *data) override;

  virtual void txSuccessCb(void) override { tx_success_cnt_++; }

  /* 功能函数 */

  /**
   * @brief       总线舵机初始化
   * @param        info: 电机可选配置参数
   * @retval       None
   * @note        使用默认构造函数后请务必调用此函数
   */
  void init(Info info)
  {
    info_ = info;
    oc_.set_offline_tick_thres(info.offline_tick_thres);

    status_ = Status();
    tx_data_ = TxData();
    rx_result_ = RxResult::kErrNoDataInput;
    rx_status_ = RxStatus::kWaitingHeader;
    rx_data_buffer_idx_ = 0;
    is_updated_ = false;
    update_cb_ = nullptr;
    encode_success_cnt_ = 0;
    encode_fail_cnt_ = 0;
    decode_success_cnt_ = 0;
    decode_fail_cnt_ = 0;
    tx_success_cnt_ = 0;
    memset(rx_data_buffer_, 0, sizeof(rx_data_buffer_));
    memset(tx_data_buffer_, 0, sizeof(tx_data_buffer_));
  }

  bool isOffline(void) { return oc_.isOffline(); }

  /* 反馈信息 */
  /**
   * @brief      获取舵机当前角度，单位：deg
   * @retval      舵机当前角度
   */
  float getAngleDeg(void) const { return status_.ang_fdb_deg; }

  /**
   * @brief      获取舵机当前角度，单位：rad
   * @retval      舵机当前角度
   */
  float getAngleRad(void) const { return Deg2Rad(getAngleDeg()); }

  /**
   * @brief      舵机是否使能
   * @retval      None
   * @note        None
   */
  bool isEnabled(void) const { return status_.enable; }

  /* 控制指令 */

  /**
   * @brief      舵机恢复力矩输出
   * @retval      None
   */
  void enable(void) { tx_data_.msg_type = SendMsgType::kEnable; }

  /**
   * @brief      舵机释放力矩，即不输出力矩
   * @retval      None
   */
  void disable(void) { tx_data_.msg_type = SendMsgType::kDisable; }

  /**
   * @brief      设置舵机期望角度
   * @param       ang_deg: 期望角度，单位：deg
   * @param       time: 运动时间，单位：ms
   * @retval      None
   * @note        角度范围为 [-90, 90] 或 [-135, 135]
   * @note        时间范围为 [0, 9999]，默认为 0，表示最快速度
   */
  void setAngleDeg(float ang_deg, uint16_t time = 0)
  {
    tx_data_.ang_ref = Bound(
        static_cast<int16_t>(ang_deg / static_cast<uint16_t>(info_.angle_range) * 2000) + 1500,
        500, 2500);
    tx_data_.move_time = Bound(time, 0, 9999);
    tx_data_.msg_type = SendMsgType::kCtrl;
  }

  /**
   * @brief      设置舵机期望角度
   * @param       ang_rad: 期望角度，单位：rad
   * @param       time: 运动时间，单位：ms
   * @retval      None
   * @note        角度范围为 [-pi/2, pi/2] 或 [-3pi/4, 3pi/4]
   * @note        时间范围为 [0, 9999]，默认为 0，表示最快速度
   */
  void setAngleRad(float ang_rad, uint16_t time = 0)
  {
    setAngleDeg(Rad2Deg(ang_rad), time);
  }

 protected:
  RxResult processByte(uint8_t byte);

  void resetDecodeProgress(bool keep_rx_buffer = false);

  bool decodeRxData(void);

  /**
   * @brief      字符串转整数
   * @param       *str: uint8_t 数组指针
   * @param       len: 数组长度
   * @retval      转换后的整数
   * @note        None
   */
  uint16_t str2uint(const uint8_t *str, size_t len);

  BusServoInfo info_ = BusServoInfo();                  ///< 舵机配置参数
  SendMsgType last_msg_type_ = SendMsgType::kGetAngle;  ///< 上次发送的指令类型

  /* 舵机状态 */
  struct Status {
    /* 反馈角度，中央位置为 1500，逆时针为正，范围 500 ~ 2500 */
    uint16_t ang_fdb = 0;
    float ang_fdb_rad = 0;  ///< 反馈角度，单位：rad
    float ang_fdb_deg = 0;  ///< 反馈角度，单位：deg

    /* 舵机是否输出力矩 */
    bool enable = false;
  } status_;

  /* 舵机控制指令 */
  struct TxData {
    /* 期望角度，中央位置为 1500，逆时针为正，范围 500 ~ 2500 */
    uint16_t ang_ref = 1500;

    /* 运动时间，单位：ms */
    uint16_t move_time = 0;

    /* 指令类型 */
    SendMsgType msg_type = SendMsgType::kGetAngle;
  } tx_data_;

  /* 接收相关 */
  RxIds rx_ids_ = {0};                       ///< 接收端 ID 列表
  OfflineChecker oc_ = OfflineChecker(200);  ///< 离线检测器

  /* 发送相关 */
  RxResult rx_result_ = RxResult::kErrNoDataInput;  ///< 接收结果
  RxStatus rx_status_ = RxStatus::kWaitingHeader;   ///< 接收状态
  uint8_t rx_data_buffer_idx_ = 0;                  ///< 接收数据索引
  bool is_updated_ = false;                         ///< 接收数据是否更新
  pUpdateCallback update_cb_ = nullptr;             ///< 更新回调函数

  /* debug 变量 */
  uint8_t rx_data_buffer_[kBusServoRxDataMaxLen] = {0};  ///< 接收数据缓存，debug 用
  uint8_t tx_data_buffer_[kBusServoTxDataMaxLen] = {0};  ///< 发给舵机的数据缓存，debug 用

  uint32_t encode_success_cnt_ = 0;  ///< 编码成功次数，debug 用
  uint32_t encode_fail_cnt_ = 0;     ///< 编码失败次数，debug 用
  uint32_t decode_success_cnt_ = 0;  ///< 解码成功次数，debug 用
  uint32_t decode_fail_cnt_ = 0;     ///< 解码失败次数，debug 用
  uint32_t tx_success_cnt_ = 0;      ///< 发送成功次数，debug 用

  static constexpr size_t kCtrlDataLen = 16u;  ///< 控制数据长度
  static constexpr size_t kCmdDataLen = 11u;   ///< 指令数据长度
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace servo
}  // namespace hello_world
#endif /* HW_COMPONENTS_DEVICES_SERVO_BUS_SERVO_HPP */
