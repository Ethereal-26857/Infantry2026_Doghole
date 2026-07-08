/**
 *******************************************************************************
 * @file      : STP23.hpp
 * @brief     : 单点激光测距模块 STP23
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2025-02-12      Jinletian       1.Create this file
 *******************************************************************************
 * @attention :
 * 1. STP23 单点激光测距模块依赖 uart 口 5V 供电，使用时请确保供电正常，例如：达妙 h7
 *    控制板需要配置 PC15 口推挽输出模式，并置为高电平。
 * 2. 该类依赖串口接收管理器 UartRxMgr，使用前请确保 UartRxMgr 按要求配置于初始化，其中
 *    串口波特率设置为 921600 bps，字长 8 Bits(include Parity)，无校验，停止位 1，只接收。
 *    串口接收管理器中 buf_len 设置为 STP::kRxDataLen + 1，max_process_data_len 设置为
 *    STP::kRxDataLen，eof_type 设置为 EofType::kIdle
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_DIST_MEASURE_STP23_HPP_
#define HW_COMPONENTS_DEVICES_DIST_MEASURE_STP23_HPP_

/* Includes ------------------------------------------------------------------*/
#include "offline_checker.hpp"
#include "receiver.hpp"

namespace hello_world
{
namespace dist_measure
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

struct __PACKED PointData {
  uint16_t distance;  ///< 距离
  uint8_t intensity;  ///< 置信度
};

struct __PACKED STP23Pkg {
  uint8_t header;        ///< 帧头 0x54
  uint8_t ver_len;       ///< 低五位是一帧数据接收到的点数，目前固定是12，高三位固定为1
  uint16_t temperature;  ///< 温度低8位，一共16位ADC，0--4096，无量纲
  uint16_t start_angle;  ///< 起始角度
  PointData point[12];   ///< 测量点数据
  uint16_t end_angle;    ///< 结束角度
  uint16_t timestamp;    ///< 时间戳
  uint8_t crc8;          ///< CRC 校验和
};

class STP23 : public comm::Receiver
{
 public:
  typedef STP23Pkg Data;

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  STP23(void) = default;

  /**
   * @brief       构造函数
   * @param        weight: 低通滤波权重，范围 [0, 1]，值越大表示越信任当前值
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  explicit STP23(float weight, uint32_t offline_tick_thres = 100)
      : Receiver(), weight_(weight), oc_(offline_tick_thres) 
  {
    HW_ASSERT(0 <= weight && weight <= 1, "Error weight: %f", weight);
  }
  STP23(const STP23 &) = default;
  STP23 &operator=(const STP23 &other) = default;
  STP23(STP23 &&other) = default;
  STP23 &operator=(STP23 &&other) = default;

  virtual ~STP23(void) {}

  /* 配置方法 */

  void init(float weight = 0.1, uint32_t offline_tick_thres = 100)
  {
    weight_ = weight;
    oc_.init(offline_tick_thres);
  }

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return 0u; }

  virtual const RxIds &rxIds(void) const override { return rx_ids_; }

  /**
   * @brief 解码输入数据并处理
   *
   * 该函数解码输入数据，并调用处理函数处理每个字节。如果解码成功，则更新状态并调用回调
   * 函数。
   *
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

  /* 数据获取 */

  /**
   * @brief      获取滤波后距离
   * @retval      distance 测量距离, 单位 mm
   * @note        None
   */
  float distance(void) const { return distance_; }

  /**
   * @brief      获取原始数据
   * @retval      Data 原始数据
   * @note        None
   */
  const Data &getData() const { return data_; }

  /* 接收数据长度 */
  static const uint8_t kRxDataLen = 47;

 private:
  void calcDistance(void);

  /* 接收数据 */
  Data data_;           ///< 原始数据
  float distance_ = 0;  ///< 测量距离，单位 mm
  uint8_t crc_ = 0;     ///< CRC 校验和
  float weight_ = 0.1;  ///< 低通滤波权重

  /* 更新相关 */
  bool is_updated_ = false;                  ///< 是否更新标志
  pUpdateCallback update_cb_ = nullptr;      ///< 更新回调函数
  OfflineChecker oc_ = OfflineChecker(100);  ///< 掉线检查

  /* 收发状态统计 */
  RxIds rx_ids_ = {0};               ///< 接收端 ID 列表
  uint32_t decode_success_cnt_ = 0;  ///< 解码成功次数
  uint32_t decode_fail_cnt_ = 0;     ///< 解码失败次数
  uint32_t crc_fail_cnt_ = 0;        ///< CRC 校验失败次数
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dist_measure
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_DIST_MEASURE_STP23_HPP_ */
