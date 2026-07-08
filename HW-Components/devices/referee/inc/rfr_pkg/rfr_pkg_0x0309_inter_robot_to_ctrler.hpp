/**
 * @file      rfr_pkg_0x0309_inter_robot_to_ctrler.hpp
 * @author    Jinletian
 * @date      2024-12-28
 * @brief
 * @par last editor  Jinletian
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2025 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | Jinletian | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0309_INTER_ROBOT_TO_CTRLER_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0309_INTER_ROBOT_TO_CTRLER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"

namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct InterRobotToCtrlerData
 * @brief 机器人与自定义控制器交互数据
 */
struct __REFEREE_PACKED InterRobotToCtrlerData {
  uint8_t data[30];  ///< 自定义数据
};
static_assert(sizeof(InterRobotToCtrlerData) == 30, "InterRobotToCtrlerData size error");
/**
 * @class InterRobotToCtrlerDataPackage
 * @brief 机器人与自定义控制器交互数据包
 * @attention RMUL 暂不适用
 *
 * 机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据。
 *
 * 数据说明：
 *
 * - 命令码：0x0309
 * - 数据长度：30
 * - 发送频率：频率上限为 10Hz
 * - 发送方/接收方：己方机器人→对应操作手选手端连接的自定义控制器 
 * - 所属数据链路：图传链路
 */
class InterRobotToCtrlerDataPackage : public ProtocolTxPackage
{
 public:
  typedef InterRobotToCtrlerData Data;

  virtual CmdId getCmdId(void) const override { return 0x0309; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMinTxIntervalMs() const override
  {
    return FREQ2INTERVAL(10);
  }

  const Data &getData(void) const { return data_; }

  virtual bool encode(uint8_t *data) override
  {
    if (!isTxIntervalSatisfied()) {
      return false;
    }
    memcpy(data, &data_, sizeof(data_));
    last_encode_tick_ = getNowTickMs();
    return true;
  }

  void setUserData(const uint8_t *data, size_t length)
  {
    if (length > sizeof(data_.data)) {
      length = sizeof(data_.data);
    }
    memset(data_.data, 0, sizeof(data_.data));
    memcpy(data_.data, data, length);
  }

 protected:
  Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0309_INTER_ROBOT_TO_CTRLER_HPP_ */
