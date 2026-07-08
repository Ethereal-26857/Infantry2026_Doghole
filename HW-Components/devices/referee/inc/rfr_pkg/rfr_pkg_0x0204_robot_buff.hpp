/**
 * @file      rfr_pkg_0x0204_robot_buff.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
 * @brief
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0204_ROBOT_BUFF_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0204_ROBOT_BUFF_HPP_

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
 * @struct RobotBuffData
 * @brief 机器人增益数据
 */
struct __REFEREE_PACKED RobotBuffData {
  uint8_t recovery_buff;  ///< 机器人回血增益，每秒恢复血量上限的百分比，值 10表示 10%
  uint8_t cooling_buff;   ///< 机器人枪口冷却倍率，值 5表示 5倍冷却
  uint8_t defence_buff;   ///< 机器人防御增益，百分比，值 50表示 50%防御增益
  /*/< 机器人负防御增益（百分比，值为 30 表示 -30%防御增益） */
  uint8_t vulnerability_buff;
  uint16_t attack_buff;  ///< 机器人攻击增益，百分比，值 50表示 50%攻击增益
  /*
   * 机器人剩余能量值反馈，以 16进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。
   * bit 0：在剩余能量≥50%时为 1，其余情况为 0
   * bit 1：在剩余能量≥30%时为 1，其余情况为 0
   * bit 2：在剩余能量≥15%时为 1，其余情况为 0
   * bit 3：在剩余能量≥5%时为 1，其余情况为 0
   * bit 4：在剩余能量≥1%时为 1，其余情况为 0
   */
  uint8_t remaining_energy;
};
static_assert(sizeof(RobotBuffData) == 7, "RobotBuffData size error");
// ! 数据结构体大小与汇总表中数据段长度不一致，等待实际检测

/** @class RobotBuffPackage
 * @brief 机器人增益数据包
 *
 * 数据说明：
 * - 命令码：0x0204
 * - 数据长度：6
 * - 发送频率：3Hz
 * - 发送方/接收方：服务器->对应机器人
 * - 所属数据链路：常规链路
 */
class RobotBuffPackage : public ProtocolRxPackage
{
 public:
  typedef RobotBuffData Data;

  virtual CmdId getCmdId(void) const override { return 0x0204; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(3);
  }

  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr) override
  {
    if (cmd_id == getCmdId()) {
      memcpy(&data_, data_ptr, sizeof(Data));
      last_decode_tick_ = getNowTickMs();
      is_handled_ = false;
      return true;
    }
    return false;
  }

  const Data &getData(void) const { return data_; }

  /**
   * @brief 机器人回血增益，每秒恢复血量上限的百分比，值 10表示 10%
   */
  uint8_t recovery_buff(void) const { return data_.recovery_buff; }
  /**
   * @brief 机器人枪口冷却倍率，值 5表示 5倍冷却
   */
  uint8_t cooling_buff(void) const { return data_.cooling_buff; }
  /**
   * @brief 机器人防御增益，百分比，值 50表示 50%防御增益
   */
  uint8_t defence_buff(void) const { return data_.defence_buff; }
  /**
   * @brief 机器人负防御增益（百分比，值为 30表示 -30%防御增益）
   */
  uint8_t vulnerability_buff(void) const { return data_.vulnerability_buff; }
  /**
   * @brief 机器人攻击增益，百分比，值 50表示 50%攻击增益
   */
  uint16_t attack_buff(void) const { return data_.attack_buff; }

  /*
   * 机器人剩余能量值反馈，以 16进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。
   * bit 0：在剩余能量≥50%时为 1，其余情况为 0
   * bit 1：在剩余能量≥30%时为 1，其余情况为 0
   * bit 2：在剩余能量≥15%时为 1，其余情况为 0
   * bit 3：在剩余能量≥5%时为 1，其余情况为 0
   * bit 4：在剩余能量≥1%时为 1，其余情况为 0
   */
  uint8_t remain_energy(void) const { return data_.remaining_energy; }
  /**
   * @brief 剩余能量是否大于等于 50%
   */
  bool remain_energy_over_50_percent(void) const { return data_.remaining_energy == 0x32; }
  /**
   * @brief 剩余能量是否大于等于 30%
   */
  bool remain_energy_over_30_percent(void) const
  {
    return data_.remaining_energy == 0x32 ||
           (data_.remaining_energy & 0x02);
  }
  /**
   * @brief 剩余能量是否大于等于 15%
   */
  bool remain_energy_over_15_percent(void) const
  {
    return data_.remaining_energy == 0x32 ||
           (data_.remaining_energy & 0x04);
  }
  /**
   * @brief 剩余能量是否大于等于 5%
   */
  bool remain_energy_over_5_percent(void) const
  {
    return data_.remaining_energy == 0x32 ||
           (data_.remaining_energy & 0x08);
  }
  /**
   * @brief 剩余能量是否大于等于 1%
   */
  bool remain_energy_over_1_percent(void) const
  {
    return data_.remaining_energy == 0x32 ||
           (data_.remaining_energy & 0x10);
  }

 private:
  Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0204_ROBOT_BUFF_HPP_ */
