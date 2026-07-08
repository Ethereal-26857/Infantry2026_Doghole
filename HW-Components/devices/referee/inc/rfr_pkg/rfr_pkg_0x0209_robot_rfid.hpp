/**
 * @file      rfr_pkg_0x0209_robot_rfid.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0209_ROBOT_RFID_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0209_ROBOT_RFID_HPP_

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
 * @struct __REFEREE_PACKED RegionsWithRFID
 * @brief 机器人 RFID 模块状态数据
 *
 * 这个结构体通过位字段存储了RFID探测到的每一个增益点的信息。
 * bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
 * @note 只有在比赛内探测到的增益点才会生效，否则，即使检测到对应的RFID卡，对应值也为0.
 */
struct __REFEREE_PACKED RobotRfidData {
  uint32_t our_base : 1;                 ///< bit 0: 己方基地增益点
  uint32_t our_central_highland : 1;     ///< bit 1: 己方中央高地增益点
  uint32_t opp_central_highland : 1;     ///< bit 2: 对方中央高地增益点
  uint32_t our_trapezoid_highland : 1;   ///< bit 3: 己方梯形高地增益点
  uint32_t opp_trapezoid_highland : 1;   ///< bit 4: 对方梯形高地增益点
  uint32_t our_launch_front : 1;         ///< bit 5: 己方飞坡增益点（靠近己方一侧飞坡前）
  uint32_t our_launch_back : 1;          ///< bit 6: 己方飞坡增益点（靠近己方一侧飞坡后）
  uint32_t opp_launch_front : 1;         ///< bit 7: 对方飞坡增益点（靠近对方一侧飞坡前）
  uint32_t opp_launch_back : 1;          ///< bit 8: 对方飞坡增益点（靠近对方一侧飞坡后）
  uint32_t our_highland_bottom : 1;      ///< bit 9: 己方地形跨越增益点（中央高地下方）
  uint32_t our_highland_top : 1;         ///< bit 10: 己方地形跨越增益点（中央高地上方）
  uint32_t opp_highland_bottom : 1;      ///< bit 11: 对方地形跨越增益点（中央高地下方）
  uint32_t opp_highland_top : 1;         ///< bit 12: 对方地形跨越增益点（中央高地上方）
  uint32_t our_highway_bottom : 1;       ///< bit 13: 己方地形跨越增益点（公路下方）
  uint32_t our_highway_top : 1;          ///< bit 14: 己方地形跨越增益点（公路上方）
  uint32_t opp_highway_bottom : 1;       ///< bit 15: 对方地形跨越增益点（公路下方）
  uint32_t opp_highway_top : 1;          ///< bit 16: 对方地形跨越增益点（公路上方）
  uint32_t our_fort : 1;                 ///< bit 17: 己方堡垒增益点
  uint32_t our_outpost : 1;              ///< bit 18: 己方前哨站增益点
  uint32_t our_restoration_1 : 1;        ///< bit 19: 己方与兑换区不重叠的补给区/RMUL 补给区 
  uint32_t our_restoration_2 : 1;        ///< bit 20: 己方与兑换区重叠的补给区
  uint32_t our_big_resource_island : 1;  ///< bit 21: 己方大资源岛增益点
  uint32_t opp_big_resource_island : 1;  ///< bit 22: 对方大资源岛增益点
  uint32_t central_boost : 1;            ///< bit 23: 中心增益点 @attention 仅RMUL适用
  uint32_t reserved : 8;                ///< 保留区域
};
static_assert(sizeof(RobotRfidData) == 4, "RobotRfidData size error");
/** @class RobotRfidPackage
 * @brief 机器人 RFID 模块状态数据包
 *
 * 数据说明：
 * - 命令码：0x0209
 * - 数据长度：4
 * - 发送频率：3Hz
 * - 发送方/接收方：服务器->己方装有RFID模块的机器人
 * - 所属数据链路：常规链路
 */
class RobotRfidPackage : public ProtocolRxPackage
{
 public:
  typedef RobotRfidData Data;

  virtual CmdId getCmdId(void) const override { return 0x0209; }
  virtual DataLength getDataLength(void) const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs(void) const override
  {
    return FREQ2INTERVAL(3);
  }

  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr) override
  {
    if (cmd_id == getCmdId()) {
      memcpy(&data_, data_ptr, sizeof(Data));
      memcpy(&raw_data_, data_ptr, sizeof(uint32_t));
      last_decode_tick_ = getNowTickMs();
      is_handled_ = false;
      return true;
    }
    return false;
  }

  const Data &getData(void) const { return data_; }

  uint32_t getRawData(void) const { return raw_data_; }

  /**
   * @brief 己方基地增益点
   */
  bool our_base(void) const { return data_.our_base; }
  /**
   * @brief 己方中央高地增益点
   */
  bool our_central_highland(void) const { return data_.our_central_highland; }
  /**
   * @brief 对方中央高地增益点
   */
  bool opp_central_highland(void) const { return data_.opp_central_highland; }
  /**
   * @brief 己方梯形高地增益点
   */
  bool our_trapezoid_highland(void) const { return data_.our_trapezoid_highland; }
  /**
   * @brief 对方梯形高地增益点
   */
  bool opp_trapezoid_highland(void) const { return data_.opp_trapezoid_highland; }
  /**
   * @brief 己方飞坡增益点（靠近己方一侧飞坡前）
   */
  bool our_launch_front(void) const { return data_.our_launch_front; }
  /**
   * @brief 己方飞坡增益点（靠近己方一侧飞坡后）
   */
  bool our_launch_back(void) const { return data_.our_launch_back; }
  /**
   * @brief 对方飞坡增益点（靠近对方一侧飞坡前）
   */
  bool opp_launch_front(void) const { return data_.opp_launch_front; }
  /**
   * @brief 对方飞坡增益点（靠近对方一侧飞坡后）
   */
  bool opp_launch_back(void) const { return data_.opp_launch_back; }
  /**
   * @brief 己方地形跨越增益点（中央高地下方）
   */
  bool our_highland_bottom(void) const { return data_.our_highland_bottom; }
  /**
   * @brief 己方地形跨越增益点（中央高地上方）
   */
  bool our_highland_top(void) const { return data_.our_highland_top; }
  /**
   * @brief 对方地形跨越增益点（中央高地下方）
   */
  bool opp_highland_bottom(void) const { return data_.opp_highland_bottom; }
  /**
   * @brief 对方地形跨越增益点（中央高地上方）
   */
  bool opp_highland_top(void) const { return data_.opp_highland_top; }
  /**
   * @brief 己方地形跨越增益点（公路下方）
   */
  bool our_highway_bottom(void) const { return data_.our_highway_bottom; }
  /**
   * @brief 己方地形跨越增益点（公路上方）
   */
  bool our_highway_top(void) const { return data_.our_highway_top; }
  /**
   * @brief 对方地形跨越增益点（公路下方）
   */
  bool opp_highway_bottom(void) const { return data_.opp_highway_bottom; }
  /**
   * @brief 对方地形跨越增益点（公路上方）
   */
  bool opp_highway_top(void) const { return data_.opp_highway_top; }
  /**
   * @brief 己方堡垒增益点
   */
  bool our_fort(void) const { return data_.our_fort; }
  /**
   * @brief 己方前哨站增益点
   */
  bool our_outpost(void) const { return data_.our_outpost; }
  /**
   * @brief 己方与兑换区不重叠的补给区/RMUL 补给区
   */
  bool our_restoration_1(void) const { return data_.our_restoration_1; }
  /**
   * @brief 己方与兑换区重叠的补给区
   */
  bool our_restoration_2(void) const { return data_.our_restoration_2; }
  /**
   * @brief 己方大资源岛增益点
   */
  bool our_big_resource_island(void) const { return data_.our_big_resource_island; }
  /**
   * @brief 对方大资源岛增益点
   */
  bool opp_big_resource_island(void) const { return data_.opp_big_resource_island; }
  /**
   * @brief 中心增益点 @attention 仅RMUL适用
   */
  bool central_boost(void) const { return data_.central_boost; }

 private:
  Data data_ = {0};
  uint32_t raw_data_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0209_ROBOT_RFID_HPP_ */
