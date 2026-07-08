/**
 * @file      rfr_pkg_0x0301_inter_radar_detection.hpp
 * @author    C88-YQ (1409947012@qq.com)
 * @date      2025-04-05
 * @brief
 * @par last editor  C88-YQ (1409947012@qq.com)
 * @version   1.0.0
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All
 * Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2025-04-05 | C88-YQ | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_RADAR_DETECTION_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_RADAR_DETECTION_HPP_

#include "rfr_pkg_0x0301_inter_among_robots.hpp"
#include "rfr_custom_pkgs.hpp"

namespace hello_world
{

namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @struct InterRadarDetectionData
 * @brief 雷达检测数据
 */
struct __REFEREE_PACKED InterRadarDetectionData {
    uint16_t stage_remain_time;  ///< 当前阶段剩余时间，单位：秒

    /**
     * @brief 雷达检测标志位
     * - bit 0 1-表示雷达数据发出
     * - bit 1 1-表示雷达检测到红方英雄
     * - bit 2 1-表示雷达检测到红方工程
     * - bit 3 1-表示雷达检测到红方3号步兵
     * - bit 4 1-表示雷达检测到红方4号步兵
     * - bit 5 1-表示雷达检测到红方哨兵
     * - bit 6 1-表示雷达检测到蓝方英雄
     * - bit 7 1-表示雷达检测到蓝方工程
     * - bit 8 1-表示雷达检测到蓝方3号步兵
     * - bit 9 1-表示雷达检测到蓝方4号步兵
     * - bit 10 1-表示雷达检测到蓝方哨兵
     * - bit 11-15 保留位
     */
    uint16_t flag;

    /**
     * @brief 雷达标记进度
     * - 0 - 未标记； 1 - 易伤； 2 - 双倍易伤
     * - bit 0-1 雷达标记敌方英雄机器人进度
     * - bit 2-3 雷达标记敌方工程机器人进度
     * - bit 4-5 雷达标记敌方3号步兵机器人进度
     * - bit 6-7 雷达标记敌方4号步兵机器人进度
     * - bit 8-9 雷达标记敌方哨兵机器人进度
     * - bit 10-15 保留位
     */
    uint16_t mark_progress;

    int16_t red_hero_x;        ///< 红方英雄机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t red_hero_y;        ///< 红方英雄机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t red_engineer_x;    ///< 红方工程机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t red_engineer_y;    ///< 红方工程机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t red_standard_3_x;  ///< 红方 3 号步兵机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t red_standard_3_y;  ///< 红方 3 号步兵机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t red_standard_4_x;  ///< 红方 4 号步兵机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t red_standard_4_y;  ///< 红方 4 号步兵机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t red_sentry_x;      ///< 红方哨兵机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t red_sentry_y;      ///< 红方哨兵机器人位置 y 轴坐标，单位：/ 2000 m

    int16_t blue_hero_x;        ///< 蓝方英雄机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t blue_hero_y;        ///< 蓝方英雄机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t blue_engineer_x;    ///< 蓝方工程机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t blue_engineer_y;    ///< 蓝方工程机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t blue_standard_3_x;  ///< 蓝方 3 号步兵机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t blue_standard_3_y;  ///< 蓝方 3 号步兵机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t blue_standard_4_x;  ///< 蓝方 4 号步兵机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t blue_standard_4_y;  ///< 蓝方 4 号步兵机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t blue_sentry_x;      ///< 蓝方哨兵机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t blue_sentry_y;      ///< 蓝方哨兵机器人位置 y 轴坐标，单位：/ 2000 m
};

class InterRadarDetectionPackage : public CustomProtocolPackage
{
   public:
    typedef InterRadarDetectionData Data;

    virtual uint32_t getMaxRxIntervalMs() const override { return FREQ2INTERVAL(0); };

    virtual CmdId getInterCmdId(void) const override { return 0x0202; }

    virtual DataLength getInterDataLength(void) const override
    {
        return sizeof(Data);
    }

    virtual bool setSenderId(RfrId id) override
    {
        if (checkSenderId(id)) {
            sender_id_ = id;
            return true;
        }
        return false;
    }

    bool setReceiverId(RfrId id) override
    {
        if (checkReceiverId(id)) {
            receiver_id_ = id;
            return true;
        }
        return false;
    }

    Data &getData(void) { return data_; }

   protected:
    virtual bool checkSenderId(RfrId id) const
    {
        return ids::GetIdType(id) == ids::IdType::kRobot;
    }

    virtual bool checkReceiverId(RfrId id) const
    {
        return id == RfrId(ids::RobotId::kBlueRadar) ||
               id == RfrId(ids::RobotId::kRedRadar);
    }

    virtual void encodeInterData(uint8_t *data) override {}

    virtual bool decodeInterData(const uint8_t *data_ptr) override
    {
        memcpy(&data_, data_ptr, sizeof(Data));
        is_handled_ = false;
        return true;
    }
    Data data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_RADAR_DETECTION_HPP_ */
