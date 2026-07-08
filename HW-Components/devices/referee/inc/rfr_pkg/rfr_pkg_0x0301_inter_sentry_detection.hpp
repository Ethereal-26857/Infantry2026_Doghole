/**
 * @file      rfr_pkg_0x0301_inter_sentry_detection.hpp
 * @author    C88-YQ (1409947012@qq.com)
 * @date      2025-03-28
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
 * | 1.0.0 | 2025-03-28 | C88-YQ | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_SENTRY_DETECTION_COMM_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_SENTRY_DETECTION_COMM_HPP_

#include "rfr_pkg_0x0301_inter_among_robots.hpp"

namespace hello_world
{

namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @struct InterRadarCmdData
 * @brief 哨兵与雷达交互数据，补盲雷达识别
 */
struct __REFEREE_PACKED InterSentryDetectionData {
    uint16_t stage_remain_time;  ///< 当前阶段剩余时间，单位：秒

    /**
     * @brief 哨兵雷达补盲标志位
     * - bit 0 1-表示哨兵全向感知数据发出
     * - bit 1 1-表示全向感知检测到红方英雄
     * - bit 2 1-表示全向感知检测到红方工程
     * - bit 3 1-表示全向感知检测到红方3号步兵
     * - bit 4 1-表示全向感知检测到红方4号步兵
     * - bit 5 1-表示全向感知检测到红方哨兵
     * - bit 6 1-表示全向感知检测到蓝方英雄
     * - bit 7 1-表示全向感知检测到蓝方工程
     * - bit 8 1-表示全向感知检测到蓝方3号步兵
     * - bit 9 1-表示全向感知检测到蓝方4号步兵
     * - bit 10 1-表示全向感知检测到蓝方哨兵
     * - bit 11 1-表示全向感知检测到其他机器人1
     * - bit 12 1-表示全向感知检测到其他机器人2
     * - bit 13 1-表示全向感知检测到其他机器人3
     * - bit 14 1-表示全向感知检测到其他机器人4
     * - bit 15 1-表示全向感知检测到其他机器人5
     */
    uint16_t flag;  ///< 哨兵雷达补盲标志位

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

    /**
     * @brief 其他机器人主要指的是当前已死亡的机器人
     */
    int16_t other_robot_1_x;  ///< 其他机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t other_robot_1_y;  ///< 其他机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t other_robot_2_x;  ///< 其他机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t other_robot_2_y;  ///< 其他机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t other_robot_3_x;  ///< 其他机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t other_robot_3_y;  ///< 其他机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t other_robot_4_x;  ///< 其他机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t other_robot_4_y;  ///< 其他机器人位置 y 轴坐标，单位：/ 2000 m
    int16_t other_robot_5_x;  ///< 其他机器人位置 x 轴坐标，单位：/ 2000 m
    int16_t other_robot_5_y;  ///< 其他机器人位置 y 轴坐标，单位：/ 2000 m
};

class InterSentryDetectionPackage : public InterAmongRobotsPackage
{
   public:
    typedef InterSentryDetectionData Data;

    virtual CmdId getInterCmdId(void) const override { return 0x0201; }
    virtual DataLength getInterDataLength(void) const override
    {
        return sizeof(Data);
    }

    virtual bool setSenderId(RfrId id) override
    {
        if (checkSenderId(id)) {
            sender_id_ = id;
            receiver_id_ = id + 2;  // 雷达 ID 比哨兵 ID 大 2
            return true;
        }
        return false;
    }

    void setData(const Data &data) { memcpy(&data_, &data, sizeof(Data)); }

   protected:
    virtual bool checkSenderId(RfrId id) const
    {
        return id == RfrId(ids::RobotId::kBlueSentry) ||
               id == RfrId(ids::RobotId::kRedSentry);
    }

    virtual bool checkReceiverId(RfrId id) const
    {
        return id == RfrId(ids::RobotId::kBlueRadar) ||
               id == RfrId(ids::RobotId::kRedRadar);
    }
    virtual void encodeInterData(uint8_t *data) override
    {
        memcpy(data, &data_, sizeof(Data));
    }

    Data data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_RFR_PKG_RFR_PKG_0X0301_INTER_SENTRY_DETECTION_COMM_HPP_ */
