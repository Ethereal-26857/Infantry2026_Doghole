#ifndef SENTRY_COMPONENTS_MODULE_STATE_HPP_
#define SENTRY_COMPONENTS_MODULE_STATE_HPP_

#include <cstdint>
#include <string>

// NOTE: 需要重新考虑各种模式

namespace robot
{
// 通用状态枚举

/**
 * @brief 电源状态
 *
 * 定义了与机器人模块相关的电源状态，包括死亡状态（断电）、复活状态（上电后的初始状态）和工作状态。
 */
enum class PwrState : uint8_t {
    kDead = 0u,     ///< 死亡状态
    kResurrection,  ///< 复活状态
    kWorking,       ///< 工作状态
};

/** 底盘工作模式 */
enum class ChassisWorkingMode : uint8_t {
    kDepart,       ///< 分离模式
    kFollow,       ///< 随动模式
    kFollowSpeed,  ///< 速度跟随模式
    kFastGyro,     ///< 快速小陀螺模式
    kSlowGyro,     ///< 慢速小陀螺模式
    kInTiltHole,   ///< 斜狗洞模式
    kInSgtHole,    ///< 直狗洞模式
};

/** 云台关节工作模式 */
enum class GimbalJointWorkingMode : uint8_t {
    kManual,  ///< 普通模式，由遥控器控制
    kAutoAim,  ///< 自瞄模式
    kSearch,   ///< 巡逻模式
    kOutpost,  ///< 前哨站模式
    kInhole,   ///< 狗洞模式
    kBuff,  ///<打符模式
};

/** 发射机构工作模式 */
enum class ShooterWorkingMode : uint8_t {
    kNormal,        ///< 正常模式
    kFricBackward,  ///< 摩擦轮倒转模式
};

enum class FeedWorkingMode : uint8_t {
    kSingle,      ///< 单发模式
    kContinuous,  ///< 连发模式
    kOff,         ///< 无效模式
};

enum class FricWorkingMode : uint8_t {
    kForward,   ///< 正常模式
    kBackward,  ///< 反转模式
};

inline std::string PwrStateToStr(PwrState state)
{
    if (state == PwrState::kDead)
        return "Dead";
    if (state == PwrState::kResurrection)
        return "Resurrection";
    if (state == PwrState::kWorking)
        return "Working";
    return "ErrWS";
};

inline std::string ChassisWorkingModeToStr(ChassisWorkingMode mode)
{
    if (mode == ChassisWorkingMode::kDepart)
        return "Depart";
    if (mode == ChassisWorkingMode::kFollow)
        return "Follow";
    if (mode == ChassisWorkingMode::kFollowSpeed)
        return "FollowSpeed";
    if (mode == ChassisWorkingMode::kFastGyro)
        return "FastGyro";
    if (mode == ChassisWorkingMode::kSlowGyro)
        return "SlowGyro";
    return "ErrCWM";
};

// inline std::string MainYawWorkingModeToStr(MainYawWorkingMode mode)
// {
//   if (mode == MainYawWorkingMode::kNormal)
//     return "Normal";
//   return "ErrMYWM";
// };

// inline std::string SmallGimbalWorkingModeToStr(SmallGimbalWorkingMode mode)
// {
//   if (mode == SmallGimbalWorkingMode::kNormal)
//     return "Normal";
//   if (mode == SmallGimbalWorkingMode::kAutoAim)
//     return "AutoAim";
//   if (mode == SmallGimbalWorkingMode::kSecAutoAim)
//     return "SecAutoAim";
//   if (mode == SmallGimbalWorkingMode::kSearch)
//     return "Search";
//   if (mode == SmallGimbalWorkingMode::kOutpostYaw)
//     return "OutpostYaw";
//   if (mode == SmallGimbalWorkingMode::kOutpostPitch)
//     return "OutpostPitch";
//   if (mode == SmallGimbalWorkingMode::kBuff)
//     return "Buff";
//   return "ErrSGWM";
// };

inline std::string GimbalJointWorkingModeToStr(GimbalJointWorkingMode mode)
{
    if (mode == GimbalJointWorkingMode::kManual)
        return "Maunal";
    if (mode == GimbalJointWorkingMode::kAutoAim)
        return "AutoAim";
    if (mode == GimbalJointWorkingMode::kSearch)
        return "Search";
    if (mode == GimbalJointWorkingMode::kOutpost)
        return "OutpostYaw";
    if (mode == GimbalJointWorkingMode::kInhole)
        return "Inhole";
    if (mode == GimbalJointWorkingMode::kBuff)
        return "Buff";
    return "ErrSGWM";
};

inline std::string ShooterWorkingModeToStr(ShooterWorkingMode mode)
{
    if (mode == ShooterWorkingMode::kNormal)
        return "Normal";
    if (mode == ShooterWorkingMode::kFricBackward)
        return "FricBackward";
    return "ErrSWM";
};

inline std::string FeedWorkingModeToStr(FeedWorkingMode mode)
{
    if (mode == FeedWorkingMode::kSingle)
        return "Single";
    if (mode == FeedWorkingMode::kContinuous)
        return "Continuous";
    return "ErrFWM";
};

inline std::string FricWorkingModeToStr(FricWorkingMode mode)
{
    if (mode == FricWorkingMode::kForward)
        return "Forward";
    if (mode == FricWorkingMode::kBackward)
        return "Backward";
    return "ErrFWM";
};

}  // namespace robot

#endif /* SENTRY_COMPONENTS_MODULE_STATE_HPP_ */