/**
 *******************************************************************************
 * @file      : motor.hpp
 * @brief     : 电机类汇总
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-14      Caikunzhen      1. 添加电机简单工厂函数
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_A1.hpp"
#include "motor_DaMiao.hpp"
#include "motor_DrEmpower.hpp"
#include "motor_JC2804.hpp"
#include "motor_GM6020.hpp"
#include "motor_GO_M8010.hpp"
#include "motor_M2006.hpp"
#include "motor_M3508.hpp"
#include "motor_M8910.hpp"
#include "motor_LingKong.hpp"
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world {
namespace motor {
/* Exported macro ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum class MotorType {
  kGM6020,
  kM2006,
  kM3508,
  kM8910,
  kA1,
  kDM_J4310,
  kDM_J6006,
  kDM_J8006,
  kDM_J8009,
  KDrEmpower,
  kMF9025v2,
  kMG6012Ei36,
  kJC2804,
  kMG8016E,
  kMG5010Ei36,
  kGO_M8010,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       根据所需的电机类型创建电机并初始化
<<<<<<< HEAD
 * @param        id: 电机 ID，取值范围由电机类型决定
 * @param        opt: 电机可选配置参数
 * @retval       创建的电机类指针
 * @note        返回的电机类是经过内存申请得到的，不使用时需要自行释放内存
 */
Motor *CreateMotor(MotorType motor_type, uint8_t id,
                   const OptionalParams &optinal_params = OptionalParams());

HW_OPTIMIZE_O2_END
} // namespace motor
} // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_ */
