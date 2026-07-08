/**
 *******************************************************************************
 * @file      : crc.hpp
 * @brief     : crc校验
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-05-12      Jinletian        1. 移植自视觉组
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENTRY_H7_SENTRY_COMPONENTS_MODULES_CRC_CRC_HPP_
#define SENTRY_H7_SENTRY_COMPONENTS_MODULES_CRC_CRC_HPP_
/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace robot
{
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief CRC8 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */

uint32_t Verify_CRC8_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief Append CRC8 value to the end of the buffer
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return none
 */

void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint8_t Get_CRC8_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);

}  // namespace sentry
#endif  // SENTRY_H7_SENTRY_COMPONENTS_MODULES_CRC_CRC_HPP_
