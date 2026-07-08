/**
 *******************************************************************************
 * @file      : tx_mgr.cpp
 * @brief     : 通信发送管理器基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  不建议用户使用及继承该类，而是使用其派生类
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "tx_mgr.hpp"

#include <cstring>

#include "assert.hpp"
#include "stm32_hal.hpp"

namespace hello_world
{
namespace comm
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START
TxMgr &TxMgr::operator=(const TxMgr &other)
{
  if (this != &other) {
    transmitter_ptr_list_ = other.transmitter_ptr_list_;
  }

  return *this;
}

TxMgr::TxMgr(TxMgr &&other)
    : transmitter_ptr_list_(
          std::move(other.transmitter_ptr_list_))
{
}

TxMgr &TxMgr::operator=(TxMgr &&other)
{
  if (this != &other) {
    transmitter_ptr_list_ =
        std::move(other.transmitter_ptr_list_);
  }

  return *this;
}

void TxMgr::addTransmitter(Transmitter *new_transmitter_ptr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(new_transmitter_ptr != nullptr, "new_transmitter_ptr is nullptr");
#pragma endregion

  /* 查看发送端是否已存在，当不存在时再添加 */
  for (auto &transmitter_ptr : transmitter_ptr_list_) {
    if (transmitter_ptr == new_transmitter_ptr) {
      return;
    }
  }
  transmitter_ptr_list_.push_back(new_transmitter_ptr);
}

void TxMgr::setTransmitterNeedToTransmit(Transmitter *transmitter_ptr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(transmitter_ptr != nullptr, "transmitter_ptr is nullptr");
#pragma endregion

  transmitter_ptr->setNeedToTransmit();
}

bool TxMgr::deleteTransmitter(Transmitter *transmitter_ptr)
{
  for (auto it = transmitter_ptr_list_.begin();
       it != transmitter_ptr_list_.end(); ++it) {
    if (*it == transmitter_ptr) {
      transmitter_ptr_list_.erase(it);
      return true;
    }
  }

  return false;
}

void TxMgr::clearTransmitter(void)
{
  transmitter_ptr_list_.clear();
}

size_t TxMgr::getRemainMsgNum(void)
{
  size_t cnt = 0;
  for (auto &it : transmitter_ptr_list_) {
    if (it->need_to_transmit()) {
      cnt++;
    }
  }

  return cnt;
}

size_t TxMgr::encode(size_t &len, uint8_t *tx_buf, uint32_t &tx_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(tx_buf != nullptr, "data is nullptr");
  HW_ASSERT(len > 0, "len is less than 0");
#pragma endregion

  size_t cnt = 0;      // 编码成功的发送端数量
  size_t max_len = 0;  // 最大数据长度
  memset(tx_buf, 0, len);

  /* 尝试编码直至有报文编码完全通过时停止或遍历完所有报文 */
  for (auto &it : transmitter_ptr_list_) {
    if (it->need_to_transmit() == false) {
      continue;
    }
    if (cnt == 0 || (tx_id == it->txId() && it->allowSharedId())) {
      size_t tmp_buf_len = len;
      it->clearNeedToTransmit();  // 无论编码是否成功，都不再需要发送
      if (it->encode(tmp_buf_len, tx_buf)) {
        max_len = std::max(max_len, tmp_buf_len);

        // 对于不允许共享 ID 编码的设备，可以在 encode 函数中修改 tx_id
        // 否则必须在 encode 函数调用前修改 tx_id
        tx_id = it->txId();
        cnt++;
        if (!it->allowSharedId()) {
          break;  // 如果不允许共享 ID 编码，则直接跳出循环
        }
      }
    }
  }

  /* 完成编译，返回信息 */
  len = max_len;
  return cnt;
}

void TxMgr::setTransmitterFinished(uint32_t tx_id)
{
  for (auto &it : transmitter_ptr_list_) {
    if (it->txId() == tx_id) {
      /* 调用对应 ID 的发送端的发送完成回调 */
      it->txSuccessCb();
    }
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world