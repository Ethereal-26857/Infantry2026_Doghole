#ifndef INSTANCE_INS_COMM_HPP_
#define INSTANCE_INS_COMM_HPP_

#include "fdcan_rx_mgr.hpp"
#include "fdcan_tx_mgr.hpp"
#include "uart_rx_mgr.hpp"
#include "uart_tx_mgr.hpp"
#include "fdcan.h"
#include "usart.h"

/* fdcan */
FDCAN_HandleTypeDef* const chassis_front_fdcan = &hfdcan1;      ///< 底盘前部分舵电机+轮电机+超电
FDCAN_HandleTypeDef* const chassis_rear_fdcan = &hfdcan2;       ///< 底盘后部分舵电机+轮电机+大YAW电机
FDCAN_HandleTypeDef* const gimbal_shooter_fdcan = &hfdcan3;     ///< 小云台电机+拨弹电机+摩擦轮电机

/* uart */
UART_HandleTypeDef* const rfr_uart = &huart1;       ///< 裁判系统
UART_HandleTypeDef* const rc_uart = &huart5;            ///< 遥控器

hello_world::comm::FdCanRxMgr* GetChassisFrontFdCanRxMgrIns(void);
hello_world::comm::FdCanTxMgr* GetChassisFrontFdCanTxMgrIns(void);

hello_world::comm::FdCanRxMgr* GetChassisRearFdCanRxMgrIns(void);
hello_world::comm::FdCanTxMgr* GetChassisRearFdCanTxMgrIns(void);

hello_world::comm::FdCanRxMgr* GetGimbalShooterFdCanRxMgrIns(void);
hello_world::comm::FdCanTxMgr* GetGimbalShooterFdCanTxMgrIns(void); 

hello_world::comm::UartRxMgr* GetRfrUartRxMgrIns(void);
hello_world::comm::UartTxMgr* GetRfrUartTxMgrIns(void);

hello_world::comm::UartRxMgr* GetRcUartRxMgrIns(void);

#endif /* INSTANCE_INS_COMM_HPP_ */