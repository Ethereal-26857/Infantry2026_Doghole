#include "comm_task.hpp"

// HAL库头文件
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "tick.hpp"

#include "ins_all.hpp"

using hello_world::comm::FdCanRxMgr;
using hello_world::comm::FdCanTxMgr;
using hello_world::comm::UartRxMgr;
using hello_world::comm::UartTxMgr;
static FdCanRxMgr* chassis_front_fdcan_rx_mgr_ptr = nullptr;
static FdCanTxMgr* chassis_front_fdcan_tx_mgr_ptr = nullptr;

static FdCanRxMgr* chassis_rear_fdcan_rx_mgr_ptr = nullptr;
static FdCanTxMgr* chassis_rear_fdcan_tx_mgr_ptr = nullptr;

static FdCanRxMgr* gimbal_shooter_fdcan_rx_mgr_ptr = nullptr;
static FdCanTxMgr* gimbal_shooter_fdcan_tx_mgr_ptr = nullptr;

static UartRxMgr* rfr_uart_rx_mgr_ptr = nullptr;
static UartTxMgr* rfr_uart_tx_mgr_ptr = nullptr;

static UartRxMgr* rc_uart_rx_mgr_ptr = nullptr;

static void PrivatePointerInit(void);
static void CommAddReceiver(void);
static void CommAddTransmitter(void);
static void CommHardwareInit(void);

void CommTaskInit(void)
{
    PrivatePointerInit();

    CommAddReceiver();
    CommAddTransmitter();

    CommHardwareInit();
}

void CommTask(void)
{
    HW_ASSERT(chassis_front_fdcan_tx_mgr_ptr != nullptr, "chassis_front_fdcan_tx_mgr_ptr is nullptr", chassis_front_fdcan_tx_mgr_ptr);
    HW_ASSERT(chassis_rear_fdcan_tx_mgr_ptr != nullptr, "chassis_rear_fdcan_tx_mgr_ptr is nullptr", chassis_rear_fdcan_tx_mgr_ptr);
    HW_ASSERT(gimbal_shooter_fdcan_tx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_tx_mgr_ptr is nullptr", gimbal_shooter_fdcan_tx_mgr_ptr);
    HW_ASSERT(rfr_uart_tx_mgr_ptr != nullptr, "rfr_uart_tx_mgr_ptr is nullptr", rfr_uart_tx_mgr_ptr);

    chassis_front_fdcan_tx_mgr_ptr->startTransmit();
    chassis_rear_fdcan_tx_mgr_ptr->startTransmit();
    gimbal_shooter_fdcan_tx_mgr_ptr->startTransmit();
    rfr_uart_tx_mgr_ptr->startTransmit();

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITS)
{
    HW_ASSERT(chassis_front_fdcan_rx_mgr_ptr != nullptr, "chassis_front_fdcan_rx_mgr_ptr is nullptr", chassis_front_fdcan_rx_mgr_ptr);
    HW_ASSERT(chassis_rear_fdcan_rx_mgr_ptr != nullptr, "chassis_rear_fdcan_rx_mgr_ptr is nullptr", chassis_rear_fdcan_rx_mgr_ptr);
    HW_ASSERT(gimbal_shooter_fdcan_rx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_rx_mgr_ptr is nullptr", gimbal_shooter_fdcan_rx_mgr_ptr);

    chassis_front_fdcan_rx_mgr_ptr->rxFifoCallback(hfdcan, RxFifo0ITS);
    chassis_rear_fdcan_rx_mgr_ptr->rxFifoCallback(hfdcan, RxFifo0ITS);
    gimbal_shooter_fdcan_rx_mgr_ptr->rxFifoCallback(hfdcan, RxFifo0ITS);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITS)
{
    HW_ASSERT(chassis_front_fdcan_rx_mgr_ptr != nullptr, "chassis_front_fdcan_rx_mgr_ptr is nullptr", chassis_front_fdcan_rx_mgr_ptr);
    HW_ASSERT(chassis_rear_fdcan_rx_mgr_ptr != nullptr, "chassis_rear_fdcan_rx_mgr_ptr is nullptr", chassis_rear_fdcan_rx_mgr_ptr);
    HW_ASSERT(gimbal_shooter_fdcan_rx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_rx_mgr_ptr is nullptr", gimbal_shooter_fdcan_rx_mgr_ptr);

    chassis_front_fdcan_rx_mgr_ptr->rxFifoCallback(hfdcan, RxFifo1ITS);
    chassis_rear_fdcan_rx_mgr_ptr->rxFifoCallback(hfdcan, RxFifo1ITS);
    gimbal_shooter_fdcan_rx_mgr_ptr->rxFifoCallback(hfdcan, RxFifo1ITS);
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan)
{
    HW_ASSERT(chassis_front_fdcan_tx_mgr_ptr != nullptr, "chassis_front_fdcan_tx_mgr_ptr is nullptr", chassis_front_fdcan_tx_mgr_ptr);
    HW_ASSERT(chassis_rear_fdcan_tx_mgr_ptr != nullptr, "chassis_rear_fdcan_tx_mgr_ptr is nullptr", chassis_rear_fdcan_tx_mgr_ptr);
    HW_ASSERT(gimbal_shooter_fdcan_tx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_tx_mgr_ptr is nullptr", gimbal_shooter_fdcan_tx_mgr_ptr);

    chassis_front_fdcan_tx_mgr_ptr->txFifoEmptyCallback(hfdcan);
    chassis_rear_fdcan_tx_mgr_ptr->txFifoEmptyCallback(hfdcan);
    gimbal_shooter_fdcan_tx_mgr_ptr->txFifoEmptyCallback(hfdcan);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatus)
{
    HW_ASSERT(chassis_front_fdcan_rx_mgr_ptr != nullptr, "chassis_front_fdcan_rx_mgr_ptr is nullptr", chassis_front_fdcan_rx_mgr_ptr);
    HW_ASSERT(chassis_rear_fdcan_rx_mgr_ptr != nullptr, "chassis_rear_fdcan_rx_mgr_ptr is nullptr", chassis_rear_fdcan_rx_mgr_ptr);
    HW_ASSERT(gimbal_shooter_fdcan_rx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_rx_mgr_ptr is nullptr", gimbal_shooter_fdcan_rx_mgr_ptr);

    HW_ASSERT(chassis_front_fdcan_tx_mgr_ptr != nullptr, "chassis_front_fdcan_tx_mgr_ptr is nullptr", chassis_front_fdcan_tx_mgr_ptr);
    HW_ASSERT(chassis_rear_fdcan_tx_mgr_ptr != nullptr, "chassis_rear_fdcan_tx_mgr_ptr is nullptr", chassis_rear_fdcan_tx_mgr_ptr);
    HW_ASSERT(gimbal_shooter_fdcan_tx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_tx_mgr_ptr is nullptr", gimbal_shooter_fdcan_tx_mgr_ptr);

    chassis_front_fdcan_rx_mgr_ptr->errorStatusCallback(hfdcan, ErrorStatus);
    chassis_rear_fdcan_rx_mgr_ptr->errorStatusCallback(hfdcan, ErrorStatus);
    gimbal_shooter_fdcan_rx_mgr_ptr->errorStatusCallback(hfdcan, ErrorStatus);

    chassis_front_fdcan_tx_mgr_ptr->errorStatusCallback(hfdcan, ErrorStatus);
    chassis_rear_fdcan_tx_mgr_ptr->errorStatusCallback(hfdcan, ErrorStatus);
    gimbal_shooter_fdcan_tx_mgr_ptr->errorStatusCallback(hfdcan, ErrorStatus);
}

uint32_t uart_rx_tick = 0;
uint32_t uart_tx_cb_in = 0;
uint32_t rc_uart_rx_cnt = 0;
uint32_t rfr_uart_rx_cnt = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    uart_rx_tick = hello_world::tick::GetTickMs();
    uart_tx_cb_in++;

    if(huart == rc_uart)
    {
        rc_uart_rx_cnt++;
        rc_uart_rx_mgr_ptr->rxEventCallback(huart, Size);
    }
    else if(huart == rfr_uart)
    {
        rfr_uart_rx_cnt++;
        rfr_uart_rx_mgr_ptr->rxEventCallback(huart, Size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    rfr_uart_tx_mgr_ptr->txCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if(huart == rc_uart)
    {
        rc_uart_rx_mgr_ptr->startReceive();
    }
    else if(huart == rfr_uart)
    {
        rfr_uart_rx_mgr_ptr->startReceive();
    }
}

static void PrivatePointerInit(void)
{
    chassis_front_fdcan_rx_mgr_ptr = GetChassisFrontFdCanRxMgrIns();
    chassis_front_fdcan_tx_mgr_ptr = GetChassisFrontFdCanTxMgrIns();

    chassis_rear_fdcan_rx_mgr_ptr = GetChassisRearFdCanRxMgrIns();
    chassis_rear_fdcan_tx_mgr_ptr = GetChassisRearFdCanTxMgrIns();

    gimbal_shooter_fdcan_rx_mgr_ptr = GetGimbalShooterFdCanRxMgrIns();
    gimbal_shooter_fdcan_tx_mgr_ptr = GetGimbalShooterFdCanTxMgrIns();

    rfr_uart_rx_mgr_ptr = GetRfrUartRxMgrIns();
    rfr_uart_tx_mgr_ptr = GetRfrUartTxMgrIns();

    rc_uart_rx_mgr_ptr = GetRcUartRxMgrIns();
}

static void CommAddReceiver(void)
{
    HW_ASSERT(chassis_front_fdcan_rx_mgr_ptr != nullptr, "chassis_front_fdcan_rx_mgr_ptr is nullptr", chassis_front_fdcan_rx_mgr_ptr);
    chassis_front_fdcan_rx_mgr_ptr->addReceiver(GetMotorWheelLeftFrontIns());
    chassis_front_fdcan_rx_mgr_ptr->addReceiver(GetMotorWheelRightFrontIns());
    chassis_front_fdcan_rx_mgr_ptr->addReceiver(GetCapIns());

    HW_ASSERT(chassis_rear_fdcan_rx_mgr_ptr != nullptr, "chassis_rear_fdcan_rx_mgr_ptr is nullptr", chassis_rear_fdcan_rx_mgr_ptr);
    chassis_rear_fdcan_rx_mgr_ptr->addReceiver(GetMotorWheelLeftRearIns());
    chassis_rear_fdcan_rx_mgr_ptr->addReceiver(GetMotorWheelRightRearIns());

    HW_ASSERT(gimbal_shooter_fdcan_rx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_rx_mgr_ptr is nullptr", gimbal_shooter_fdcan_rx_mgr_ptr);
    gimbal_shooter_fdcan_rx_mgr_ptr->addReceiver(GetMotorGimbalPitchIns());
    gimbal_shooter_fdcan_rx_mgr_ptr->addReceiver(GetMotorGimbalYawIns());
    gimbal_shooter_fdcan_rx_mgr_ptr->addReceiver(GetMotorGimbalRollIns());
    gimbal_shooter_fdcan_rx_mgr_ptr->addReceiver(GetMotorShooterFeedIns());
    gimbal_shooter_fdcan_rx_mgr_ptr->addReceiver(GetMotorShooterFricLeftIns());
    gimbal_shooter_fdcan_rx_mgr_ptr->addReceiver(GetMotorShooterFricRightIns());

    HW_ASSERT(rfr_uart_rx_mgr_ptr != nullptr, "rfr_uart_rx_mgr_ptr is nullptr", rfr_uart_rx_mgr_ptr);
    rfr_uart_rx_mgr_ptr->addReceiver(GetRfrIns());

    HW_ASSERT(rc_uart_rx_mgr_ptr != nullptr, "rc_uart_rx_mgr_ptr is nullptr", rc_uart_rx_mgr_ptr);
    rc_uart_rx_mgr_ptr->addReceiver(GetRcIns());
}

static void CommAddTransmitter(void)
{
    HW_ASSERT(chassis_front_fdcan_tx_mgr_ptr != nullptr, "chassis_front_fdcan_tx_mgr_ptr is nullptr", chassis_front_fdcan_tx_mgr_ptr);
    chassis_front_fdcan_tx_mgr_ptr->addTransmitter(GetMotorWheelLeftFrontIns());
    chassis_front_fdcan_tx_mgr_ptr->addTransmitter(GetMotorWheelRightFrontIns());
    chassis_front_fdcan_tx_mgr_ptr->addTransmitter(GetCapIns());

    HW_ASSERT(chassis_rear_fdcan_tx_mgr_ptr != nullptr, "chassis_rear_fdcan_tx_mgr_ptr is nullptr", chassis_rear_fdcan_tx_mgr_ptr);
    chassis_rear_fdcan_tx_mgr_ptr->addTransmitter(GetMotorWheelLeftRearIns());
    chassis_rear_fdcan_tx_mgr_ptr->addTransmitter(GetMotorWheelRightRearIns());

    HW_ASSERT(gimbal_shooter_fdcan_tx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_tx_mgr_ptr is nullptr", gimbal_shooter_fdcan_tx_mgr_ptr);
    gimbal_shooter_fdcan_tx_mgr_ptr->addTransmitter(GetMotorGimbalPitchIns());
    gimbal_shooter_fdcan_tx_mgr_ptr->addTransmitter(GetMotorGimbalYawIns());
    gimbal_shooter_fdcan_tx_mgr_ptr->addTransmitter(GetMotorGimbalRollIns());
    gimbal_shooter_fdcan_tx_mgr_ptr->addTransmitter(GetMotorShooterFeedIns());
    gimbal_shooter_fdcan_tx_mgr_ptr->addTransmitter(GetMotorShooterFricLeftIns());
    gimbal_shooter_fdcan_tx_mgr_ptr->addTransmitter(GetMotorShooterFricRightIns());

    HW_ASSERT(rfr_uart_tx_mgr_ptr != nullptr, "rfr_uart_tx_mgr_ptr is nullptr", rfr_uart_tx_mgr_ptr);
    rfr_uart_tx_mgr_ptr->addTransmitter(GetRfrIns());
    rfr_uart_tx_mgr_ptr->addTransmitter(GetUiMgrIns());
}

static void CommHardwareInit(void)
{
    /* FDCAN init */
    HW_ASSERT(chassis_front_fdcan_rx_mgr_ptr != nullptr, "chassis_front_fdcan_rx_mgr_ptr is nullptr", chassis_front_fdcan_rx_mgr_ptr);
    chassis_front_fdcan_rx_mgr_ptr->filterInit();
    chassis_front_fdcan_rx_mgr_ptr->startReceive();
    HAL_FDCAN_Start(chassis_front_fdcan);

    HW_ASSERT(chassis_rear_fdcan_rx_mgr_ptr != nullptr, "chassis_rear_fdcan_rx_mgr_ptr is nullptr", chassis_rear_fdcan_rx_mgr_ptr);
    chassis_rear_fdcan_rx_mgr_ptr->filterInit();
    chassis_rear_fdcan_rx_mgr_ptr->startReceive();
    HAL_FDCAN_Start(chassis_rear_fdcan);

    HW_ASSERT(gimbal_shooter_fdcan_rx_mgr_ptr != nullptr, "gimbal_shooter_fdcan_rx_mgr_ptr is nullptr", gimbal_shooter_fdcan_rx_mgr_ptr);
    gimbal_shooter_fdcan_rx_mgr_ptr->filterInit();
    gimbal_shooter_fdcan_rx_mgr_ptr->startReceive();
    HAL_FDCAN_Start(gimbal_shooter_fdcan);

    /* rc DMA init */
    HW_ASSERT(rc_uart_rx_mgr_ptr != nullptr, "rc_uart_rx_mgr_ptr is nullptr", rc_uart_rx_mgr_ptr);
    rc_uart_rx_mgr_ptr->startReceive();

    /* rfr DMA init */
    HW_ASSERT(rfr_uart_rx_mgr_ptr != nullptr, "rfr_uart_rx_mgr_ptr is nullptr", rfr_uart_rx_mgr_ptr);
    rfr_uart_rx_mgr_ptr->startReceive();
}

// USB CDC 回调桩函数 (NUC 已移除，保留空实现兼容 USB 栈)
void UsbReceiveCallBack()
{
}


