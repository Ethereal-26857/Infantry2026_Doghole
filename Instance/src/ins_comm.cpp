#include "ins_comm.hpp"

#include "DT7.hpp"

typedef hello_world::comm::FdCanRxMgr FdCanRxMgr;
typedef hello_world::comm::FdCanTxMgr FdCanTxMgr;

typedef hello_world::comm::UartRxMgr UartRxMgr;
typedef hello_world::comm::UartTxMgr UartTxMgr;

static const size_t kRxRcBufferSize = hello_world::remote_control::DT7::kRcRxDataLen_ + 1;
static uint8_t rc_rx_buffer[kRxRcBufferSize] __attribute__((section(".RAM_D1"))) = {0};

static const size_t kRfrDmaRxBufferSize = 32;   ///< 裁判系统接收缓冲区大小
static uint8_t rfr_rx_buffer[kRfrDmaRxBufferSize] __attribute__((section(".RAM_D1"))) = {0};
static const size_t kRfrDmaTxBufferSize = 255;  ///< 裁判系统发送缓冲区大小
static uint8_t rfr_tx_buffer[kRfrDmaTxBufferSize] __attribute__((section(".RAM_D1"))) = {0};


static bool is_chassis_front_fdcan_rx_mgr_inited = false;
static FdCanRxMgr chassis_front_fdcan_rx_mgr = FdCanRxMgr();

static bool is_chassis_front_fdcan_tx_mgr_inited = false;
static FdCanTxMgr chassis_front_fdcan_tx_mgr = FdCanTxMgr();

static bool is_chassis_rear_fdcan_rx_mgr_inited = false;
static FdCanRxMgr chassis_rear_fdcan_rx_mgr = FdCanRxMgr();

static bool is_chassis_rear_fdcan_tx_mgr_inited = false;
static FdCanTxMgr chassis_rear_fdcan_tx_mgr = FdCanTxMgr();

static bool is_gimbal_shooter_fdcan_rx_mgr_inited = false;
static FdCanRxMgr gimbal_shooter_fdcan_rx_mgr = FdCanRxMgr();

static bool is_gimbal_shooter_fdcan_tx_mgr_inited = false;
static FdCanTxMgr gimbal_shooter_fdcan_tx_mgr = FdCanTxMgr();

static bool is_rfr_uart_rx_mgr_inited = false;  
static UartRxMgr rfr_uart_rx_mgr = UartRxMgr();

static bool is_rfr_uart_tx_mgr_inited = false;
static UartTxMgr rfr_uart_tx_mgr = UartTxMgr();

static bool is_rc_uart_rx_mgr_inited = false;
static UartRxMgr rc_uart_rx_mgr = UartRxMgr();

FdCanTxMgr::ItLine GetFdCanTxItLine(FDCAN_HandleTypeDef* tx_fdcan)
{
    if (tx_fdcan == &hfdcan1 || tx_fdcan == &hfdcan2)
    {
        return FdCanTxMgr::ItLine::k0;
    }
    else
    {
        return FdCanTxMgr::ItLine::k1;
    }
};

FdCanRxMgr::ItLine GetFdCanRxItLine(FDCAN_HandleTypeDef* rx_fdcan)
{
    if (rx_fdcan == &hfdcan1 || rx_fdcan == &hfdcan2)
    {
        return FdCanRxMgr::ItLine::k0;
    }
    else
    {
        return FdCanRxMgr::ItLine::k1;
    }
};

FdCanRxMgr::RxType GetFdCanRxType(FDCAN_HandleTypeDef* rx_fdcan)
{
    if (rx_fdcan == &hfdcan1 || rx_fdcan == &hfdcan2)
    {
        return FdCanRxMgr::RxType::kFifo0;
    }
    else
    {
        return FdCanRxMgr::RxType::kFifo1;
    }
};

FdCanRxMgr *GetChassisFrontFdCanRxMgrIns(void)
{
  if (!is_chassis_front_fdcan_rx_mgr_inited)
  {
    chassis_front_fdcan_rx_mgr.init(chassis_front_fdcan, GetFdCanRxType(chassis_front_fdcan));
    chassis_front_fdcan_rx_mgr.configInterruptLines(GetFdCanRxItLine(chassis_front_fdcan));
    chassis_front_fdcan_rx_mgr.clearReceiver();
    is_chassis_front_fdcan_rx_mgr_inited = true;
  }
  return &chassis_front_fdcan_rx_mgr;
};

FdCanTxMgr *GetChassisFrontFdCanTxMgrIns(void)
{
  if (!is_chassis_front_fdcan_tx_mgr_inited)
  {
    chassis_front_fdcan_tx_mgr.init(chassis_front_fdcan);
    chassis_front_fdcan_tx_mgr.configInterruptLines(GetFdCanTxItLine(chassis_front_fdcan));
    chassis_front_fdcan_tx_mgr.clearTransmitter();
    is_chassis_front_fdcan_tx_mgr_inited = true;
  }
  return &chassis_front_fdcan_tx_mgr;
};

FdCanRxMgr *GetChassisRearFdCanRxMgrIns(void)
{
  if (!is_chassis_rear_fdcan_rx_mgr_inited)
  {
    chassis_rear_fdcan_rx_mgr.init(chassis_rear_fdcan, GetFdCanRxType(chassis_rear_fdcan));
    chassis_rear_fdcan_rx_mgr.configInterruptLines(GetFdCanRxItLine(chassis_rear_fdcan));
    chassis_rear_fdcan_rx_mgr.clearReceiver();
    is_chassis_rear_fdcan_rx_mgr_inited = true;
  }
  return &chassis_rear_fdcan_rx_mgr;
};

FdCanTxMgr *GetChassisRearFdCanTxMgrIns(void)
{
  if (!is_chassis_rear_fdcan_tx_mgr_inited)
  {
    chassis_rear_fdcan_tx_mgr.init(chassis_rear_fdcan);
    chassis_rear_fdcan_tx_mgr.configInterruptLines(GetFdCanTxItLine(chassis_rear_fdcan));
    chassis_rear_fdcan_tx_mgr.clearTransmitter();
    is_chassis_rear_fdcan_tx_mgr_inited = true;
  }
  return &chassis_rear_fdcan_tx_mgr;
};

FdCanRxMgr *GetGimbalShooterFdCanRxMgrIns(void)
{
  if (!is_gimbal_shooter_fdcan_rx_mgr_inited)
  {
    gimbal_shooter_fdcan_rx_mgr.init(gimbal_shooter_fdcan, GetFdCanRxType(gimbal_shooter_fdcan));
    gimbal_shooter_fdcan_rx_mgr.configInterruptLines(GetFdCanRxItLine(gimbal_shooter_fdcan));
    gimbal_shooter_fdcan_rx_mgr.clearReceiver();
    is_gimbal_shooter_fdcan_rx_mgr_inited = true;
  }
  return &gimbal_shooter_fdcan_rx_mgr;
};

FdCanTxMgr *GetGimbalShooterFdCanTxMgrIns(void)
{
  if (!is_gimbal_shooter_fdcan_tx_mgr_inited)
  {
    gimbal_shooter_fdcan_tx_mgr.init(gimbal_shooter_fdcan);
    gimbal_shooter_fdcan_tx_mgr.configInterruptLines(GetFdCanTxItLine(gimbal_shooter_fdcan));
    gimbal_shooter_fdcan_tx_mgr.clearTransmitter();
    is_gimbal_shooter_fdcan_tx_mgr_inited = true;
  }
  return &gimbal_shooter_fdcan_tx_mgr;
};

UartRxMgr *GetRfrUartRxMgrIns(void)
{
  if (!is_rfr_uart_rx_mgr_inited)
  {
    rfr_uart_rx_mgr.init(rfr_uart, UartRxMgr::EofType::kManual, rfr_rx_buffer, kRfrDmaRxBufferSize, kRfrDmaRxBufferSize);
    rfr_uart_rx_mgr.clearReceiver();
    is_rfr_uart_rx_mgr_inited = true;
  }
  return &rfr_uart_rx_mgr;
};

UartTxMgr *GetRfrUartTxMgrIns(void)
{
  if (!is_rfr_uart_tx_mgr_inited)
  {
    rfr_uart_tx_mgr.init(rfr_uart, rfr_tx_buffer, kRfrDmaTxBufferSize);
    rfr_uart_tx_mgr.clearTransmitter();
    is_rfr_uart_tx_mgr_inited = true;
  }
  return &rfr_uart_tx_mgr;
};

UartRxMgr *GetRcUartRxMgrIns(void)
{
  if (!is_rc_uart_rx_mgr_inited)
  {
    rc_uart_rx_mgr.init(rc_uart, UartRxMgr::EofType::kIdle, rc_rx_buffer, kRxRcBufferSize, kRxRcBufferSize - 1);
    rc_uart_rx_mgr.clearReceiver();
    is_rc_uart_rx_mgr_inited = true;
  }
  return &rc_uart_rx_mgr;
};







