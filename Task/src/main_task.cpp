#include "main_task.hpp"

#include "comm_task.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "tim.h"

static robot::Robot *robot_ptr = nullptr;

static uint32_t start_time = 0;
static uint32_t main_task_time_cost = 0;  // MainTask 耗时 (us)
static uint32_t total_time_cost = 0;      // 总耗时 (us)
static uint32_t diff_time = 0;            // 两次定时器中断之间的时间差 (us)
static uint32_t debug_count = 0;

static void PrivatePointerInit(void);
static void HardwareInit(void);

void MainTaskInit(void)
{
  PrivatePointerInit();
  CommTaskInit();

  HardwareInit();
}

void MainTask(void)
{
  HW_ASSERT(robot_ptr != nullptr, "robot::Robot is nullptr", robot_ptr);
  robot_ptr->update();
  robot_ptr->run();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim == &htim6)
  {
    debug_count++;
    if (start_time > 0) {
      diff_time = hello_world::tick::GetTickUs() - start_time;
    }
   
    start_time = hello_world::tick::GetTickUs();
    //无才，无德，无能
    MainTask();
    main_task_time_cost = hello_world::tick::GetTickUs() - start_time;

    CommTask();
    total_time_cost = hello_world::tick::GetTickUs() - start_time;
  }
}

static void PrivatePointerInit(void)
{
  robot_ptr = GetRobotIns();
};

static void HardwareInit(void)
{
  // imu init
  robot_ptr->imuInitHardware();

  // buzzer init

  // iwdg init
  MX_IWDG1_Init();
  
  // Tim init
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
}