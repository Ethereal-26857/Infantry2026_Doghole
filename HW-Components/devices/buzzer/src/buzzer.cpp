/**
 *******************************************************************************
 * @file      : buzzer.cpp
 * @brief     : 实现蜂鸣器设备
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-28      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *  V1.0.1      2024-01-22      Caikunzhen      1. 修复频率计算错误问题
 *  V1.1.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 使用前请先确保定时器的频率为 1MHz，同时 STM32CubeMX 配置文件中 TIM 的 Counter
 *  Period 需大于 0，否则会报错
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "buzzer.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

#include <cmath>

#include "assert.hpp"
#include "base.hpp"
#include "tick.hpp"

namespace hello_world
{
namespace buzzer
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

static const float kTimerFreq = 1e6f;
static const float kMaxDuty = 0.5f;  ///* PWM 最大占空比
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t channel,
               PlayConfig play_config, const TuneListInfo *kTuneListInfoPtr)
    : htim_(htim), channel_(channel), play_config_(play_config)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(play_config == PlayConfig::kLoopPlayback ||
                play_config == PlayConfig::kSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
  HW_ASSERT(kTuneListInfoPtr != nullptr, "Error tune list info");
#pragma endregion

  setNewTune(kTuneListInfoPtr);
}

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t channel,
               PlayConfig play_config, const SimplifiedTuneListInfo *kSimplifiedTuneListInfoPtr)
    : htim_(htim), channel_(channel), play_config_(play_config)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(play_config == PlayConfig::kLoopPlayback ||
                play_config == PlayConfig::kSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
  HW_ASSERT(kSimplifiedTuneListInfoPtr != nullptr, "Error tune list info");
#pragma endregion

  setNewTune(kSimplifiedTuneListInfoPtr);
}

Buzzer &Buzzer::operator=(const Buzzer &other)
{
  if (this != &other) {
    htim_ = other.htim_;
    channel_ = other.channel_;
    hardware_inited_ = other.hardware_inited_;
    play_config_ = other.play_config_;
    intensity_scale_ = other.intensity_scale_;
    tune_duration_ = other.tune_duration_;
    tune_idx_ = other.tune_idx_;
    last_tune_ = other.last_tune_;
    tune_start_tick_ = other.tune_start_tick_;
    tune_switch_ = other.tune_switch_;
    is_playing_ = other.is_playing_;
    last_is_playing_ = other.last_is_playing_;
    tune_list_ = other.tune_list_;
    simplified_note_list_ = other.simplified_note_list_;
  }

  return *this;
}

Buzzer::Buzzer(Buzzer &&other)
    : htim_(other.htim_),
      channel_(other.channel_),
      hardware_inited_(other.hardware_inited_),
      intensity_scale_(other.intensity_scale_),
      tune_duration_(other.tune_duration_),
      play_config_(other.play_config_),
      tune_idx_(other.tune_idx_),
      last_tune_(other.last_tune_),
      tune_start_tick_(other.tune_start_tick_),
      tune_switch_(other.tune_switch_),
      is_playing_(other.is_playing_),
      last_is_playing_(other.last_is_playing_),
      tune_list_(other.tune_list_),
      simplified_note_list_(other.simplified_note_list_)
{
  other.htim_ = nullptr;
  other.tune_list_ = nullptr;
  other.simplified_note_list_ = nullptr;
}

Buzzer &Buzzer::operator=(Buzzer &&other)
{
  if (this != &other) {
    htim_ = other.htim_;
    channel_ = other.channel_;
    hardware_inited_ = other.hardware_inited_;
    play_config_ = other.play_config_;
    intensity_scale_ = other.intensity_scale_;
    tune_duration_ = other.tune_duration_;
    tune_idx_ = other.tune_idx_;
    last_tune_ = other.last_tune_;
    tune_start_tick_ = other.tune_start_tick_;
    tune_switch_ = other.tune_switch_;
    is_playing_ = other.is_playing_;
    last_is_playing_ = other.last_is_playing_;
    tune_list_ = other.tune_list_;
    simplified_note_list_ = other.simplified_note_list_;

    other.htim_ = nullptr;
    other.tune_list_ = nullptr;
    other.simplified_note_list_ = nullptr;
  }

  return *this;
}

void Buzzer::init(TIM_HandleTypeDef *htim, uint32_t channel,
                  PlayConfig play_config, const TuneListInfo *kTuneListInfoPtr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(play_config == PlayConfig::kLoopPlayback ||
                play_config == PlayConfig::kSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
  HW_ASSERT(kTuneListInfoPtr != nullptr, "Error tune list info");
#pragma endregion

  htim_ = htim;
  channel_ = channel;
  hardware_inited_ = false;
  play_config_ = play_config;

  setNewTune(kTuneListInfoPtr);
}

void Buzzer::init(TIM_HandleTypeDef *htim, uint32_t channel,
                  PlayConfig play_config,
                  const SimplifiedTuneListInfo *kSimplifiedTuneListInfoPtr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(play_config == PlayConfig::kLoopPlayback ||
                play_config == PlayConfig::kSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
  HW_ASSERT(kSimplifiedTuneListInfoPtr != nullptr, "Error tune list info");
#pragma endregion

  htim_ = htim;
  channel_ = channel;
  hardware_inited_ = false;
  play_config_ = play_config;

  setNewTune(kSimplifiedTuneListInfoPtr);
}

void Buzzer::initHardWare(void)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(htim_ != nullptr, "Error TIM handle");
  HW_ASSERT(IS_TIM_INSTANCE(htim_->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(channel_), "Error TIM channel: %d", channel);
#pragma endregion

  __HAL_TIM_SET_AUTORELOAD(htim_, 0);
  __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
  HAL_TIM_PWM_Start(htim_, channel_);
  hardware_inited_ = true;
}

void Buzzer::play(void)
{
  if (!hardware_inited_) {
    initHardWare();
  }

  if (!is_playing_) {
    return;
  }

  uint32_t tick = tick::GetTickMs();
  /* 刚开始播放 */
  if (!last_is_playing_ && is_playing_) {
    tune_start_tick_ = tick;
  }

  /* 播放结束处理 */
  if (tuneEndHandle()) {
    return;
  }

  /* 切换音符 */
  if (tune_switch_) {
    tune_switch_ = false;

    /* 计算重载值 */
    uint32_t auto_reload = tune2AutoReload(current_tune());

    /* 计算比较值 */
    uint32_t cmp = kMaxDuty * intensity_scale_ * auto_reload;
    
    __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
    __HAL_TIM_SET_COMPARE(htim_, channel_, cmp);
    HAL_TIM_GenerateEvent(htim_, TIM_EVENTSOURCE_UPDATE);
  }

  /* 音符播放达到要求时间后切换为下一音符 */
  if (tick - tune_start_tick_ >= tune_duration_) {
    tune_start_tick_ = tick;
    tune_idx_++;
    if (last_tune_ != current_tune()) {
      tune_switch_ = true;
      last_tune_ = current_tune();
    }
  }

  last_is_playing_ = is_playing_;
}

void Buzzer::setNewTune(const TuneListInfo *kTuneListInfoPtr)
{
  /* 变量检查 */
#pragma region

  HW_ASSERT(kTuneListInfoPtr != nullptr, "Error tune list info");
  HW_ASSERT(kTuneListInfoPtr->intensity_scale >= 0 &&
                kTuneListInfoPtr->intensity_scale <= 1,
            "Error intensity scale: %f", kTuneListInfoPtr->intensity_scale);
  HW_ASSERT(kTuneListInfoPtr->tune_duration > 0,
            "Error tune duration: %d", kTuneListInfoPtr->tune_duration);
#pragma endregion
  intensity_scale_ = kTuneListInfoPtr->intensity_scale;
  tune_duration_ = kTuneListInfoPtr->tune_duration;
  tonic_ = kTuneC4;
  tune_idx_ = 0;
  tune_start_tick_ = 0;
  tune_switch_ = true;
  is_playing_ = true;
  last_is_playing_ = false;
  tune_list_ = kTuneListInfoPtr->list;
  simplified_note_list_ = nullptr;

  __HAL_TIM_SET_AUTORELOAD(htim_, 0);
  __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
  HAL_TIM_PWM_Start(htim_, channel_);
}

void Buzzer::setNewTune(const SimplifiedTuneListInfo *kSimplifiedTuneInfoPtr)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(kSimplifiedTuneInfoPtr != nullptr, "Error tune list info");
  HW_ASSERT(kSimplifiedTuneInfoPtr->intensity_scale >= 0 &&
                kSimplifiedTuneInfoPtr->intensity_scale <= 1,
            "Error intensity scale: %f", kSimplifiedTuneInfoPtr->intensity_scale);
  HW_ASSERT(kSimplifiedTuneInfoPtr->tune_duration > 0,
            "Error tune duration: %d", kSimplifiedTuneInfoPtr->tune_duration);
  HW_ASSERT(kSimplifiedTuneInfoPtr->tonic >= kTuneB0 && kSimplifiedTuneInfoPtr->tonic <= kTuneC8,
            "Error tonic: %d", kSimplifiedTuneInfoPtr->tonic);
#pragma endregion

  intensity_scale_ = kSimplifiedTuneInfoPtr->intensity_scale;
  tune_duration_ = kSimplifiedTuneInfoPtr->tune_duration;
  tonic_ = kSimplifiedTuneInfoPtr->tonic;
  tune_idx_ = 0;
  last_tune_ = kTuneEnd;
  tune_start_tick_ = 0;
  tune_switch_ = true;
  is_playing_ = true;
  last_is_playing_ = false;
  tune_list_ = nullptr;
  simplified_note_list_ = kSimplifiedTuneInfoPtr->list;
}

void Buzzer::set_play_config(PlayConfig play_config)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(play_config == PlayConfig::kLoopPlayback ||
                play_config == PlayConfig::kSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
#pragma endregion

  play_config_ = play_config;
}

bool Buzzer::tuneEndHandle(void)
{
  /* 音符溢出则直接关闭播放 */
  if (tune_idx_ >= kTuneListMaxLen) {
    is_playing_ = last_is_playing_ = false;
    tune_switch_ = false;
    tune_idx_ = 0;
    mute();
    return true;
  }

  /* 播放至结尾处理 */
  if (current_tune() == kTuneEnd) {
    switch (play_config_) {
      case PlayConfig::kLoopPlayback:
        /* 播放重置 */
        tune_switch_ = true;
        tune_idx_ = 0;
        return false;

      case PlayConfig::kSinglePlayback:
        /* 关闭播放 */
        is_playing_ = last_is_playing_ = false;
        tune_switch_ = false;
        mute();
        return true;

      default:
        HW_ASSERT(false, "Error play config: %d", play_config_);
        return true;
    }
  }

  return false;
}

uint32_t Buzzer::tune2AutoReload(Tune tune) const
{
  if (tune < kTuneB0 || tune >= kTuneC8) {
    return 0;
  } else {
    uint32_t auto_reload = static_cast<uint32_t>(
        /* 440 * 2^((n-69)/12) Hz */
        kTimerFreq / (440.0f * powf(1.05946f, (tune)-69)) - 1);

    return auto_reload;
  }
}
/* Private function definitions ----------------------------------------------*/
}  // namespace buzzer
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */