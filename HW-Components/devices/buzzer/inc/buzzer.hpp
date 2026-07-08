/**
 *******************************************************************************
 * @file      : buzzer.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_BUZZER_BUZZER_HPP_
#define HW_COMPONENTS_DEVICES_BUZZER_BUZZER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

#include "allocator.hpp"

namespace hello_world
{
namespace buzzer
{
/* Exported macro ------------------------------------------------------------*/

#pragma region 音符定义
/* 五线谱音符 */
enum Tune : uint8_t {
  kTuneA0 = 21U,        ///* A0
  kTuneA0S,             ///* A0# (Sharp)
  kTuneB0F = kTuneA0S,  ///* B0b (Flat)
  kTuneB0,              ///* B0

  kTuneC1,              ///* C1
  kTuneC1S,             ///* C1# (Sharp)
  kTuneD1F = kTuneC1S,  ///* D1b (Flat)
  kTuneD1,              ///* D1
  kTuneD1S,             ///* D1# (Sharp)
  kTuneE1F = kTuneD1S,  ///* E1b (Flat)
  kTuneE1,              ///* E1
  kTuneF1,              ///* F1
  kTuneF1S,             ///* F1# (Sharp)
  kTuneG1F = kTuneF1S,  ///* G1b (Flat)
  kTuneG1,              ///* G1
  kTuneG1S,             ///* G1# (Sharp)
  kTuneA1F = kTuneG1S,  ///* A1b (Flat)
  kTuneA1,              ///* A1
  kTuneA1S,             ///* A1# (Sharp)
  kTuneB1F = kTuneA1S,  ///* B1b (Flat)
  kTuneB1,              ///* B1

  kTuneC2,              ///* C2
  kTuneC2S,             ///* C2# (Sharp)
  kTuneD2F = kTuneC2S,  ///* D2b (Flat)
  kTuneD2,              ///* D2
  kTuneD2S,             ///* D2# (Sharp)
  kTuneE2F = kTuneD2S,  ///* E2b (Flat)
  kTuneE2,              ///* E2
  kTuneF2,              ///* F2
  kTuneF2S,             ///* F2# (Sharp)
  kTuneG2F = kTuneF2S,  ///* G2b (Flat)
  kTuneG2,              ///* G2
  kTuneG2S,             ///* G2# (Sharp)
  kTuneA2F = kTuneG2S,  ///* A2b (Flat)
  kTuneA2,              ///* A2
  kTuneA2S,             ///* A2# (Sharp)
  kTuneB2F = kTuneA2S,  ///* B2b (Flat)
  kTuneB2,              ///* B2

  kTuneC3,              ///* C3
  kTuneC3S,             ///* C3# (Sharp)
  kTuneD3F = kTuneC3S,  ///* D3b (Flat)
  kTuneD3,              ///* D3
  kTuneD3S,             ///* D3# (Sharp)
  kTuneE3F = kTuneD3S,  ///* E3b (Flat)
  kTuneE3,              ///* E3
  kTuneF3,              ///* F3
  kTuneF3S,             ///* F3# (Sharp)
  kTuneG3F = kTuneF3S,  ///* G3b (Flat)
  kTuneG3,              ///* G3
  kTuneG3S,             ///* G3# (Sharp)
  kTuneA3F = kTuneG3S,  ///* A3b (Flat)
  kTuneA3,              ///* A3
  kTuneA3S,             ///* A3# (Sharp)
  kTuneB3F = kTuneA3S,  ///* B3b (Flat)
  kTuneB3,              ///* B3

  kTuneC4,              ///* C4 (中央C)
  kTuneC4S,             ///* C4# (Sharp)
  kTuneD4F = kTuneC4S,  ///* D4b (Flat)
  kTuneD4,              ///* D4
  kTuneD4S,             ///* D4# (Sharp)
  kTuneE4F = kTuneD4S,  ///* E4b (Flat)
  kTuneE4,              ///* E4
  kTuneF4,              ///* F4
  kTuneF4S,             ///* F4# (Sharp)
  kTuneG4F = kTuneF4S,  ///* G4b (Flat)
  kTuneG4,              ///* G4
  kTuneG4S,             ///* G4# (Sharp)
  kTuneA4F = kTuneG4S,  ///* A4b (Flat)
  kTuneA4,              ///* A4, 440Hz
  kTuneA4S,             ///* A4# (Sharp)
  kTuneB4F = kTuneA4S,  ///* B4b (Flat)
  kTuneB4,              ///* B4

  kTuneC5,              ///* C5
  kTuneC5S,             ///* C5# (Sharp)
  kTuneD5F = kTuneC5S,  ///* D5b (Flat)
  kTuneD5,              ///* D5
  kTuneD5S,             ///* D5# (Sharp)
  kTuneE5F = kTuneD5S,  ///* E5b (Flat)
  kTuneE5,              ///* E5
  kTuneF5,              ///* F5
  kTuneF5S,             ///* F5# (Sharp)
  kTuneG5F = kTuneF5S,  ///* G5b (Flat)
  kTuneG5,              ///* G5
  kTuneG5S,             ///* G5# (Sharp)
  kTuneA5F = kTuneG5S,  ///* A5b (Flat)
  kTuneA5,              ///* A5
  kTuneA5S,             ///* A5# (Sharp)
  kTuneB5F = kTuneA5S,  ///* B5b (Flat)
  kTuneB5,              ///* B5

  kTuneC6,              ///* C6
  kTuneC6S,             ///* C6# (Sharp)
  kTuneD6F = kTuneC6S,  ///* D6b (Flat)
  kTuneD6,              ///* D6
  kTuneD6S,             ///* D6# (Sharp)
  kTuneE6F = kTuneD6S,  ///* E6b (Flat)
  kTuneE6,              ///* E6
  kTuneF6,              ///* F6
  kTuneF6S,             ///* F6# (Sharp)
  kTuneG6F = kTuneF6S,  ///* G6b (Flat)
  kTuneG6,              ///* G6
  kTuneG6S,             ///* G6# (Sharp)
  kTuneA6F = kTuneG6S,  ///* A6b (Flat)
  kTuneA6,              ///* A6
  kTuneA6S,             ///* A6# (Sharp)
  kTuneB6F = kTuneA6S,  ///* B6b (Flat)
  kTuneB6,              ///* B6

  kTuneC7,              ///* C7
  kTuneC7S,             ///* C7# (Sharp)
  kTuneD7F = kTuneC7S,  ///* D7b (Flat)
  kTuneD7,              ///* D7
  kTuneD7S,             ///* D7# (Sharp)
  kTuneE7F = kTuneD7S,  ///* E7b (Flat)
  kTuneE7,              ///* E7
  kTuneF7,              ///* F7
  kTuneF7S,             ///* F7# (Sharp)
  kTuneG7F = kTuneF7S,  ///* G7b (Flat)
  kTuneG7,              ///* G7
  kTuneG7S,             ///* G7# (Sharp)
  kTuneA7F = kTuneG7S,  ///* A7b (Flat)
  kTuneA7,              ///* A7
  kTuneA7S,             ///* A7# (Sharp)
  kTuneB7F = kTuneA7S,  ///* B7b (Flat)
  kTuneB7,              ///* B7

  kTuneC8,  ///* C8

  kTuneRst = 109U,  ///* 休止符
  kTuneEnd = 110U,  ///* 结束标志
};

/* 简谱音符 */
enum SimplifiedNote : int8_t {
  kLL1 = -24,     ///* 倍低音 1
  kLL1S,          ///* 倍低音 1# (Sharp)
  kLL2F = kLL1S,  ///* 倍低音 2b (Flat)
  kLL2,           ///* 倍低音 2
  kLL2S,          ///* 倍低音 2# (Sharp)
  kLL3F = kLL2S,  ///* 倍低音 3b (Flat)
  kLL3,           ///* 倍低音 3
  kLL4,           ///* 倍低音 4
  kLL4S,          ///* 倍低音 4# (Sharp)
  kLL5F = kLL4S,  ///* 倍低音 5b (Flat)
  kLL5,           ///* 倍低音 5
  kLL5S,          ///* 倍低音 5# (Sharp)
  kLL6F = kLL5S,  ///* 倍低音 6b (Flat)
  kLL6,           ///* 倍低音 6
  kLL6S,          ///* 倍低音 6# (Sharp)
  kLL7F = kLL6S,  ///* 倍低音 7b (Flat)
  kLL7,           ///* 倍低音 7

  kL1 = -12,    ///* 低音 1
  kL1S,         ///* 低音 1# (Sharp)
  kL2F = kL1S,  ///* 低音 2b (Flat)
  kL2,          ///* 低音 2
  kL2S,         ///* 低音 2# (Sharp)
  kL3F = kL2S,  ///* 低音 3b (Flat)
  kL3,          ///* 低音 3
  kL4,          ///* 低音 4
  kL4S,         ///* 低音 4# (Sharp)
  kL5F = kL4S,  ///* 低音 5b (Flat)
  kL5,          ///* 低音 5
  kL5S,         ///* 低音 5# (Sharp)
  kL6F = kL5S,  ///* 低音 6b (Flat)
  kL6,          ///* 低音 6
  kL6S,         ///* 低音 6# (Sharp)
  kL7F = kL6S,  ///* 低音 7b (Flat)
  kL7,          ///* 低音 7

  kM1 = 0,      ///* 中音 1
  kM1S,         ///* 中音 1# (Sharp)
  kM2F = kM1S,  ///* 中音 2b (Flat)
  kM2,          ///* 中音 2
  kM2S,         ///* 中音 2# (Sharp)
  kM3F = kM2S,  ///* 中音 3b (Flat)
  kM3,          ///* 中音 3
  kM4,          ///* 中音 4
  kM4S,         ///* 中音 4# (Sharp)
  kM5F = kM4S,  ///* 中音 5b (Flat)
  kM5,          ///* 中音 5
  kM5S,         ///* 中音 5# (Sharp)
  kM6F = kM5S,  ///* 中音 6b (Flat)
  kM6,          ///* 中音 6
  kM6S,         ///* 中音 6# (Sharp)
  kM7F = kM6S,  ///* 中音 7b (Flat)
  kM7,          ///* 中音 7

  kH1 = 12,     ///* 高音 1
  kH1S,         ///* 高音 1# (Sharp)
  kH2F = kH1S,  ///* 高音 2b (Flat)
  kH2,          ///* 高音 2
  kH2S,         ///* 高音 2# (Sharp)
  kH3F = kH2S,  ///* 高音 3b (Flat)
  kH3,          ///* 高音 3
  kH4,          ///* 高音 4
  kH4S,         ///* 高音 4# (Sharp)
  kH5F = kH4S,  ///* 高音 5b (Flat)
  kH5,          ///* 高音 5
  kH5S,         ///* 高音 5# (Sharp)
  kH6F = kH5S,  ///* 高音 6b (Flat)
  kH6,          ///* 高音 6
  kH6S,         ///* 高音 6# (Sharp)
  kH7F = kH6S,  ///* 高音 7b (Flat)
  kH7,          ///* 高音 7

  kHH1 = 24,      ///* 倍高音 1
  kHH1S,          ///* 倍高音 1# (Sharp)
  kHH2F = kHH1S,  ///* 倍高音 2b (Flat)
  kHH2,           ///* 倍高音 2
  kHH2S,          ///* 倍高音 2# (Sharp)
  kHH3F = kHH2S,  ///* 倍高音 3b (Flat)
  kHH3,           ///* 倍高音 3
  kHH4,           ///* 倍高音 4
  kHH4S,          ///* 倍高音 4# (Sharp)
  kHH5F = kHH4S,  ///* 倍高音 5b (Flat)
  kHH5,           ///* 倍高音 5
  kHH5S,          ///* 倍高音 5# (Sharp)
  kHH6F = kHH5S,  ///* 倍高音 6b (Flat)
  kHH6,           ///* 倍高音 6
  kHH6S,          ///* 倍高音 6# (Sharp)
  kHH7F = kHH6S,  ///* 倍高音 7b (Flat)
  kHH7,           ///* 倍高音 7

  kRst = 109,  ///* 休止符
  kEnd = 110,  ///* 结束标志
};
#pragma endregion

enum class PlayConfig : uint8_t {
  kLoopPlayback,    ///* 循环播放
  kSinglePlayback,  ///* 单次播放
};

/* Exported constants --------------------------------------------------------*/

static const size_t kTuneListMaxLen = 4096u;
/* Exported types ------------------------------------------------------------*/

/** 五线谱乐曲信息，建议声明为常量 */
struct TuneListInfo : public MemMgr {
  float intensity_scale = 1;                ///* 声音强度，(0, 1]
  uint16_t tune_duration = 1;               ///* 单音持续时长，单位：ms
  Tune list[kTuneListMaxLen] = {kTuneEnd};  ///* 音符列表，必须以 kTuneEnd 结尾
};

/** 简谱乐曲信息，建议声明为常量 */
struct SimplifiedTuneListInfo : public MemMgr {
  float intensity_scale = 1;                      ///* 声音强度，(0, 1]
  uint16_t tune_duration = 1;                     ///* 基础音符时长，单位：ms
  Tune tonic = kTuneC4;                           ///* 音阶基准音符，默认为中央C
  SimplifiedNote list[kTuneListMaxLen] = {kEnd};  ///* 简谱音符列表，必须以 kEnd 结尾
};

class Buzzer : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  Buzzer(void) = default;
  /**
   * @brief       蜂鸣器初始化
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器PWM对应输出通道，可选值为：
   *   @arg        TIM_CHANNEL_x：定时器通道x，x = 1, 2, 3, 4
   * @param        play_config: 播放配置，可选值为：
   *   @arg        PlayConfig::kLoopPlayback: 循环播放
   *   @arg        PlayConfig::kSinglePlayback: 单次播放
   * @param        kTuneListInfoPtr: 乐曲信息指针，请确保播放期间指针所指内容没有被
   *               释放
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，声音强度与播放的音符会被自行限定
   *              到合理取值范围中，蜂鸣器默认开启
   */
  Buzzer(TIM_HandleTypeDef *htim, uint32_t channel, PlayConfig play_config,
         const TuneListInfo *kTuneListInfoPtr);

  /**
   * @brief       蜂鸣器初始化
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器PWM对应输出通道，可选值为：
   *   @arg        TIM_CHANNEL_x：定时器通道x，x = 1, 2, 3, 4
   * @param        play_config: 播放配置，可选值为：
   *   @arg        PlayConfig::kLoopPlayback: 循环播放
   *   @arg        PlayConfig::kSinglePlayback: 单次播放
   * @param        kSimplifiedTuneListInfoPtr: 乐曲简谱信息指针，
   *               请确保播放期间指针所指内容没有被释放
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，声音强度与播放的音符会被自行限定
   *              到合理取值范围中，蜂鸣器默认开启
   */
  Buzzer(TIM_HandleTypeDef *htim, uint32_t channel, PlayConfig play_config,
         const SimplifiedTuneListInfo *kSimplifiedTuneListInfoPtr);
  Buzzer(const Buzzer &other) = default;
  Buzzer &operator=(const Buzzer &other);
  Buzzer(Buzzer &&other);
  Buzzer &operator=(Buzzer &&other);

  virtual ~Buzzer(void) {}

  /* 配置方法 */

  /**
   * @brief       蜂鸣器初始化，使用默认构造函数后请务必调用此函数
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器PWM对应输出通道，可选值为：
   *   @arg        TIM_CHANNEL_x：定时器通道x，x = 1, 2, 3, 4
   * @param        play_config: 播放配置，可选值为：
   *   @arg        PlayConfig::kLoopPlayback: 循环播放
   *   @arg        PlayConfig::kSinglePlayback: 单次播放
   * @param        kTuneListInfoPtr: 乐曲信息指针，请确保播放期间指针所指内容没有被
   *               释放
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，声音强度与播放的音符会被自行限定
   *              到合理取值范围中，蜂鸣器开启
   */
  void init(TIM_HandleTypeDef *htim, uint32_t channel, PlayConfig play_config,
            const TuneListInfo *kTuneListInfoPtr);

  /**
   * @brief       蜂鸣器初始化，使用默认构造函数后请务必调用此函数
   * @param        htim: 定时器句柄指针
   * @param        channel: 定时器PWM对应输出通道，可选值为：
   *   @arg        TIM_CHANNEL_x：定时器通道x，x = 1, 2, 3, 4
   * @param        play_config: 播放配置，可选值为：
   *   @arg        PlayConfig::kLoopPlayback: 循环播放
   *   @arg        PlayConfig::kSinglePlayback: 单次播放
   * @param        kSimplifiedTuneListInfoPtr: 乐曲简谱信息指针，
   *               请确保播放期间指针所指内容没有被释放
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，声音强度与播放的音符会被自行限定
   *              到合理取值范围中，蜂鸣器开启
   */
  void init(TIM_HandleTypeDef *htim, uint32_t channel, PlayConfig play_config,
            const SimplifiedTuneListInfo *kSimplifiedTuneListInfoPtr);

  /* 功能性方法 */

  /**
   * @brief       关闭蜂鸣器
   * @retval       None
   * @note        None
   */
  void mute(void) const { HAL_TIM_PWM_Stop(htim_, channel_); }

  /**
   * @brief       播放乐曲
   * @retval       None
   * @note        请确保该方法以至少 1kHz 的频率被调用
   */
  void play(void);

  /**
   * @brief       设置新的乐曲信息
   * @param        kTuneListInfoPtr: 乐曲信息指针，请确保播放期间指针所指内容没有被
   *               释放
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，声音强度与播放的音符会被自行限定
   *              到合理取值范围中，调用后蜂鸣器会自动开启
   */
  void setNewTune(const TuneListInfo *kTuneListInfoPtr);

  /**
   * @brief       设置新的乐曲信息
   * @param        kSimplifiedTuneListInfoPtr: 乐曲简谱信息指针，
   *               请确保播放期间指针所指内容没有被释放
   * @retval       None
   * @note        使用前请先确保定时器的频率为 1MHz，声音强度与播放的音符会被自行限定
   *              到合理取值范围中，调用后蜂鸣器会自动开启
   */
  void setNewTune(const SimplifiedTuneListInfo *kSimplifiedTuneInfoPtr);

  /**
   * @brief       初始化 buzzer 硬件
   * @retval       None
   * @note        在首次 play 前由 Buzzer 自动调用
   */
  void initHardWare(void);

  /* 数据修改与获取 */

  const TIM_HandleTypeDef *htim(void) const { return htim_; }

  uint32_t channel(void) const { return channel_; }

  bool is_playing(void) const { return is_playing_; }

  PlayConfig get_play_config(void) const { return play_config_; }

  Tune current_tune(void) const
  {
    if (tune_list_ != nullptr) {
      return tune_list_[tune_idx_];
    } else if (simplified_note_list_ != nullptr) {
      if (simplified_note_list_[tune_idx_] == kRst) {
        return kTuneRst;
      } else if (simplified_note_list_[tune_idx_] == kEnd) {
        return kTuneEnd;
      }
      int8_t tune_tmp =
          static_cast<int8_t>(simplified_note_list_[tune_idx_]) +
          static_cast<int8_t>(tonic_);
      if (kTuneA0 <= tune_tmp && tune_tmp <= kTuneB7) {
        return static_cast<Tune>(tune_tmp);
      }
    }

    return kTuneEnd;
  }

  /**
   * @brief       设置播放配置
   * @param        play_config: 播放配置，可选值为：
   *   @arg        PlayConfig::kLoopPlayback: 循环播放
   *   @arg        PlayConfig::kSinglePlayback: 单次播放
   * @retval       None
   * @note        None
   */
  void set_play_config(PlayConfig play_config);

 private:
  /**
   * @brief       乐曲播放结束处理
   * @retval       播放是否结束
   * @note        None
   */
  bool tuneEndHandle(void);

  /**
   * @brief       将音符转化为对应的定时器重载值
   * @param        tune: 音符
   * @retval       对应的定时器重载值
   * @note        音符会被自行限定到合理取值范围中
   */
  uint32_t tune2AutoReload(Tune tune) const;

  /* 硬件相关 */

  TIM_HandleTypeDef *htim_ = nullptr;
  uint32_t channel_ = TIM_CHANNEL_1;
  bool hardware_inited_ = false;

  /* 乐曲配置相关 */

  const Tune *tune_list_ = nullptr;                       ///* 五线谱音符列表
  const SimplifiedNote *simplified_note_list_ = nullptr;  ///* 简谱音符列表
  PlayConfig play_config_ = PlayConfig::kSinglePlayback;  ///* 播放配置
  Tune tonic_ = kTuneC4;                                  ///* 音阶基准音符，默认为中央C

  /* 乐曲播放相关 */

  float intensity_scale_ = 1.0f;  ///* 声音强度，(0, 1]
  uint32_t tune_duration_ = 0;    ///* 音符持续时间，单位：us
  size_t tune_idx_ = 0;           ///* 当前音符索引
  Tune last_tune_ = kTuneEnd;      ///* 上一个音符
  uint32_t tune_start_tick_ = 0;  ///* 当前音符开始时间
  bool tune_switch_ = false;      ///* 音符切换标志
  bool is_playing_ = false;
  bool last_is_playing_ = false;
};
/* Exported variables --------------------------------------------------------*/
#pragma region 曲谱示例
static const TuneListInfo kWarningList1B = {
    .intensity_scale = 1.0f,
    .tune_duration = 500,
    .list = {kTuneA4, kTuneRst, kTuneEnd}};

static const TuneListInfo kWarningList2B = {
    .intensity_scale = 1.0f,
    .tune_duration = 250,
    .list = {kTuneA4, kTuneRst, kTuneA4, kTuneRst, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneEnd}};

static const TuneListInfo kWarningList3B = {
    .intensity_scale = 1.0f,
    .tune_duration = 167,
    .list = {kTuneA4, kTuneRst, kTuneA4, kTuneRst, kTuneA4, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneRst, kTuneRst, kTuneRst,
             kTuneEnd}};

static const TuneListInfo kWarningList4B = {
    .intensity_scale = 1.0f,
    .tune_duration = 125,
    .list = {kTuneA4, kTuneRst, kTuneA4, kTuneRst,
             kTuneA4, kTuneRst, kTuneA4, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneRst, kTuneEnd}};

/** 德彪西-贝加梅斯克组曲IV */
static const TuneListInfo kMusicDemo = {
    .intensity_scale = 0.005f,
    .tune_duration = 100,
    .list = {
        kTuneRst, kTuneRst, kTuneRst, kTuneRst,
        kTuneRst, kTuneRst, kTuneRst, kTuneRst,
        kTuneG5F, kTuneG5F, kTuneG5F, kTuneG5F,
        kTuneG5F, kTuneG5F, kTuneG5F, kTuneG5F,

        kTuneG5F, kTuneG5F, kTuneG5F, kTuneRst,
        kTuneD6F, kTuneD6F, kTuneRst, kTuneRst,
        kTuneB5, kTuneB5, kTuneB5, kTuneB5,
        kTuneB5, kTuneB5, kTuneB5, kTuneB5,

        kTuneB5, kTuneB5, kTuneB5, kTuneRst,
        kTuneA5, kTuneRst, kTuneA5F, kTuneRst,
        kTuneG5F, kTuneG5F, kTuneRst, kTuneRst,
        kTuneE5, kTuneRst, kTuneD5, kTuneRst,

        kTuneRst, kTuneRst, kTuneD5F, kTuneRst,
        kTuneD5, kTuneRst, kTuneE5, kTuneRst,
        kTuneG5F, kTuneG5F, kTuneRst, kTuneRst,
        kTuneA5, kTuneA5, kTuneRst, kTuneRst,

        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,

        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneRst,
        kTuneA5, kTuneA5, kTuneA5, kTuneRst,
        kTuneA6F, kTuneA6F, kTuneRst, kTuneRst,

        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneG5F, kTuneG5F, kTuneG5F, kTuneRst,
        kTuneE6, kTuneE6, kTuneRst, kTuneRst,

        kTuneE5, kTuneE5, kTuneE5, kTuneE5,
        kTuneRst, kTuneRst, kTuneRst, kTuneRst,
        kTuneE5F, kTuneE5F, kTuneE5F, kTuneRst,
        kTuneD6F, kTuneD6F, kTuneRst, kTuneRst,

        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,
        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,
        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,
        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,

        kTuneEnd}};
#pragma endregion
/* Exported function prototypes ----------------------------------------------*/
}  // namespace buzzer
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* HW_COMPONENTS_DEVICES_BUZZER_BUZZER_HPP_ */
