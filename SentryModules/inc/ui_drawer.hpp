/** 
 *******************************************************************************
 * @file      :ui_drawer.hpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UI_DRAWER_HPP_
#define UI_DRAWER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "fsm.hpp"
#include "feed.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "module_state.hpp"
#include "ui_mgr.hpp"
#include "rfr_pkg/rfr_pkg_0x0003_comp_robots_hp.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef hello_world::referee::Graphic::Color GraphicColor;
typedef hello_world::referee::Pixel Pixel;
typedef hello_world::referee::UiMgr UiMgr;

enum RobotsId {
  Hero = 0u,
  Engineer,
  Standard3,
  Standard4,
  Sentry,
  RobotsNum,  ///< 需要读血量的机器人数量
};

class UiDrawer
{
 public:
  typedef hello_world::referee::GraphicOperation GraphicOperation;
  typedef hello_world::referee::ids::RobotId RobotId;
  typedef hello_world::referee::String String;
  typedef hello_world::referee::StraightLine StraightLine;
  typedef hello_world::referee::Rectangle Rectangle;
  typedef hello_world::referee::Circle Circle;
  typedef hello_world::referee::FloatingNumber FloatingNumber;
  typedef hello_world::referee::Arc Arc;
  typedef hello_world::referee::Integer Integer;

  typedef robot::Chassis::WorkingMode ChassisWorkingMode;
  typedef hello_world::module::CtrlMode FsmCtrlMode;
  typedef hello_world::module::ManualCtrlSrc FsmManualCtrlSrc;
  typedef robot::PwrState FsmWorkState;
  typedef robot::GimbalJointWorkingMode GimbalWorkingMode;
  typedef robot::ShooterWorkingMode ShooterWorkingMode;
  
  
  enum StaticUiIdx {
    kSuiDelAll = 0,
    kSuiPassLinePkgGroup1,
    kSuiPassLinePkgGroup2,
    kSuiChassisTitle,
    kSuiGimbalTitle,
    kSuiPkgNum,
  };

  enum DynamicUiIdx {
    kDuiChassisContent,
    kDuiGimbalContent,
    kDuiPkgGroup1,  ///< 云台 pitch yaw 角度反馈，拨盘角度反馈，摩擦轮转速反馈，超电，底盘朝向(2)
    kDuiPkgGroup2,  ///< 云台 pitch yaw 角度期望，拨盘角度期望，摩擦轮转速期望，小云台预设值，小云台当前俯仰角度
    kDuiPkgGroup3,
    kDuiPkgGroup4,
    kDuiPkgGroup5,
    kDuiPkgNum,
  };

  static constexpr size_t kNumAllPkgs = (size_t)kDuiPkgNum + (size_t)kSuiPkgNum;

  UiDrawer(){};
  ~UiDrawer(){};

#pragma region 接口函数
  //操作接口
  void initUiDrawer(UiMgr *ui_mgr_ptr)
  {
    HW_ASSERT(ui_mgr_ptr != nullptr, "UiMgr pointer cannot be null.");

    // 设置静态字符串
    setStaticStrings();
    // 设置静态 UI
    setStaticGraphics();
    // 添加图形
    addGraphics(ui_mgr_ptr);
  };
  void updateDynamicUi()
  {
    // 更新动态字符串
    updateDynamicStrings();
    // 更新视觉相关的UI
    updateVisTgtCir(cir_vis_tgt_);
    //更新发弹热量
    updateShooterHeat(arc_heat_);
    updateCapPwrPercent(rec_cap_pwr_percent_,f_cap_pwr_percent_num_);
    updateBufferCapPwrPercent(rec_buffer_cap_pwr_percent_,f_buffer_cap_pwr_percent_num_);
   
    updateVisionbox(rec_vision_box_);
    // 更新裁判系统相关的UI
    // updateRobotsHp(int_red_1_hp_, int_red_2_hp_, int_red_3_hp_, int_red_4_hp_, int_red_7_hp_, int_blue_1_hp_, int_blue_2_hp_, int_blue_3_hp_, int_blue_4_hp_,
    //                int_blue_7_hp_);
    updateBulletNum(f_bullet_num_);
    updateChassisStatus(arc_head_,arc_other_);
    updateChassisPassLineLeft(sline_pass_line_left_);
    updateChassisPassLineRight(sline_pass_line_right_);
    updateChassisDirection(sline_chassis_direction_);
    updateChassisJumpLine(sline_jump_line_);
    updateArmorHit(arc_amor_);
    updateChassisForwardDistance(f_forward_distance_);
  }
#pragma endregion 接口函数

 #pragma region 设置需求参数
  //设置机器人id
  void setSenderId(RobotId id) { sender_id_ = id; }
  //设置机器人血量
  void setRobotsHp(hello_world::referee::CompRobotsHpData robots_hp)
  {
    robots_hp_[robot::RobotsId::Hero] = robots_hp.ally_1_robot_HP;
    robots_hp_[robot::RobotsId::Engineer] = robots_hp.ally_2_robot_HP;
    robots_hp_[robot::RobotsId::Standard3] = robots_hp.ally_3_robot_HP;
    robots_hp_[robot::RobotsId::Standard4] = robots_hp.ally_4_robot_HP;
    robots_hp_[robot::RobotsId::Sentry] = robots_hp.ally_7_robot_HP;

    for(size_t i = 0; i < RobotsNum; ++i) {
      if (robots_hp_[i] > robots_max_hp_[i]) {
        robots_max_hp_[i] = robots_hp_[i];  // 刷新最大血量
      }
    }
  }
  //设置底盘相关状态
  void setChassisWorkState(FsmWorkState state)
  {
    if (state != chassis_work_state_) {
      last_chassis_work_state_ = chassis_work_state_;
      chassis_work_state_ = state;
    }
  }
  void setChassisCtrlMode(FsmCtrlMode mode)
  {
    if (mode != chassis_ctrl_mode_) {
      last_chassis_ctrl_mode_ = chassis_ctrl_mode_;
      chassis_ctrl_mode_ = mode;
    }
  }
  void setChassisManualCtrlSrc(FsmManualCtrlSrc src)
  {
    if (src != chassis_manual_ctrl_src_) {
      last_chassis_manual_ctrl_src_ = chassis_manual_ctrl_src_;
      chassis_manual_ctrl_src_ = src;
    }
  }
  void setChassisWorkingMode(ChassisWorkingMode mode)
  {
    if (mode != chassis_working_mode_) {
      last_chassis_working_mode_ = chassis_working_mode_;
      chassis_working_mode_ = mode;
    }
  }
  void setChassisHeadDir(float theta_i2r) { theta_i2r_ = theta_i2r; }
  //设置云台相关状态
  void setGimbalWorkState(FsmWorkState state)
  {
    if (state != gimbal_work_state_) {
      last_gimbal_work_state_ = gimbal_work_state_;
      gimbal_work_state_ = state;
    }
  }
  void setGimbalCtrlMode(FsmCtrlMode mode)
  {
    if (mode != gimbal_ctrl_mode_) {
      last_gimbal_ctrl_mode_ = gimbal_ctrl_mode_;
      gimbal_ctrl_mode_ = mode;
    }
  }
  void setGimbalWorkingMode(GimbalWorkingMode mode)
  {
    if (mode != gimbal_working_mode_) {
      last_gimbal_working_mode_ = gimbal_working_mode_;
      gimbal_working_mode_ = mode;
    }
  }

  void setForwardMeasureValue(float distance)
  {
    forward_distance_ = distance;
  }
  //设置云台角度
  void setGimbalJointAngPitchFdb(float pitch) { gimbal_joint_ang_pitch_fdb_ = pitch; }
  void setGimbalJointAngPitchRef(float pitch) { gimbal_joint_ang_pitch_ref_ = pitch; }
  void setGimbalJointAngYawFdb(float yaw) { gimbal_joint_ang_yaw_fdb_ = yaw; }
  void setGimbalJointAngYawRef(float yaw) { gimbal_joint_ang_yaw_ref_ = yaw; }
  //设置发射机构相关
  void setHeat(float heat) { heat_ = heat; }
  void setHeatLimit(float limit) { heat_limit_ = limit; }
  void setFeedStuckFlag(bool flag)
  {
    if (flag != feed_stuck_flag_) {
      last_feed_stuck_flag_ = feed_stuck_flag_;
      feed_stuck_flag_ = flag;
    }
  }
  void setFricStuckFlag(bool flag)
  {
    if (flag != fric_stuck_flag_) {
      last_fric_stuck_flag_ = fric_stuck_flag_;
      fric_stuck_flag_ = flag;
    }
  }
  void setBulletNum(uint16_t num) { bullet_num_ = num; }
  //基地受击标识
  void setBaseAttack(bool flag) { is_base_attack_ = flag; }
  //超电显示
  void setCapPwrPercent(float percent) { cap_pwr_percent_ = percent; }
  //缓冲电容显示
  void setBufferCapPwrPercent(float percent) { buffer_cap_pwr_percent_ = percent; }
  //巡航标志
  void setNavigateFlag(bool state) { is_navigating_ = state; }
  //装甲板受击
  void setHurtModuleid(uint8_t id){hurt_module_id_ = id; }
  void setisArmorHit(bool is_armor_hit){is_armor_hit_ = is_armor_hit; }  
  //正向还是斜向
  void setIsStraight(bool is_straight){is_straight_ = is_straight; }
  //自瞄目标
  void setVisTgtX(uint16_t x, bool valid) { vis_tgt_x_ = valid ? x : -1; }
  void setVisTgtY(uint16_t y, bool valid) { vis_tgt_y_ = valid ? y : -1; }
  void setisvisionvalid(bool isvisionvalid) { is_vision_valid_ = isvisionvalid; }
#pragma endregion
 private:
 #pragma region UI图形注册与更新函数定义

  //设置静态 UI
  void setStaticStrings();
  void setStaticGraphics();

  //注册Graphic
  void addGraphics(UiMgr *ui_mgr_ptr);

  //更新动态 UI
  void updateDynamicStrings()
  {
    // updateBaseAttacted();
    updateOurBaseHitContent();
    updateGimbalWorkStateContent();
    updateChassisWorkStateContent();
  }
  void updateGimbalWorkStateContent();
  void updateChassisWorkStateContent();
    // updateBaseAttacted();
  void  updateOurBaseHitContent();
  void updateVisTgtCir(hello_world::referee::Circle &g);
  void updateShooterHeat(hello_world::referee::Arc &g);
  void updateRobotsHp(hello_world::referee::Integer &g_num_red_1, hello_world::referee::Integer &g_num_red_2, hello_world::referee::Integer &g_num_red_3,
                      hello_world::referee::Integer &g_num_red_4, hello_world::referee::Integer &g_num_red_7, hello_world::referee::Integer &g_num_blue_1,
                      hello_world::referee::Integer &g_num_blue_2, hello_world::referee::Integer &g_num_blue_3, hello_world::referee::Integer &g_num_blue_4,
                      hello_world::referee::Integer &g_num_blue_7);
  void updateCapPwrPercent(hello_world::referee::Rectangle& g_rect, hello_world::referee::FloatingNumber& g_num);
  void updateBufferCapPwrPercent(hello_world::referee::Rectangle& g_rect, hello_world::referee::FloatingNumber& g_num);
  void updateVisionbox(hello_world::referee::Rectangle& g_rect);
  void updateBulletNum(hello_world::referee::FloatingNumber &g);
  void updateChassisStatus(hello_world::referee::Arc& g_head, hello_world::referee::Arc& g_other);
  void updateChassisPassLineLeft(hello_world::referee::StraightLine& g);
  void updateChassisPassLineRight(hello_world::referee::StraightLine& g);
  void updateChassisDirection(hello_world::referee::StraightLine& g);
  void updateChassisJumpLine(hello_world::referee::StraightLine& g);
  void updateChassisForwardDistance(hello_world::referee::FloatingNumber& g);
  void updateArmorHit(hello_world::referee::Arc &g_hit) ;

#pragma endregion


#pragma region 图形定义
    //chassis
  Arc arc_head_;
  Arc arc_other_;
  StraightLine sline_pass_line_left_, sline_pass_line_right_;
  StraightLine sline_chassis_direction_;
  StraightLine sline_jump_line_;
  Rectangle rec_cap_pwr_percent_;
  FloatingNumber f_cap_pwr_percent_num_;
  Rectangle rec_buffer_cap_pwr_percent_;
  FloatingNumber f_buffer_cap_pwr_percent_num_;
    //vision
  Rectangle rec_vision_box_;
  Circle cir_vis_tgt_;
    //shooter
  Arc arc_heat_;
  FloatingNumber f_bullet_num_;
  FloatingNumber f_forward_distance_;
    //referee
  Arc arc_amor_;
  Integer int_red_1_hp_, int_red_2_hp_, int_red_3_hp_, int_red_4_hp_, int_red_7_hp_, int_blue_1_hp_, int_blue_2_hp_, int_blue_3_hp_,
      int_blue_4_hp_, int_blue_7_hp_;
//工作状态
  String str_gimbal_workstate_title_;
  String str_gimbal_workstate_content_;
  String str_chassis_workstate_title_;
  String str_chassis_workstate_content_;
  // String str_base_attacted_flag_;
  String str_our_base_hit_content_;                   ///< 我方基地受击提示
#pragma endregion 图形定义

#pragma region 变量定义
  RobotId sender_id_ = RobotId::kBlueStandard3;
  //var for chassis
  FsmWorkState chassis_work_state_ = FsmWorkState::kDead;
  FsmWorkState last_chassis_work_state_ = FsmWorkState::kDead;
  FsmCtrlMode chassis_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmCtrlMode last_chassis_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmManualCtrlSrc chassis_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  FsmManualCtrlSrc last_chassis_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  ChassisWorkingMode chassis_working_mode_ = ChassisWorkingMode::kDepart;
  ChassisWorkingMode last_chassis_working_mode_ = ChassisWorkingMode::kDepart;
  float theta_i2r_ = 0.0f;

  // var for gimbal
  FsmWorkState gimbal_work_state_ = FsmWorkState::kDead;
  FsmWorkState last_gimbal_work_state_ = FsmWorkState::kDead;
  FsmCtrlMode gimbal_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmCtrlMode last_gimbal_ctrl_mode_ = FsmCtrlMode::kManual;
  FsmManualCtrlSrc gimbal_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  FsmManualCtrlSrc last_gimbal_manual_ctrl_src_ = FsmManualCtrlSrc::kRc;
  GimbalWorkingMode gimbal_working_mode_ = GimbalWorkingMode::kManual;
  GimbalWorkingMode last_gimbal_working_mode_ = GimbalWorkingMode::kManual;
  float gimbal_joint_ang_pitch_fdb_ = 0.0f, gimbal_joint_ang_pitch_ref_ = 0.0f;
  float gimbal_joint_ang_yaw_fdb_ = 0.0f, gimbal_joint_ang_yaw_ref_ = 0.0f;

  uint16_t robots_max_hp_[RobotsNum] = {0};  ///< 各个机器人有记录的最大血量，单位为 HP
  uint16_t robots_hp_[RobotsNum] = {0};      ///< 各个机器人的当前血量，单位为 HP
  bool is_base_attack_ = false;  ///< 基地受攻击标志位
  bool last_base_attack_ = false;  ///< 上一次基地受攻击标志位
  //受打击装甲板
  uint8_t hurt_module_id_ = 0;  ///< 受打击的装甲板ID，0表示未受打击
  uint8_t last_hurt_module_id_ = 0;  ///< 上一次受打击的装甲板ID，用于判断是否变化
  float last_armor_angle_hit_ = 0.0f;
  bool is_armor_hit_ = 0;  ///< 是否有装甲板被打击
  bool is_straight_ = 0;
  // var for shooter
  bool feed_stuck_flag_ = false, last_feed_stuck_flag_ = false;
  bool fric_stuck_flag_ = false, last_fric_stuck_flag_ = false;
  float heat_ = 0;
  float heat_limit_ = 100;
  float bullet_num_ = 0;

  //var for navigation
  bool is_navigating_ = false; //巡航模式

  // var for super capacitor
  float cap_pwr_percent_ = 0;
  float buffer_cap_pwr_percent_ = 0;

  float forward_distance_ = 0.0f;

  // var for vision
  int16_t vis_tgt_x_ = 0;  ///< 视觉瞄准目标的 x 坐标，单位为像素，负数为无效值
  int16_t vis_tgt_y_ = 0;  ///< 视觉瞄准目标的 y 坐标，单位为像素，负数为无效值
  bool is_vision_valid_ = false;//视觉是否瞄到
#pragma endregion 变量定义
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero

#endif /* UI_DRAWER_HPP_ */
