/** 
 *******************************************************************************
 * @file      :ui_drawer.cpp
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
/* Includes ------------------------------------------------------------------*/
#include <string>

#include "ui_drawer.hpp"

// typedef hello_world::referee hello_world::referee ;
/* Private macro -------------------------------------------------------------*/

namespace robot
{
/* Private constants ---------------------------------------------------------*/

#pragma region names of graphics
//动态或者静止
const hello_world::referee::GraphicLayer kStaticUiLayer = hello_world::referee::GraphicLayer::k0;
const hello_world::referee::GraphicLayer kDynamicUiLayer = hello_world::referee::GraphicLayer::k1;
//颜色
const GraphicColor kUiNormalColor = GraphicColor::kGreen;
const GraphicColor kUiWarningColor = GraphicColor::kOrange;
const GraphicColor kUiErrorColor = GraphicColor::kPurple;
const GraphicColor kUiStringTitleColor = GraphicColor::kYellow;
const GraphicColor kUiVisTgtColor = GraphicColor::kGreen;
const GraphicColor kUiVisBoxColor = GraphicColor::kWhite;

// 各模块模式状态 左上角
const Pixel kUiModuleStateFontSize = 16;
const Pixel kUiModuleStateLineWidth = 3;
//血量UI显示
const Pixel kUiRobotsHpFontSize = 20;
const Pixel kUiRobotsHpLineWidth = 4;

// 布局、尺寸
const uint16_t kUiScreenMiddleX = 1920 / 2;  // 操作界面中心位置
const uint16_t kUiScreenMiddleY = 1080 / 2;  // 操作界面中心位置

const uint16_t kUiRedHpAreaX1 = kUiScreenMiddleX - 14 * kUiRobotsHpFontSize;
const uint16_t kUiRedHpAreaX2 = kUiRedHpAreaX1 - 6 * kUiRobotsHpFontSize;
const uint16_t kUiRedHpAreaX3 = kUiRedHpAreaX2 - 6 * kUiRobotsHpFontSize;
const uint16_t kUiRedHpAreaX4 = kUiRedHpAreaX3 - 6 * kUiRobotsHpFontSize;
const uint16_t kUiRedHpAreaX5 = kUiRedHpAreaX4 - 6 * 2 * kUiRobotsHpFontSize;
const uint16_t kUiBlueHpAreaX1 = kUiScreenMiddleX + 11 * kUiRobotsHpFontSize;
const uint16_t kUiBlueHpAreaX2 = kUiBlueHpAreaX1 + 6 * kUiRobotsHpFontSize;
const uint16_t kUiBlueHpAreaX3 = kUiBlueHpAreaX2 + 6 * kUiRobotsHpFontSize;
const uint16_t kUiBlueHpAreaX4 = kUiBlueHpAreaX3 + 6 * kUiRobotsHpFontSize;
const uint16_t kUiBlueHpAreaX5 = kUiBlueHpAreaX4 + 6 * 2 * kUiRobotsHpFontSize;
const uint16_t kUiHpAreaY1 = 870;

// 各模块状态 左上角
// chassis
const uint16_t kUiChassisDirCircleX = 960;//灯条中心位置
const uint16_t kUiChassisDirCircleY = 540;

const uint16_t kPixelCenterXCapBox = 1920 / 2;  //超电位置
const uint16_t kPixelCenterYCapBox = 120;
const uint16_t kPixelCapBoxWidth = 400; //超电能量余量外框
const uint16_t kPixelCapBoxHeight = 10;

const uint16_t kPixelCenterXBufferCapBox = 1750;  //缓冲电位置
const uint16_t kPixelCenterYBufferCapBox = 1080/2 + 100;
const uint16_t kPixelBufferCapBoxWidth = 10; //超电能量余量外框
const uint16_t kPixelBufferCapBoxHeight = 400;

const uint16_t kUiWorkStateAreaX1 = 100;
const uint16_t kUiWorkStateAreaX2_1 = kUiWorkStateAreaX1 + 125;
const uint16_t kUiWorkStateAreaY1 = 800;
const int16_t kUiWorkStateAreaYDelta = -35;

// 视觉框位置
const uint16_t kPixelCenterXVisionBox = 938;  //云台视觉状态位置
const uint16_t kPixelCenterYVisionBox =440.3;
const uint16_t kPixelVisionBoxWidth = 446; //状态外框 云台视觉
const uint16_t kPixelVisionBoxHeight = 200.3;

#pragma endregion names of graphics

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

#pragma region 初始化
void UiDrawer::setStaticStrings()
{
  std::string str;
  // chassis
  str = "Chassis:";
  str_chassis_workstate_title_ = 
      String(kUiWorkStateAreaX1, kUiWorkStateAreaY1 , kUiModuleStateFontSize, str, kUiStringTitleColor, kUiModuleStateLineWidth, kStaticUiLayer);
  //gimbal
  str = "Gimbal:";
  str_gimbal_workstate_title_ =
      String(kUiWorkStateAreaX1, kUiWorkStateAreaY1 + 40, kUiModuleStateFontSize, str, kUiStringTitleColor, kUiModuleStateLineWidth, kStaticUiLayer);
}
void UiDrawer::setStaticGraphics()
{

};
void UiDrawer::addGraphics(UiMgr *ui_mgr_ptr)
{
  HW_ASSERT(ui_mgr_ptr != nullptr, "UiMgr pointer cannot be null.");
  ui_mgr_ptr->addGraphic(&str_gimbal_workstate_title_);
  ui_mgr_ptr->addGraphic(&str_gimbal_workstate_content_);
  ui_mgr_ptr->addGraphic(&str_chassis_workstate_title_);
  ui_mgr_ptr->addGraphic(&str_chassis_workstate_content_);
  ui_mgr_ptr->addGraphic(&rec_vision_box_);
  ui_mgr_ptr->addGraphic(&cir_vis_tgt_);
  ui_mgr_ptr->addGraphic(&arc_heat_);
  ui_mgr_ptr->addGraphic(&f_bullet_num_);
  ui_mgr_ptr->addGraphic(&f_cap_pwr_percent_num_);
  ui_mgr_ptr->addGraphic(&rec_cap_pwr_percent_);
  ui_mgr_ptr->addGraphic(&rec_buffer_cap_pwr_percent_);
  ui_mgr_ptr->addGraphic(&f_buffer_cap_pwr_percent_num_);
  ui_mgr_ptr->addGraphic(&f_forward_distance_);
  ui_mgr_ptr->addGraphic(&arc_amor_);
  ui_mgr_ptr->addGraphic(&arc_head_);
  ui_mgr_ptr->addGraphic(&arc_other_);
  ui_mgr_ptr->addGraphic(&sline_pass_line_left_);
  ui_mgr_ptr->addGraphic(&sline_pass_line_right_);
  ui_mgr_ptr->addGraphic(&sline_chassis_direction_);
  ui_mgr_ptr->addGraphic(&sline_jump_line_);
  // ui_mgr_ptr->addGraphic(&int_red_1_hp_);
  // ui_mgr_ptr->addGraphic(&int_red_2_hp_);
  // ui_mgr_ptr->addGraphic(&int_red_3_hp_);
  // ui_mgr_ptr->addGraphic(&int_red_4_hp_);
  // ui_mgr_ptr->addGraphic(&int_red_7_hp_);
  // ui_mgr_ptr->addGraphic(&int_blue_1_hp_);
  // ui_mgr_ptr->addGraphic(&int_blue_2_hp_);
  // ui_mgr_ptr->addGraphic(&int_blue_3_hp_);
  // ui_mgr_ptr->addGraphic(&int_blue_4_hp_);
  // ui_mgr_ptr->addGraphic(&int_blue_7_hp_);
  // ui_mgr_ptr->addGraphic(&str_our_base_hit_content_);
};
#pragma endregion

#pragma region UI 组
// void UiDrawer::updateBaseAttacted()
// {
//   std::string str;
//   hello_world::referee::Pixel linewidth = 6;
//   if (is_base_attack_==true)
//   {
//     linewidth = 6;
//   }
//   else
//   {
//     linewidth = 0;
//   }

//   str = "BASE!!";
//   str_base_attacted_flag_ = 
//       String(kUiScreenMiddleX, 800, kUiModuleStateFontSize, str, hello_world::referee::GraphicColor::kPurple, linewidth, kStaticUiLayer);
// }
void UiDrawer::updateOurBaseHitContent()
{
  static bool is_string_inited = false;
  static uint16_t show_tick = 0;
  std::string str = "BASE!";
  GraphicColor color = kUiErrorColor;

  if (!is_string_inited) {
    str_our_base_hit_content_ = String(880, 800, 40, str, color, 6, kDynamicUiLayer);
    str_our_base_hit_content_.setVisibility(false);
    is_string_inited = true;
  } else {
    if (is_base_attack_) {
      show_tick += 750;
      if (show_tick > 1500) {
        show_tick = 1500;  // 限制最大显示时间
      }
    }
    if (show_tick > 0) {
      str_our_base_hit_content_.setVisibility(true);
      show_tick--;
    } else {
      str_our_base_hit_content_.setVisibility(false);
    }
  }
}

void UiDrawer::updateVisionbox(hello_world::referee::Rectangle& g_rect)
{
  if (is_vision_valid_) {
    g_rect.setColor(hello_world::referee::String::Color::kPurple);
  }
  else {
    g_rect.setColor(hello_world::referee::String::Color::kWhite);
  }

  g_rect.setStartPos(655, 250);
  g_rect.setEndPos(1269, 651);
  g_rect.setLayer(kStaticUiLayer);
  g_rect.setLineWidth(1.5);
};

void UiDrawer::updateChassisStatus(hello_world::referee::Arc& g_head, hello_world::referee::Arc& g_other)
{
  float now_head_ang = -theta_i2r_ * 180 / M_PI;
  float start_ang_head = now_head_ang - 40, end_ang_head = now_head_ang + 40;
  start_ang_head = hello_world::NormPeriodData(0, 360, start_ang_head);
  end_ang_head = hello_world::NormPeriodData(0, 360, end_ang_head);

  float start_ang_other = start_ang_head + 180, end_ang_other = end_ang_head + 180;
  start_ang_other = hello_world::NormPeriodData(0, 360, start_ang_other);
  end_ang_other = hello_world::NormPeriodData(0, 360, end_ang_other);

  float radius = 50;//灯条所处圆的半径
  g_head.setColor(hello_world::referee::Arc::Color::kYellow);
  g_head.setAng(start_ang_head, end_ang_head);
  g_head.setCenterPos(kUiChassisDirCircleX, kUiChassisDirCircleY);//灯条中心位置
  g_head.setRadius(radius, radius);
  g_head.setLineWidth(3);
  g_head.setLayer(kDynamicUiLayer);

  g_other.setColor(hello_world::referee::Arc::Color::kCyan);
  g_other.setAng(start_ang_other, end_ang_other);
  g_other.setCenterPos(kUiChassisDirCircleX, kUiChassisDirCircleY);
  g_other.setRadius(radius, radius);
  g_other.setLineWidth(3);
  g_head.setLayer(kDynamicUiLayer);
};
void UiDrawer::updateBulletNum(hello_world::referee::FloatingNumber &g)
{
  g.setDisplayValue(bullet_num_);
  g.setStartPos(1360, 690);
  g.setColor(hello_world::referee::FloatingNumber::Color::kPurple);
  g.setFontSize(30);
  g.setLineWidth(kUiModuleStateLineWidth);
  g.setLayer(kDynamicUiLayer);
};
float debug_c = 0.0f;
void UiDrawer::updateCapPwrPercent(hello_world::referee::Rectangle &g_rect, hello_world::referee::FloatingNumber &g_num)
{
  debug_c = cap_pwr_percent_;
  uint16_t start_x = kPixelCenterXCapBox - kPixelCapBoxWidth / 2;
  float percent = hello_world::Bound(cap_pwr_percent_/100.0f, 0, 1);
  uint16_t end_x = start_x + kPixelCapBoxWidth * percent;

  hello_world::referee::GraphicColor color = hello_world::referee::Rectangle::Color::kGreen;

  if (percent > 0.8) {
  } else if (percent > 0.6) {
    color = hello_world::referee::Rectangle::Color::kYellow;
  } else if (percent > 0.4) {
    color = hello_world::referee::Rectangle::Color::kOrange;
  } else {
    color = hello_world::referee::Rectangle::Color::kPurple;
  }

  g_rect.setStartPos(start_x, kPixelCenterYCapBox - kPixelCapBoxHeight / 2);
  g_rect.setEndPos(end_x, kPixelCenterYCapBox + kPixelCapBoxHeight / 2);
  g_rect.setColor(color);
  g_rect.setLineWidth(kPixelCapBoxHeight * 2);
  g_rect.setLayer(kDynamicUiLayer);
  
  g_num.setLayer(kDynamicUiLayer);
  g_num.setDisplayValue(percent * 100);
  g_num.setStartPos(start_x - 100, kPixelCenterYCapBox);
  g_num.setColor(color);
  g_num.setFontSize(20);//调整超电剩余电量的数字大小
  g_num.setLineWidth(kUiModuleStateLineWidth);
};

void UiDrawer::updateBufferCapPwrPercent(hello_world::referee::Rectangle &g_rect, hello_world::referee::FloatingNumber &g_num)
{
  uint16_t start_y = kPixelCenterYBufferCapBox - kPixelBufferCapBoxHeight / 2;
  float percent = hello_world::Bound(buffer_cap_pwr_percent_/100.0f, 0, 1);
  uint16_t end_y = start_y + kPixelBufferCapBoxHeight * percent;

  hello_world::referee::GraphicColor color = hello_world::referee::Rectangle::Color::kGreen;

  if (percent > 0.8) {
  } else if (percent > 0.6) {
    color = hello_world::referee::Rectangle::Color::kYellow;
  } else if (percent > 0.4) {
    color = hello_world::referee::Rectangle::Color::kOrange;
  } else {
    color = hello_world::referee::Rectangle::Color::kPurple;
  }

  g_rect.setStartPos(kPixelCenterXBufferCapBox - kPixelBufferCapBoxWidth / 2, start_y);
  g_rect.setEndPos(kPixelCenterXBufferCapBox + kPixelBufferCapBoxWidth / 2, end_y);
  g_rect.setColor(color);
  g_rect.setLineWidth(kPixelBufferCapBoxWidth * 2);
  g_rect.setLayer(kDynamicUiLayer);
  
  g_num.setLayer(kDynamicUiLayer);
  g_num.setDisplayValue(percent * 100);
  g_num.setStartPos(kPixelCenterXBufferCapBox, start_y-50);
  g_num.setColor(color);
  g_num.setFontSize(20);//调整缓冲电量剩余电量的数字大小
  g_num.setLineWidth(kUiModuleStateLineWidth);
};


void UiDrawer::updateChassisPassLineLeft(hello_world::referee::StraightLine& g)
{
  uint16_t end_posX = 0;
  uint16_t start_posX = 0;
  uint16_t end_posY = 0;
  end_posX = gimbal_joint_ang_pitch_fdb_ * 7.242 + 828.896;
  start_posX = gimbal_joint_ang_pitch_fdb_ * 480.26 + 628.58;
  end_posY = gimbal_joint_ang_pitch_fdb_ * -770.76 + 321.96;
  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, 0);
  g.setEndPos(end_posX, end_posY);
  g.setColor(hello_world::referee::Graphic::Color::kOrange);
  g.setLineWidth(3);
};
void UiDrawer::updateChassisPassLineRight(hello_world::referee::StraightLine& g)
{
  uint16_t end_posX = 0;
  uint16_t start_posX = 0;
  uint16_t end_posY = 0;
  start_posX = gimbal_joint_ang_pitch_fdb_ * -636.9189 + 1248.9175;
  end_posX = gimbal_joint_ang_pitch_fdb_ * 15.465 + 1067.601;
  end_posY = gimbal_joint_ang_pitch_fdb_ * -770.76 + 321.96;
  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, 0);
  g.setEndPos(end_posX, end_posY);
  g.setColor(hello_world::referee::Graphic::Color::kOrange);
  g.setLineWidth(3);
};
void UiDrawer::updateChassisDirection(hello_world::referee::StraightLine& g)
{
  uint16_t start_posX = kUiWorkStateAreaX1+10;
  uint16_t start_posY = kUiWorkStateAreaY1-100;
  uint16_t end_posX = 0;
  uint16_t end_posY = 0;

  if (!is_straight_) {
    end_posX = start_posX;
    end_posY = start_posY + 60;
  } else {
    end_posX = start_posX + 70;
    end_posY = start_posY + 60;
  }

  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, start_posY);
  g.setEndPos(end_posX, end_posY);
  g.setColor(hello_world::referee::Graphic::Color::kOrange);
  g.setLineWidth(3);
};

void UiDrawer::updateChassisJumpLine(hello_world::referee::StraightLine& g)
{
  uint16_t start_posX = 700;
  uint16_t start_posY = gimbal_joint_ang_pitch_fdb_ * -814.36 + 326.21;
  uint16_t end_posX = 1200;
  uint16_t end_posY = gimbal_joint_ang_pitch_fdb_ * -814.36 + 326.21;

  g.setLayer(kDynamicUiLayer);
  g.setStartPos(start_posX, start_posY);
  g.setEndPos(end_posX, end_posY);
  g.setColor(hello_world::referee::Graphic::Color::kCyan);
  g.setLineWidth(3);
};

void UiDrawer::updateArmorHit(hello_world::referee::Arc &g_hit) {
  const float kHitArcAngleRangeGyro = 65.0f;
  const float kHitArcAngleBiasGyro = 7.0f;
  const float kHitArcAngleRangeFollow = 60.0f;

  if (is_armor_hit_) {
    g_hit.setLineWidth(4);
  } else {
    g_hit.setLineWidth(0);
  }

  // 跟随模式前向对应的装甲板应设置为1号，其余装甲板序号由裁判系统要求，逆时针递增
  float armor_angle_hit = 0.0f, arc_angle_hit_start = 0.0f,
        arc_angle_hit_end = 0.0f;
  if (chassis_working_mode_ == ChassisWorkingMode::kFastGyro) {
    if (hurt_module_id_ == last_hurt_module_id_) {
      armor_angle_hit = last_armor_angle_hit_;
    }
    arc_angle_hit_start =
        armor_angle_hit - kHitArcAngleRangeGyro / 2.0f + kHitArcAngleBiasGyro,
    arc_angle_hit_end =
        armor_angle_hit + kHitArcAngleRangeGyro / 2.0f + kHitArcAngleBiasGyro;
  } else {
    armor_angle_hit = -(theta_i2r_ * 180.0f / M_PI + 90.0f * hurt_module_id_);
    arc_angle_hit_start = armor_angle_hit - kHitArcAngleRangeFollow / 2.0f;
    arc_angle_hit_end = armor_angle_hit + kHitArcAngleRangeFollow / 2.0f;
  }

  arc_angle_hit_start =
      hello_world::NormPeriodData(0.0f, 360.0f, arc_angle_hit_start);
  arc_angle_hit_end =
      hello_world::NormPeriodData(0.0f, 360.0f, arc_angle_hit_end);

  float radius = 350.0f; // 所处圆的半径
  g_hit.setColor(hello_world::referee::Graphic::Color::kPurple);
  g_hit.setAng(arc_angle_hit_start, arc_angle_hit_end);
  g_hit.setCenterPos(960,540); // 视觉框中心位置
  g_hit.setRadius(radius, radius);
  g_hit.setLayer(kDynamicUiLayer);
  last_hurt_module_id_ = hurt_module_id_;
  last_armor_angle_hit_ = armor_angle_hit;
};

void UiDrawer::updateChassisForwardDistance(hello_world::referee::FloatingNumber &g)
  {
    g.setDisplayValue(forward_distance_ * 100);
    g.setStartPos(100, 700);
    g.setColor(hello_world::referee::FloatingNumber::Color::kPurple);
    g.setFontSize(30);
    g.setLineWidth(kUiModuleStateLineWidth);
    g.setLayer(kDynamicUiLayer);
  };

#pragma endregion
#pragma region 发射机构相关 UI
void UiDrawer::updateShooterHeat(hello_world::referee::Arc &g)
{
  float percent = heat_limit_ > 0 ? heat_ / heat_limit_ : 0.0f;
  percent = hello_world::Bound(percent, 0.0f, 1.0f);
  g.setCenterPos(1920 / 2, 1080 / 2);
  hello_world::referee::GraphicColor color = hello_world::referee::Arc::Color::kGreen;
  if (percent > 0.8) {
    color = hello_world::referee::Arc::Color::kPurple;
  } else if (percent > 0.6) {
    color = hello_world::referee::Arc::Color::kOrange;
  } else if (percent > 0.3) {
    color = hello_world::referee::Arc::Color::kYellow;
  }
  g.setRadius(100, 100);
  g.setAng(360 - 360 * percent, 0);
  g.setColor(color);
  g.setLayer(kDynamicUiLayer);
  g.setLineWidth(percent == 0 ? 0 : 2);
};

void UiDrawer::updateGimbalWorkStateContent()
{
  static bool is_string_inited = false;
  std::string str = "Unknown";
  GraphicColor color = kUiNormalColor;

  // gimbal
  if (gimbal_work_state_ != robot::PwrState::kWorking) {
    color = kUiWarningColor;
    str = PwrStateToStr(gimbal_work_state_);
  } else {
    str = robot::GimbalJointWorkingModeToStr(gimbal_working_mode_) + "-" + hello_world::module::CtrlModeSrcToStr(gimbal_ctrl_mode_, gimbal_manual_ctrl_src_);
  }

  if (!is_string_inited) {
    str_gimbal_workstate_content_ =
        String(kUiWorkStateAreaX2_1, kUiWorkStateAreaY1+40, kUiModuleStateFontSize, str, color, kUiModuleStateLineWidth, kDynamicUiLayer);
    is_string_inited = true;
  } else {
    str_gimbal_workstate_content_.setString(str);
    str_gimbal_workstate_content_.setColor(color);
  }
}
void UiDrawer::updateChassisWorkStateContent()
{
  static bool is_string_inited = false;
  std::string str = "Unknown";
  GraphicColor color = kUiNormalColor;

  // gimbal
  if (chassis_work_state_ != robot::PwrState::kWorking) {
    color = kUiWarningColor;
    str = PwrStateToStr(chassis_work_state_);
  } else {
    str = robot::ChassisWorkingModeToStr(chassis_working_mode_) + "-" + hello_world::module::CtrlModeSrcToStr(chassis_ctrl_mode_, chassis_manual_ctrl_src_);
  }

  if (!is_string_inited) {
    str_chassis_workstate_content_ =
        String(kUiWorkStateAreaX2_1, kUiWorkStateAreaY1, kUiModuleStateFontSize, str, color, kUiModuleStateLineWidth, kDynamicUiLayer);
    is_string_inited = true;
  } else {
    str_chassis_workstate_content_.setString(str);
    str_chassis_workstate_content_.setColor(color);
  }
}

void UiDrawer::updateVisTgtCir(hello_world::referee::Circle &g)
{
  if (is_vision_valid_) {
    g.setLineWidth(3);
  } else {
    g.setLineWidth(0);
  }

  g.setCenterPos(vis_tgt_x_ , 1080 - vis_tgt_y_ );
  g.setRadius(20);
  g.setColor(kUiNormalColor);
  g.setLayer(kDynamicUiLayer);
};

// void UiDrawer::updateRobotsHp(hello_world::referee::Integer &g_num_red_1, hello_world::referee::Integer &g_num_red_2,
//                               hello_world::referee::Integer &g_num_red_3, hello_world::referee::Integer &g_num_red_4,
//                               hello_world::referee::Integer &g_num_red_7, hello_world::referee::Integer &g_num_blue_1,
//                               hello_world::referee::Integer &g_num_blue_2, hello_world::referee::Integer &g_num_blue_3,
//                               hello_world::referee::Integer &g_num_blue_4, hello_world::referee::Integer &g_num_blue_7)
// {
//   GraphicColor color[RobotsId::RobotsNum] = {kUiNormalColor, kUiNormalColor, kUiNormalColor, kUiNormalColor, kUiNormalColor,
//                                              kUiNormalColor, kUiNormalColor, kUiNormalColor, kUiNormalColor, kUiNormalColor};
//   for (size_t i = 0; i < RobotsId::RobotsNum; ++i) {
//     if (robots_hp_[i] < 0.3f * robots_max_hp_[i] || robots_max_hp_[i] == 0) {  // 血量小于30%时显示为紫色
//       color[i] = kUiErrorColor;
//     } else if (robots_hp_[i] < 0.7f * robots_max_hp_[i]) {  // 血量处于30%-70%时显示为橙色
//       color[i] = kUiWarningColor;
//     } else {  // 血量大于70%时显示为绿色
//       color[i] = kUiNormalColor;
//     }
//   }
//   // 红方
//   g_num_red_1.setDisplayValue(robots_hp_[RobotsId::RedHero]);
//   g_num_red_1.setStartPos(kUiRedHpAreaX1, kUiHpAreaY1);
//   g_num_red_1.setColor(color[RobotsId::RedHero]);
//   g_num_red_1.setLayer(kDynamicUiLayer);
//   g_num_red_1.setFontSize(kUiRobotsHpFontSize);
//   g_num_red_1.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_red_2.setDisplayValue(robots_hp_[RobotsId::RedEngineer]);
//   g_num_red_2.setStartPos(kUiRedHpAreaX2, kUiHpAreaY1);
//   g_num_red_2.setColor(color[RobotsId::RedEngineer]);
//   g_num_red_2.setLayer(kDynamicUiLayer);
//   g_num_red_2.setFontSize(kUiRobotsHpFontSize);
//   g_num_red_2.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_red_3.setDisplayValue(robots_hp_[RobotsId::RedStandard3]);
//   g_num_red_3.setStartPos(kUiRedHpAreaX3, kUiHpAreaY1);
//   g_num_red_3.setColor(color[RobotsId::RedStandard3]);
//   g_num_red_3.setLayer(kDynamicUiLayer);
//   g_num_red_3.setFontSize(kUiRobotsHpFontSize);
//   g_num_red_3.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_red_4.setDisplayValue(robots_hp_[RobotsId::RedStandard4]);
//   g_num_red_4.setStartPos(kUiRedHpAreaX4, kUiHpAreaY1);
//   g_num_red_4.setColor(color[RobotsId::RedStandard4]);
//   g_num_red_4.setLayer(kDynamicUiLayer);
//   g_num_red_4.setFontSize(kUiRobotsHpFontSize);
//   g_num_red_4.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_red_7.setDisplayValue(robots_hp_[RobotsId::RedSentry]);
//   g_num_red_7.setStartPos(kUiRedHpAreaX5, kUiHpAreaY1);
//   g_num_red_7.setColor(color[RobotsId::RedSentry]);
//   g_num_red_7.setLayer(kDynamicUiLayer);
//   g_num_red_7.setFontSize(kUiRobotsHpFontSize);
//   g_num_red_7.setLineWidth(kUiRobotsHpLineWidth);

//   // 蓝方
//   g_num_blue_1.setDisplayValue(robots_hp_[RobotsId::BlueHero]);
//   g_num_blue_1.setStartPos(kUiBlueHpAreaX1, kUiHpAreaY1);
//   g_num_blue_1.setColor(color[RobotsId::BlueHero]);
//   g_num_blue_1.setLayer(kDynamicUiLayer);
//   g_num_blue_1.setFontSize(kUiRobotsHpFontSize);
//   g_num_blue_1.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_blue_2.setDisplayValue(robots_hp_[RobotsId::BlueEngineer]);
//   g_num_blue_2.setStartPos(kUiBlueHpAreaX2, kUiHpAreaY1);
//   g_num_blue_2.setColor(color[RobotsId::BlueEngineer]);
//   g_num_blue_2.setLayer(kDynamicUiLayer);
//   g_num_blue_2.setFontSize(kUiRobotsHpFontSize);
//   g_num_blue_2.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_blue_3.setDisplayValue(robots_hp_[RobotsId::BlueStandard3]);
//   g_num_blue_3.setStartPos(kUiBlueHpAreaX3, kUiHpAreaY1);
//   g_num_blue_3.setColor(color[RobotsId::BlueStandard3]);
//   g_num_blue_3.setLayer(kDynamicUiLayer);
//   g_num_blue_3.setFontSize(kUiRobotsHpFontSize);
//   g_num_blue_3.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_blue_4.setDisplayValue(robots_hp_[RobotsId::BlueStandard4]);
//   g_num_blue_4.setStartPos(kUiBlueHpAreaX4, kUiHpAreaY1);
//   g_num_blue_4.setColor(color[RobotsId::BlueStandard4]);
//   g_num_blue_4.setLayer(kDynamicUiLayer);
//   g_num_blue_4.setFontSize(kUiRobotsHpFontSize);
//   g_num_blue_4.setLineWidth(kUiRobotsHpLineWidth);

//   g_num_blue_7.setDisplayValue(robots_hp_[RobotsId::BlueSentry]);
//   g_num_blue_7.setStartPos(kUiBlueHpAreaX5, kUiHpAreaY1);
//   g_num_blue_7.setColor(color[RobotsId::BlueSentry]);
//   g_num_blue_7.setLayer(kDynamicUiLayer);
//   g_num_blue_7.setFontSize(kUiRobotsHpFontSize);
//   g_num_blue_7.setLineWidth(kUiRobotsHpLineWidth);
// };

#pragma endregion
/* Private function definitions ----------------------------------------------*/

}  