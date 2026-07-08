#ifndef SENTRY_MODULES_ROBOT_HPP_
#define SENTRY_MODULES_ROBOT_HPP_

#include "DT7.hpp"
#include "buzzer.hpp"
#include "fdcan_tx_mgr.hpp"
// feed.hpp/fric.hpp before gimbal.hpp: ensures hello_world::module types are
// declared before module_state.hpp processes its using-declarations
#include "feed.hpp"
#include "fric.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "fsm.hpp"
#include "motor.hpp"
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"
#include "vision.hpp"
#include "super_cap.hpp"
#include "tick.hpp"
#include "transmitter.hpp"
#include "uart_rx_mgr.hpp"
#include "uart_tx_mgr.hpp"
#include "imu.hpp"
#include "rfr_default_data.hpp"
#include "ui_drawer.hpp"
#include "ui_mgr.hpp"
#include "stm32h723xx.h"

namespace robot
{
    class Robot : public Fsm
    {
    public:
        typedef hello_world::buzzer::Buzzer Buzzer;
        typedef hello_world::cap::SuperCap Cap;
        typedef hello_world::comm::Transmitter Transmitter;
        typedef hello_world::comm::FdCanTxMgr FdCanTxMgr;
        typedef hello_world::comm::UartTxMgr UartTxMgr;
        typedef hello_world::comm::TxMgr TxMgr;
        typedef hello_world::motor::Motor Motor;
        typedef hello_world::remote_control::DT7 DT7;
        typedef hello_world::remote_control::SwitchState RcSwitchState;
        typedef hello_world::imu::Imu Imu;

        typedef hello_world::referee::Referee Referee;
        typedef hello_world::referee::CompStatusPackage CompStatusPkg;
        typedef hello_world::referee::TeamEventPackage TeamEventPkg;
        typedef hello_world::referee::RobotPerformancePackage RobotPerformancePkg;
        typedef hello_world::referee::RobotPosPackage RobotPosPkg;
        typedef hello_world::referee::RobotResourcePackage RobotResourcePkg;
        typedef hello_world::referee::RobotRfidPackage RobotRfidPkg;
        typedef hello_world::referee::RobotsGroundPosPackage RobotsGroundPosPkg;
        typedef hello_world::referee::RobotSentryDecisionPackage RobotSentryDecisionPkg;
        typedef hello_world::referee::CompRobotsHpPackage CompRobotsHpPkg;
        typedef hello_world::referee::InterMapClientToRobotPackage InterMapClientToRobotPkg;
        typedef hello_world::referee::RobotPowerHeatPackage RobotPowerHeatPkg;
        typedef hello_world::referee::RobotShooterPackage RobotShooterPkg;
        typedef hello_world::referee::RobotHurtPackage RobotHurtPkg;
        typedef hello_world::referee::RobotBuffPackage RobotBuffPkg;
        typedef hello_world::referee::InterRadarDetectionPackage InterRadarDetectionPkg;
        typedef hello_world::referee::InterRobotTrajPackage InterRobotTrajPkg;
        
        typedef hello_world::referee::InterSentryCmd InterSentryCmd;
        typedef hello_world::referee::InterSentryDetectionPackage InterSentryDetectionPkg;

        typedef robot::Chassis Chassis;
        typedef robot::Gimbal Gimbal;
        typedef robot::UiDrawer UiDrawer;
        typedef hello_world::referee::UiMgr UiMgr;

        typedef hello_world::module::Feed Feed;
        typedef hello_world::module::Fric Fric;

        typedef hello_world::vision::Vision Vision;

        Robot() {};
        ~Robot() {};

        // 状态机主要接口函数
        void update() override;
        void run() override;
        void reset() override;
        void standby() override;

        // 注册函数
        void registerChassis(Chassis *ptr);
        void registerGimbal(Gimbal *ptr);
        void registerFeed(Feed *ptr);
        void registerFric(Fric *ptr);
        void registerBuzzer(Buzzer *ptr);
        void registerImu(Imu *ptr);
        void registerMotorWheels(Motor *dev_ptr, uint8_t idx);
        void registerMotorGimbal(Motor *dev_ptr, uint8_t idx);
        void registerMotorFeed(Motor *dev_ptr);
        void registerMotorFric(Motor *dev_ptr, uint8_t idx);
        void registerCap(Cap *dev_ptr);
        void registerVision(hello_world::vision::Vision *ptr);
        void registerUiMgr(UiMgr *ptr);
        void registerUiDrawer(UiDrawer *ptr);
        void registerRc(DT7 *ptr);
        void registerRfr(Referee *dev_ptr);

        void registerRfrCompStatusPkg(CompStatusPkg *ptr);
        void registerRfrTeamEventPkg(TeamEventPkg *ptr);
        void registerRfrRobotPerformancePkg(RobotPerformancePkg *ptr);
        void registerRfrRobotPosPkg(RobotPosPkg *ptr);
        void registerRfrRobotResourcePkg(RobotResourcePkg *ptr);
        void registerRfrRobotRfidPkg(RobotRfidPkg *ptr);
        void registerRfrRobotsGroundPosPkg(RobotsGroundPosPkg *ptr);
        void registerRfrRobotSentryDecisionPkg(RobotSentryDecisionPkg *ptr);
        void registerRfrCompRobotsHpPkg(CompRobotsHpPkg *ptr);
        void registerRfrInterMapClientToRobotPkg(InterMapClientToRobotPkg *ptr);
        void registerRfrRobotPowerHeatPkg(RobotPowerHeatPkg *ptr);
        void registerRfrRobotShooterPkg(RobotShooterPkg *ptr);
        void registerRfrRobotHurtPkg(RobotHurtPkg *ptr);
        void registerRfrInterSentryCmd(InterSentryCmd *ptr);
        void registerRfrRobotBuffPkg(RobotBuffPkg *ptr);
        void registerRfrInterSentryDetectionPkg(InterSentryDetectionPkg *ptr);
        void registerRfrInterRadarDetectionPkg(InterRadarDetectionPkg *ptr);
        void registerRfrInterRobotTrajPkg(InterRobotTrajPkg *ptr);

        void imuInitHardware() {imu_ptr_->initHardware();} ///< 初始化 IMU 硬件

    private:
        // 数据更新和工作状态更新，由update函数调用
        void updateData();
        void updateImuData();
        void updateRfrData();
        void updateRcData();
        void updateModuleState();
        void updatePwrState();

        // 各种工作状态任务执行函数
        void runOnDead();
        void runOnResurrection();
        void runOnWorking();

        void genModulesCmd();
        void genModulesCmdFromKb();

        //工具函数
        // 获取遥控器指令并设定Gimbal
        void GimbalRcCmd(Gimbal::Cmd &cmd);
        // 获取遥控器指令并设定Chassis
        void ChassisRcCmd(Chassis::Cmd &cmd, float v_max);

        // 设置通信组件数据函数
        void setCommData();
        void setUiDrawerData();

        // 重置数据函数
        void resetDataOnDead();
        void resetDataOnResurrection();

        // 发送通讯组件数据函数
        void sendCommData();
        void sendCanData();
        void sendWheelsMotorData();
        void sendGimbalPitchMotorData();
        void sendGimbalYawMotorData();
        void sendFeedMotorData();
        void sendFricMotorData();
        void sendCapData();
        void sendRfrData();
        void sendUsartData();
        void sendVisionData();

        // IMU数据在update函数中更新
        bool is_imu_caled_offset_ = false; ///< 是否已经校准零飘
                
        // RC数据在 update 函数中更新
        RcSwitchState now_SL_ = RcSwitchState::kErr;
        RcSwitchState now_SR_ = RcSwitchState::kErr;
        RcSwitchState last_SL_ = RcSwitchState::kErr;
        RcSwitchState last_SR_ = RcSwitchState::kErr;

        // 控制源：RC遥控器 或 KB键鼠
        hello_world::module::ManualCtrlSrc manual_ctrl_src_ = hello_world::module::ManualCtrlSrc::kRc;

        // 行进方向标志（true=正向，false=斜向）
        bool is_straight_ = true;
         
        // 调试模式下，根据遥控器指令生成
        bool is_chassis_standby_ = false; ///< 底盘是否处于待机状态
        bool is_gimbal_standby_ = false;  ///< 云台是否处于待机状态
        bool is_feed_standby_ = false;    ///< 拨弹盘是否处于待机状态
        bool is_fric_standby_ = false;    ///< 摩擦轮是否处于待机状态   

        bool manual_shoot_continuous_flag_ = false; ///< 调试模式下，连发模式的触发情况

        // 主要模块状态机组件指针
        Chassis *chassis_ptr_ = nullptr; ///< 底盘模块组件指针
        Gimbal *gimbal_ptr_ = nullptr;   ///< 云台模块组件指针
        Feed *feed_ptr_ = nullptr;       ///< 拨弹盘模块组件指针
        Fric *fric_ptr_ = nullptr;       ///< 摩擦轮模块组件指针

        // 无通信功能的组件指针
        Buzzer *buzzer_ptr_ = nullptr; ///< 蜂鸣器模块指针
        Imu *imu_ptr_ = nullptr;       ///< IMU模块指针

        // 只接收数据的组件指针
        DT7 *rc_ptr_ = nullptr; ///< 遥控器模块指针

        // 只发送数据的组件指针
        Cap *cap_ptr_ = nullptr;
        Motor *wheel_motor_ptr_[Chassis::kWheelMotorNum] = {nullptr};
        Motor *gimbal_motor_ptr_[Gimbal::kJointNum] = {nullptr};
        Motor *feed_motor_ptr_ = {nullptr};
        Motor *fric_motor_ptr_[2] = {nullptr};

        // 发送接收数据的组件指针
        Referee *rfr_ptr_ = nullptr;
        CompStatusPkg *rfr_comp_status_pkg_ptr_ = nullptr;
        bool rfr_csp_send_flag_ = false;
        TeamEventPkg *rfr_team_event_pkg_ptr_ = nullptr;
        bool rfr_tep_send_flag_ = false;
        RobotPerformancePkg *rfr_robot_performance_pkg_ptr_ = nullptr;
        bool rfr_rpp_send_flag_ = false;
        RobotPosPkg *rfr_robot_pos_pkg_ptr_ = nullptr;
        bool rfr_rpos_send_flag_ = false;
        RobotResourcePkg *rfr_robot_resource_pkg_ptr_ = nullptr;
        bool rfr_rrp_send_flag_ = false;
        RobotRfidPkg *rfr_robot_rfid_pkg_ptr_ = nullptr;
        bool rfr_rrfid_send_flag_ = false;
        RobotsGroundPosPkg *rfr_robots_ground_pos_pkg_ptr_ = nullptr;
        bool rfr_rgp_send_flag_ = false;
        RobotSentryDecisionPkg *rfr_robot_sentry_decision_pkg_ptr_ = nullptr;
        bool rfr_rsd_send_flag_ = false;
        CompRobotsHpPkg *rfr_comp_robots_hp_pkg_ptr_ = nullptr;
        bool rfr_crh_send_flag_ = false;
        InterMapClientToRobotPkg *rfr_inter_map_client_to_robot_pkg_ptr_ = nullptr;
        bool rfr_imctr_send_flag_ = false;
        RobotHurtPkg *rfr_robot_hurt_pkg_ptr_ = nullptr;
        bool rfr_rh_send_flag_ = false;
        RobotBuffPkg  *rfr_robot_buff_pkg_ptr_ = nullptr;
        bool rfr_rb_send_flag_ = false;

        InterRadarDetectionPkg *rfr_inter_radar_detection_pkg_ptr_ = nullptr;
        bool rfr_irsc_send_flag_ = false;
        
        InterSentryDetectionPkg *rfr_inter_sentry_detection_pkg_ptr_ = nullptr;
        InterRobotTrajPkg *rfr_inter_robot_traj_pkg_ptr_ = nullptr;

        float shoot_speed = 23.5;
        float last_shoot_speed = 23.5;
        
        RobotPowerHeatPkg *rfr_robot_power_heat_pkg_ptr_ = nullptr;
        RobotShooterPkg *rfr_robot_shooter_pkg_ptr_ = nullptr;
        InterSentryCmd *rfr_inter_sentry_cmd_ptr_ = nullptr;
        uint16_t robot_id_ = kDefaultRobotPerformanceData.robot_id;

        hello_world::vision::Vision *vision_ptr_ = nullptr;

        // UI 组件
        UiMgr *ui_mgr_ptr_ = nullptr;
        UiDrawer *ui_drawer_ptr_ = nullptr;


    };
} // namespace robot

#endif /* SENTRY_MODULES_ROBOT_HPP_ */