#ifndef SENTRY_MODULES_GIMBAL_HPP_
#define SENTRY_MODULES_GIMBAL_HPP_

#include "filter.hpp"
#include "imu.hpp"
#include "module_fsm.hpp"
#include "module_state.hpp"
#include "motor.hpp"
#include "pid.hpp"

namespace robot
{
union GimbalCmd {
    struct
    {
        float gimbal_pitch;
        float gimbal_yaw;
    };
    float data[2];

    GimbalCmd operator+(const GimbalCmd &other) const { return {gimbal_pitch + other.gimbal_pitch, gimbal_yaw + other.gimbal_yaw}; }

    GimbalCmd operator-(const GimbalCmd &other) const { return {gimbal_pitch - other.gimbal_pitch, gimbal_yaw - other.gimbal_yaw}; }

    GimbalCmd operator*(const float &scale) const { return {gimbal_pitch * scale, gimbal_yaw * scale}; }

    GimbalCmd operator+=(const GimbalCmd &other) { return *this = *this + other; }

    GimbalCmd operator-=(const GimbalCmd &other) { return *this = *this - other; }

    GimbalCmd operator*=(const float &scale) { return *this = *this * scale; }

    friend GimbalCmd operator*(const float &scale, const GimbalCmd &cmd);
};

class Gimbal : public Fsm
{
   public:
    typedef hello_world::motor::DaMiao Motor;
    typedef hello_world::pid::MultiNodesPid Pid;

    typedef GimbalCmd Cmd;

    struct Config {

        struct GimbalJointConfig {
            float pid_spd_kp;          ///< 速度环 kp
            float sensitivity_normal;  ///< 手动模式下的灵敏度，单位：rad/ms
            float sensitivity_search;  ///< 巡航模式下的灵敏度，单位：rad/ms
            float max_ang_motor;       ///< 机械最大角度，单位：rad
            float min_ang_motor;       ///< 机械最小角度，单位：rad
            float max_ang_outpost;     ///< 前哨模式下的最大角度，单位：rad
            float min_ang_outpost;     ///< 前哨模式下的最小角度，单位：rad
            float max_ang_search;      ///< 巡逻模式下的最大角度，单位：rad
            float min_ang_search;      ///< 巡逻模式下的最小角度，单位：rad
            float fix_ang_search=0.0f;  ///< 云台巡逻模式下的固定俯仰角度，单位：Deg
            float fix_ang_outpost=25.0f;  ///< 云台前哨模式下的固定俯仰角度，单位：Deg
        } GimbalPitch, GimbalYaw;
    };

    enum JointIdx : uint8_t {
        kJointIdxGimbalPitch = 0U,
        kJointIdxGimbalYaw = 1U,
        kJointIdxGimbalRoll = 2U,
        kJointNum = 3U,
    };

    enum Dir : int8_t {
        kDirPos = 1,
        kDirNeg = -1,
    };

    enum VisionCmdStatus : uint8_t {
        kPitchOutOfRange = 0,
        kYawBigerThanRange = 1,
        kYawSmallerThanRange = 2,
        kVaild = 3,
    };

    Gimbal(const Config &cfg) { cfg_ = cfg; }
    ~Gimbal() {};

    void update() override;

    void run() override;

    void reset() override;

    void standby() override;

    void updateIsRfrPwrOn(bool flag) { is_rfr_pwr_on_ = flag; }

    void setNormCmdDelta(const Cmd &cmd) { norm_cmd_delta_ = cmd; }
    void setNormCmdDelta(float gimbal_pitch, float gimbal_yaw)
    {
        norm_cmd_delta_.gimbal_pitch = gimbal_pitch;
        norm_cmd_delta_.gimbal_yaw = gimbal_yaw;
    }
    const Cmd &getNormCmdDelta() const { return norm_cmd_delta_; }

    VisionCmdStatus isVisCmdVaild(float pitch, float yaw);
    bool isVisCmdInRange(float pitch, float yaw);

    void setVisCmd(float pitch, float yaw)
    {
        // NOTE: 自瞄下发的数据为云台的角度

        vis_cmd_.gimbal_pitch = pitch;
        vis_cmd_.gimbal_yaw = yaw;
    }

    float getJointGimbalPitchAng() const { return joint_ang_fdb_[kJointIdxGimbalPitch]; }
    float getJointGimbalYawAng() const { return joint_ang_fdb_[kJointIdxGimbalYaw]; }

    void setGimbalJointWorkingMode(GimbalJointWorkingMode mode)
    {
        gimbal_joint_working_mode_ = mode;
    }
    GimbalJointWorkingMode getGimbalJointWorkingMode() const { return gimbal_joint_working_mode_; }

    void setSearchGimbalYawMoveDir(Dir dir) { search_gimbal_yaw_move_dir_ = dir; }
    Dir getSearchGimbalYawMoveDir() const { return search_gimbal_yaw_move_dir_; }

    void setSearchGimbalPitchMoveDir(Dir dir) { search_gimbal_pitch_move_dir_ = dir; }
    Dir getSearchGimbalPitchMoveDir() const { return search_gimbal_pitch_move_dir_; }

    // 注册组件指针
    void registerMotor(Motor *ptr, JointIdx idx);
    void registerPid(Pid *ptr, JointIdx idx);
    void setjumpFlag(bool flag) { jump_flag = flag; }

   private:
    // 数据更新
    void updateData();
    void updateMotorData();
    void updateIsPwrOn();
    void updatePwrState();
    void updateJointData();

    // 任务执行
    void runOnDead();
    void runOnResurrection();
    void runOnWorking();

    void calcJointAngRefOnStandby();
    void calcJointAngRef();

    void calcGimbalJointAngRefReset();
    void calcGimbalJointAngRefManualMode();
    void calcGimbalJointAngRefSearchMode();
    void calcGimbalJointAngRefOutpostMode();
    void calcGimbalJointAngRefInholeMode();
    void calcGimbalJointAngRefBuffMode();

    void calcGimbalJointAngRefAutoAimMode();

    void calcJointTorRef();
    void calcGimbalJointTorRef();

    //工具复用函数

    void GimbalYawSearch(float min , float max);
    void GimbalPitchSearch(float min, float max);

    // 数据重置
    void resetDataOnDead();
    void resetDataOnResurrection();
    void resetPids();

    // 设置通讯数据
    void setCommData(bool working_flag);

    // 由 robot 设置的数据
    bool is_rfr_pwr_on_ = false;  ///< 裁判系统电源管理 Gimbal 是否输出

    Cmd norm_cmd_delta_ = {0.0, 0.0};  ///< 控制指令的增量
    Cmd vis_cmd_ = {0.0, 0.0};         ///< 视觉控制指令

    GimbalJointWorkingMode gimbal_joint_working_mode_ = GimbalJointWorkingMode::kManual;       ///< 云台工作模式
    GimbalJointWorkingMode gimbal_joint_last_working_mode_ = GimbalJointWorkingMode::kManual;  ///< 云台上一工作模式

    Dir search_gimbal_yaw_move_dir_ = kDirPos;    ///< 巡逻模式下Yaw轴运动方向
    Dir search_gimbal_pitch_move_dir_ = kDirNeg;  ///< 巡逻模式下Pitch轴运动方向

    // 由 Gimbal 内部维护的数据
    Config cfg_;  ///< 配置参数

    bool is_pwr_on_ = false;              ///< 电源是否开启
    bool is_first_enter_standby_ = true;  ///< 是否第一次进入待机状态
    bool jump_flag= false;

    // 控制电机的 PID 所需的数据
    float last_joint_ang_ref_[kJointNum] = {0.0f};  ///< 上一控制周期 Joint 关节角度期望值
    float joint_ang_ref_[kJointNum] = {0.0f};       ///< Joint 关节角度期望值
    float joint_ang_fdb_[kJointNum] = {0.0f};       ///< Joint 关节角度反馈值
    float joint_spd_fdb_[kJointNum] = {0.0f};       ///< Joint 关节速度反馈值
    float joint_input_ref_[kJointNum] = {0.0f};     ///< Joint 关节输入期望值

    // 从电机中拿到的数据
    bool is_any_motor_pwr_on_ = false;  ///< 是否有电机上电
    bool is_all_motor_pwr_on_ = false;  ///< 是否所有电机上电

    float motor_ang_fdb_[kJointNum] = {0.0f};       ///< 电机角度反馈值
    float motor_spd_fdb_[kJointNum] = {0.0f};       ///< 电机速度反馈值
    float last_motor_spd_fdb_[kJointNum] = {0.0f};  ///< 上一次的电机速度反馈值

    // PID 数据，调参用
    Pid::Datas pid_data[kJointNum][2];

    // 各组件指针
    // 无通信功能的组件指针
    Pid *pid_ptr_[kJointNum] = {nullptr};  ///< PID 指针

    // 接收、发送数据的组件指针
    Motor *motor_ptr_[kJointNum] = {nullptr};  ///< 电机指针

};  // class Gimbal

inline GimbalCmd operator*(float scalar, const GimbalCmd &cmd)
{
    return {scalar * cmd.gimbal_pitch, scalar * cmd.gimbal_yaw};
}

};  // namespace robot

#endif /* SENTRY_MODULES_GIMBAL_HPP_ */