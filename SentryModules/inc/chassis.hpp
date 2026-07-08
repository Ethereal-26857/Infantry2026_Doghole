#ifndef SENRTY_MODULES_CHASSIS_HPP_
#define SENRTY_MODULES_CHASSIS_HPP_

#include "allocator.hpp"
#include "chassis_iksolver.hpp"
#include "module_fsm.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "power_limiter.hpp"
#include "super_cap.hpp"

namespace robot
{
    union ChassisCmd
    {
        struct
        {
            float v_x;
            float v_y;
            float w;
        };
        float data[3];
        void reset()
        {
            v_x = 0;
            v_y = 0;
            w = 0;
        }
        ChassisCmd operator+(const ChassisCmd &other) const { return {v_x + other.v_x, v_y + other.v_y, w + other.w}; }

        ChassisCmd operator-(const ChassisCmd &other) const { return {v_x - other.v_x, v_y - other.v_y, w - other.w}; }

        ChassisCmd operator*(float scalar) const { return {v_x * scalar, v_y * scalar, w * scalar}; }

        ChassisCmd operator+=(const ChassisCmd &other)
        {
            v_x += other.v_x;
            v_y += other.v_y;
            w += other.w;
            return *this;
        }

        ChassisCmd operator-=(const ChassisCmd &other)
        {
            v_x -= other.v_x;
            v_y -= other.v_y;
            w -= other.w;
            return *this;
        }

        ChassisCmd operator*=(float scalar)
        {
            v_x *= scalar;
            v_y *= scalar;
            w *= scalar;
            return *this;
        }

        friend ChassisCmd operator*(float scalar, const ChassisCmd &cmd);
    };

    struct ChassisRfrData
    {
        bool is_pwr_on = true;    ///< 机器人底盘电源是否开启【裁判系统告知，离线时默认开启】
        float pwr = 0;            ///< 机器人底盘功率【裁判系统告知，离线时默认为0】
        uint16_t pwr_limit = 100; ///< 机器人底盘功率限制【裁判系统告知，离线时默认为默认值】
        uint16_t pwr_buffer = 60; ///< 机器人底盘功率缓冲【裁判系统告知，离线时默认为默认值】

        uint16_t voltage = 24;   ///< 机器人底盘电压【裁判系统告知，离线时默认为默认值】
        uint16_t current_hp = 0; ///< 机器人底盘电流【裁判系统告知，离线时默认为默认值】
        uint8_t current_remaining_energy = 31; ///< 机器人底盘低压电流【裁判系统告知，离线时默认为默认值】
    };

    struct ChassisConfig
    {
        float norm_trans_vel; ///< 正常模式下(超电没开启）最大平移速度
        float norm_rot_spd;   ///< 正常模式下（超电没开启）最大旋转速度
        float max_trans_vel;  ///< 最大平移速度
        float max_rot_spd;    ///< 最大旋转速度
        float min_follow_spd; ///< 最小跟随速度
        float ffd_kp_gyro; ///< 小陀螺前馈增益
        float ffd_kp_vel;  ///< 平移速度前馈增益
        float real_intilthole_ang = 0.486145430783f; //rad
        float real_insgthole_ang = 0.0f;//rad
    };

    class Chassis : public Fsm
    {
    public:
        typedef hello_world::motor::Motor Motor;
        typedef hello_world::pid::MultiNodesPid MultiNodesPid;
        typedef hello_world::chassis_ik_solver::ChassisIkSolver ChassisIkSolver;
        typedef hello_world::cap::SuperCap Cap;
        typedef hello_world::power_limiter::PowerLimiter PwrLimiter;

        typedef ChassisWorkingMode WorkingMode;

        typedef ChassisCmd Cmd;
        typedef ChassisRfrData RfrData;
        typedef ChassisConfig Config;

        enum class GyroDir : int8_t
        {
            kNotRotate = 0,     ///< 不旋转
            kAntiClockwise = 1, ///< 逆时针旋转
            kClockwise = -1,    ///< 顺时针旋转
        };

        enum WheelMotorIdx : uint8_t
        {
            kWheelMotorIdxLeftFront = 0,  ///< 左前轮电机下标
            kWheelMotorIdxLeftRear,   ///< 左后轮电机下标
            kWheelMotorIdxRightRear,  ///< 右后轮电机下标
            kWheelMotorIdxRightFront, ///< 右前轮电机下标
            kWheelMotorNum            ///< 轮电机数量
        };

        enum PidIdx : uint8_t
        {
            kPidIdxWheelMotorLeftFront,   ///< 左前轮电机PID下标
            kPidIdxWheelMotorLeftRear,    ///< 左后轮电机PID下标
            kPidIdxWheelMotorRightRear,   ///< 右后轮电机PID下标
            kPidIdxWheelMotorRightFront,  ///< 右前轮电机PID下标
            kPidIdxFollowOmega,        ///< 跟随模式下底盘绕Z轴旋转的PID
            kPidIdxFollowSpeedOmega,       ///< 跟随模式下底盘合成速度的PID
            kPidIdxGyro,               ///< 小陀螺PID下标
            kPidNum ///< PID数量
        };

        Chassis(const Config &config) { config_ = config; };
        ~Chassis() {};

        virtual void update() override;
        virtual void run() override;
        virtual void reset() override;
        virtual void standby() override;

        WorkingMode getWorkingMode() const { return working_mode_; };
        WorkingMode getLastWorkingMode() const { return last_working_mode_; };
        void setWorkingMode(WorkingMode mode)
        {
            if (working_mode_ != mode)
            {
                last_working_mode_ = working_mode_;
                working_mode_ = mode;
            }
        }
        void setNormCmd(const Cmd &cmd) { norm_cmd_ = cmd; }
        void setRfrData(const RfrData &data) { rfr_data_ = data; }

        void setGyroDir(GyroDir dir) { gyro_dir_ = dir; }
        void changeGyroDir()
        {
            if (gyro_dir_ == GyroDir::kClockwise)
            {
                gyro_dir_ = GyroDir::kAntiClockwise;
            }
            else if (gyro_dir_ == GyroDir::kAntiClockwise)
            {
                gyro_dir_ = GyroDir::kClockwise;
            }
        }

        void setUseCapFlag(bool flag) { use_cap_flag_ = flag; }
        bool getUseCapFlag() const { return use_cap_flag_; }

        void setSentryInhole(uint8_t flag) {is_sentry_inhole_ = flag; }
        void setSentryLastInhole(uint8_t flag) {last_is_sentry_inhole_ = flag; }
        void setImuAng (float imu_ang) {imu_fdb_ang_ = imu_ang; }
        void setImuRealAng(float imu_real_ang) {imu_real_ang_ = imu_real_ang; }

        float getConfigMaxTransVel() const { return config_.max_trans_vel; }
        float getConfigMaxRotSpd() const { return config_.max_rot_spd; }

        float getConfigNormTransVel() const { return config_.norm_trans_vel; }
        float getConfigNormRotSpd() const { return config_.norm_rot_spd; }
        float getChassisVelX() const { return actual_vx; }
        float getChassisVelY() const { return actual_vy; }

        void registerIkSolver(ChassisIkSolver *ptr);
        void registerWheelMotor(Motor *ptr, uint8_t idx);
        void registerYawMotor(Motor *ptr);
        void registerPid(MultiNodesPid *ptr, uint8_t idx);
        void registerCap(Cap *ptr);
        void registerPwrLimiter(PwrLimiter *ptr);

    private:
        void updateData();
        void updateMotor();
        void updateCap();
        void updateIsPowerOn();
        void updatePwrState();

        void runOnDead();
        void runOnResurrection();
        void runOnWorking();

        // 工作状态下，获取控制指令的函数
        void revNormCmd();
        void revNormCmdOnDepart(Cmd &cmd);
        void revNormCmdOnFollow(Cmd &cmd);
        void revNormCmdOnFollowSpeed(Cmd &cmd);
        void revNormCmdOnInHole(Cmd &cmd, float real_inhole_ang);
        void revNormCmdOnFastGyro(Cmd &cmd);
        void revNormCmdOnSlowGyro(Cmd &cmd);
        void BoundCmd(Cmd &cmd);
        void calcMotorRef();
        void calcMotorLimitedRef();
        void calcWheelCurrentRef();

        // 重置数据
        void resetDataOnDead();
        void resetDataOnResurrection();
        void resetDataOnStandby();
        void resetPids();

        // 设置通信组件数据函数
        void setCommData(bool working_flag)
        {
            setCommDataWheels(working_flag);
            setCommDataCap(working_flag);
        };
        void setCommDataWheels(bool working_flag);
        void setCommDataCap(bool working_flag);

        // 工具函数
        void setCmdSmoothly(const Cmd &cmd, float beta);
        void iksolver_forward();

        // 配置参数
        Config config_;

        // 由 robot 设置的数据
        // TODO: 还需要确定哨兵超电启动的方式
        bool use_cap_flag_ = false;              ///< 是否使用超级电容
        GyroDir gyro_dir_ = GyroDir::kNotRotate; ///< 小陀螺旋转方向
        Cmd norm_cmd_ = {0.0f, 0.0f, 0.0f};      ///< 原始控制指令，基于大YAW坐标系
        Cmd speed_solve_cmd_ = {0.0f, 0.0f, 0.0f}; ///< 速度求解指令，debug
        ChassisRfrData rfr_data_;                ///< 底盘裁判系统数据

        WorkingMode working_mode_ = WorkingMode::kDepart;      ///< 工作模式
        WorkingMode last_working_mode_ = WorkingMode::kDepart; ///< 上一工作模式

        // 在 update 函数中更新的数据据
        bool is_power_on_ = true; ///< 机器人底盘电源是否开启
        uint32_t last_pwr_off_tick_ = 0; ///< 上一次底盘电源关闭的时间
        uint32_t resurrection_tick_ = 0; ///< 机器人底盘复活的时间
        float actual_vx = 0.0f; ///< 实际的底盘vx速度 m/s
        float actual_vy = 0.0f; ///< 实际的底盘vy

        // 在 runOnWorking 函数中更新的数据
        Cmd cmd_ = {0.0f, 0.0f, 0.0f};                             ///< 控制指令，基于大YAW坐标系
        Cmd last_cmd_ = {0.0f, 0.0f, 0.0f};                        ///< 上一次的控制指令
        float wheel_speed_ref_[kWheelMotorNum] = {0.0f};           ///< 轮电机期望速度 rad/s
        float wheel_limited_speed_ref_[kWheelMotorNum] = {0.0f};   ///< 轮电机期望速度（限幅后）rad/s
        float wheel_current_ref_[kWheelMotorNum] = {0.0f};         ///< 轮电机期望电流 A
        float chassis_composite_vel_ref_ = 0.0f; ///< 底盘合成速度参考 m/s
        float k_limit_ = 0.0f;                  ///< 功率限制系数

        // motor fdb data 在 update 函数中更新
        // NOTE: 考虑如果有舵电机掉线或者轮电机掉线的时候的处理方式
        bool is_all_motor_online_ = false;                   ///< 所有电机是否在线
        bool is_any_motor_online_ = false;                   ///< 任意电机是否在线
        float wheel_speed_fdb_[kWheelMotorNum] = {0.0f};     ///< 轮电机反馈速度 rad/s
        float wheel_current_fdb_[kWheelMotorNum] = {0.0f};   ///< 轮电机反馈电流 A
        float theta_y2r_ = 0.0f;                             ///< 大YAW坐标系到底盘坐标系的旋转角度 rad
        float spd_y2r_ = 0.0f;                               ///< 大YAW坐标系到底盘坐标系的旋转速度 rad/s
        float chassis_composite_vel_fdb_ = 0.0f; ///< 底盘合成速度反馈 m/s

        // cap fdb data 在 update 函数中更新
        // TODO: 暂时没有超电，后续补充测试
        bool is_high_spd_enabled_ = false; ///< 是否开启高速模式 (开启意味从超电取电)
        bool super_cap_lock_ = false;
        float cap_remaining_energy_ = 0.0f; ///< 超级电容剩余能量百分比，单位：%
     
        bool last_is_sentry_inhole_ = false; ///< 上一次哨兵是否在洞内
        bool is_sentry_inhole_ = false;
        float imu_fdb_ang_ = 0.0f;//robot中更新 rad
        float imu_real_ang_ = 0.0f; ///< IMU实际角度，单位：rad

        // 各组件指针
        ChassisIkSolver *ik_solver_ptr_ = nullptr;    ///< 底盘逆解器
        MultiNodesPid *pid_ptr_[kPidNum] = {nullptr}; ///< PID控制器
        PwrLimiter *pwr_limiter_ptr_ = nullptr;       ///< 功率限制器

        // 只接收数据的组件指针
        Motor *yaw_motor_ptr_ = nullptr; ///< 大YAW电机指针，只接收数据

        // 接收、发送数据的组件指针
        Cap *cap_ptr_ = nullptr;                                ///< 超级电容指针
        Motor *wheel_motor_ptr_[kWheelMotorNum] = {nullptr};   ///< 轮电机指针
    };

    inline ChassisCmd operator*(float scalar, const ChassisCmd &cmd) { return cmd * scalar; };

} // namespace robot

#endif /* SENRTY_MODULES_CHASSIS_HPP_ */