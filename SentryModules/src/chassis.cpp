#include "chassis.hpp"
#include "ins_chassis_iksolver.hpp"
#include "arm_math.h"

// DEBUG: PID 调试时使用
static float debug_spd_ref[4] = {0.0f};
static float debug_spd_fdb[4] = {0.0f};
static float debug_spd_out[4] = {0.0f};

static float debug_ang_flow_ref = 0.0f;
static float debug_ang_flow_fdb = 0.0f;
static float debug_ang_flow_out = 0.0f;

namespace robot
{ 
#pragma region 数据更新

    void Chassis::update()
    {
        updateData();
        updatePwrState();
    };

    void Chassis::updateData()
    {
        work_tick_ = getCurrentTickMs();

        iksolver_forward();
        updateMotor();
        updateCap();
        updateIsPowerOn();
    };

    /**
     * @brief 更新电源状态
     *
     * 无论处于任何状态，只要掉电就切换到死亡状态
     * 死亡状态下，如果上电，则切到复活状态
     * 复活状态下，如果有轮电机上电完毕（底盘已经准备完毕），则切换到工作状态
     * 工作状态下，保持当前状态
     * 其他状态下，认为为死亡状态
     */
    void Chassis::updatePwrState()
    {
        // 无论处于任何状态，只要掉电就切换到死亡状态
        if (!is_power_on_)
        {
            setPwrState(PwrState::kDead);
            return;
        }

        PwrState current_state = getPwrState();
        PwrState next_state = current_state;

        if (current_state == PwrState::kDead)
        {
            // 死亡状态下，如果上电，则切到复活状态
            if (is_power_on_)
            {
                resurrection_tick_ = 0;
                next_state = PwrState::kResurrection;
            }
        }
        else if (current_state == PwrState::kResurrection)
        {
            // 复活状态下，如果有轮电机上电完毕（底盘已经准备完毕），且云台板准备就绪，则切到工作状态
            // 1. 为什么只要有轮电机上电完毕，就认为底盘就准备好了？
            // 因为底盘逆解不需要所有轮电机都在线也可以运行
            // 在后续正常工作状态下，会对轮电机的状态进行检测
            // 如果有轮电机掉电，会进行专门的处理，比如 pid 清空、can 发无效数据等
            // 2. 所有控制模式都需要 yaw 轴电机的角度，为什么不判断 yaw 轴电机的状态？
            // 因为 yaw 轴电机的状态不影响底盘的运动解算
            // 当 yaw 轴电机离线时，底盘会按照底盘坐标系进行解算（yaw 电机离线后数据清空，默认返回 0，能跑但是疯了，但总比不能跑强）
            // 当 yaw 轴电机上电时，底盘会按照图传坐标系进行解算
            if (is_any_motor_online_)
            {
                resurrection_tick_++;
            } else {
                resurrection_tick_ = 0;
            }
            
            if (resurrection_tick_ > 2000)
            {
                next_state = PwrState::kWorking;
            }
        }
        else if (current_state == PwrState::kWorking)
        {
            // 工作状态下，保持当前状态
            next_state = PwrState::kWorking;
        }
        else
        {
            // 其他状态下，认为为死亡状态
            next_state = PwrState::kDead;
        }

        setPwrState(next_state);
    };

    void Chassis::updateMotor()
    {
        Motor *wheel_motor_ptr = nullptr;
        bool is_all_motor_online = true;
        bool is_any_motor_online = false;

        for (size_t i = 0; i < kWheelMotorNum; i++)
        {
            wheel_motor_ptr = wheel_motor_ptr_[i];
            HW_ASSERT(wheel_motor_ptr != nullptr, "pointer to motor %d is nullptr", i);
            if (wheel_motor_ptr->isOffline())
            {
                wheel_speed_fdb_[i] = 0.0f;
                wheel_current_fdb_[i] = 0.0f;
                is_all_motor_online = false;
            }
            else
            {
                wheel_speed_fdb_[i] = wheel_motor_ptr->vel();
                wheel_current_fdb_[i] = wheel_motor_ptr->curr();
                is_any_motor_online = true;
            }
        }

        is_all_motor_online_ = is_all_motor_online;
        is_any_motor_online_ = is_any_motor_online;

        HW_ASSERT(yaw_motor_ptr_ != nullptr, "pointer to yaw motor is nullptr");
        if (yaw_motor_ptr_->isOffline())
        {
            theta_y2r_ = 0.0f;
            spd_y2r_ = 0.0f;
        }
        else
        {
            theta_y2r_ = -1.0f * yaw_motor_ptr_->angle();
            spd_y2r_ = -1.0f * yaw_motor_ptr_->vel();
        }
    };

    void Chassis::updateCap()
    {
        HW_ASSERT(cap_ptr_ != nullptr, "pointer to cap is nullptr");
        if (cap_ptr_->isOffline())
        {
            is_high_spd_enabled_ = false;
            cap_remaining_energy_ = 0.0f;
        }
        else
        {
            is_high_spd_enabled_ = cap_ptr_->isUsingSuperCap();
            cap_remaining_energy_ = cap_ptr_->getRemainingPower();
        }
    };

    void Chassis::updateIsPowerOn()
    {
        is_power_on_ = rfr_data_.is_pwr_on || is_any_motor_online_;
        if (!is_power_on_)
        {
            last_pwr_off_tick_ = work_tick_;
        }
    };

#pragma endregion

#pragma region 执行任务
    void Chassis::run()
    {
        if (pwr_state_ == PwrState::kDead)
        {
            runOnDead();
        }
        else if (pwr_state_ == PwrState::kResurrection)
        {
            runOnResurrection();
        }
        else if (pwr_state_ == PwrState::kWorking)
        {
            runOnWorking();
        }
        else
        {
            runOnDead();
        }
    };

    void Chassis::runOnDead()
    {
        resetDataOnDead();
        setCommData(false);
    };

    void Chassis::runOnResurrection()
    {
        resetDataOnResurrection();
        setCommData(false);
    };

    void Chassis::runOnWorking()
    {
        revNormCmd();
        calcMotorRef();
        calcMotorLimitedRef();

        calcWheelCurrentRef();
        last_is_sentry_inhole_ = is_sentry_inhole_;
        setCommData(true);
    };

    void Chassis::standby()
    {
        resetDataOnStandby();
        setCommData(false);
    };

#pragma endregion

#pragma region 工作状态下，获取控制指令的函数

    void Chassis::revNormCmd()
    {
        float beta = 0.9f;
        last_cmd_ = cmd_;
        cmd_ = norm_cmd_;

        arm_sqrt_f32(cmd_.v_x * cmd_.v_x + cmd_.v_y * cmd_.v_y, &chassis_composite_vel_ref_);

        if (working_mode_ == WorkingMode::kDepart)
        {
            revNormCmdOnDepart(cmd_);
        }
        else if (working_mode_ == WorkingMode::kFollow)
        {
            revNormCmdOnFollow(cmd_);
        }
        else if (working_mode_ == WorkingMode::kFollowSpeed)
        {
            revNormCmdOnFollowSpeed(cmd_);
        }
        else if (working_mode_ == WorkingMode::kInTiltHole)
        {
            revNormCmdOnInHole(cmd_ , config_.real_intilthole_ang);
        }
        else if (working_mode_ == WorkingMode::kInSgtHole)
        {
            revNormCmdOnInHole(cmd_ , config_.real_insgthole_ang);
        }
        else if (working_mode_ == WorkingMode::kFastGyro)
        {
            revNormCmdOnFastGyro(cmd_);
        }
        else if (working_mode_ == WorkingMode::kSlowGyro)
        {
            revNormCmdOnSlowGyro(cmd_);
        }

        if (working_mode_ != WorkingMode::kFastGyro &&
            working_mode_ != WorkingMode::kSlowGyro)
        {
            gyro_dir_ = GyroDir::kNotRotate;
        }

        BoundCmd(cmd_);
        setCmdSmoothly(cmd_, beta);
    };

    void Chassis::BoundCmd(Cmd &cmd)
    {
        float max_trans_vel = is_high_spd_enabled_ ? config_.max_trans_vel : config_.norm_trans_vel;
        float max_rot_spd = is_high_spd_enabled_ ? config_.max_rot_spd : config_.norm_rot_spd;

        // 等比限幅，防止速度畸变 # FOR 导航
        if (chassis_composite_vel_ref_ > max_trans_vel)
        {
            float scale = max_trans_vel / chassis_composite_vel_ref_;
            cmd.v_x *= scale;
            cmd.v_y *= scale;
        }

        cmd.w = hello_world::Bound(cmd.w, -max_rot_spd, max_rot_spd);
    };

    void Chassis::revNormCmdOnDepart(Cmd &cmd) {
        // 分离模式下，不对原始控制指令 norm_cmd_ 进行额外处理
    };

    void Chassis::revNormCmdOnFastGyro(Cmd &cmd)
    {
        // 小陀螺模式下，如果外部没有设置旋转方向，则随机选择
        if (gyro_dir_ == GyroDir::kNotRotate)
        {
            if (rand() % 2)
            {
                gyro_dir_ = GyroDir::kClockwise;
            }
            else
            {
                gyro_dir_ = GyroDir::kAntiClockwise;
            }
        }

        cmd.w = config_.max_rot_spd * static_cast<float>(gyro_dir_);
    };

    void Chassis::revNormCmdOnSlowGyro(Cmd &cmd)
    {
        // 小陀螺模式下，如果外部没有设置旋转方向，则随机选择
        if (gyro_dir_ == GyroDir::kNotRotate)
        {
            if (rand() % 2)
            {
                gyro_dir_ = GyroDir::kClockwise;
            }
            else
            {
                gyro_dir_ = GyroDir::kAntiClockwise;
            }
        }

        cmd.w = 0.5 * config_.max_rot_spd * static_cast<float>(gyro_dir_);
    };

    void Chassis::revNormCmdOnFollow(Cmd &cmd)
    {
        // NOTE: 导航发布信息有一个wz的参数，跟随模式跟随云台即可
        MultiNodesPid *pid = pid_ptr_[kPidIdxFollowOmega];
        HW_ASSERT(pid != nullptr, "pointer to follow pid is nullptr", pid);
        
        static float theta_ref_temp = 0.0f;

        float theta_ref[1] = {theta_ref_temp};
        float theta_fdb[2] = {theta_y2r_, spd_y2r_};
        pid->calc(theta_ref, theta_fdb, nullptr, &cmd.w);
        
        debug_ang_flow_ref = theta_ref[0];
        debug_ang_flow_fdb = theta_fdb[0];
        debug_ang_flow_out = cmd.w;
        // NOTE: 速度的坐标系转换已经在逆解中完成
        // 保证跟随过程中大Yaw坐标系和底盘坐标系方向不一致的时候，速度仍能正确映射到底盘坐标系下
        // cmd.v_x = cmd.v_x * cosf(theta_y2r_) - cmd.v_y * sinf(theta_y2r_);
        // cmd.v_y = cmd.v_x * sinf(theta_y2r_) + cmd.v_y * cosf(theta_y2r_);
    };

    void Chassis::revNormCmdOnInHole(Cmd &cmd ,float real_inhole_ang)
    {
        // NOTE: 导航发布信息有一个wz的参数，跟随模式跟随进洞方向
        MultiNodesPid *pid = pid_ptr_[kPidIdxFollowOmega];
        HW_ASSERT(pid != nullptr, "pointer to follow pid is nullptr", pid);
        static float theta_ref_temp = 0.0f;
        // //imu_ang + imu_err_ang = world_ang
        // //world_ang + theta_y2r_ref = chassis_ang = real_inhole_ang``
        // // real_inhole_ang = imu_ang + imu_err_ang + theta_y2r_ref
        // float inhole_ang_norm = hello_world::AngleNormRad(real_inhole_ang - imu_real_ang_);
        // float inhole_ang_more = hello_world::AngleNormRad(real_inhole_ang - imu_real_ang_ + PI);
        // theta_ref_temp = fabs(inhole_ang_norm - theta_y2r_) < fabs(inhole_ang_more - theta_y2r_) ?
        //     inhole_ang_norm : inhole_ang_more;
        float theta_ref[1] = {theta_ref_temp};
        float theta_fdb[2] = {theta_y2r_, spd_y2r_};
        pid->calc(theta_ref, theta_fdb, nullptr, &cmd.w);
        
        debug_ang_flow_ref = theta_ref[0];
        debug_ang_flow_fdb = theta_fdb[0];
        debug_ang_flow_out = cmd.w;
    };

    void Chassis::revNormCmdOnFollowSpeed(Cmd &cmd)
    {
        MultiNodesPid *pid = pid_ptr_[kPidIdxFollowSpeedOmega];
        HW_ASSERT(pid != nullptr, "pointer to follow pid is nullptr", pid);
        
        static float theta_ref_temp = 0.0f;
        arm_atan2_f32(cmd.v_y, cmd.v_x, &theta_ref_temp);

        float theta_ref[1] = {theta_ref_temp};
        float theta_fdb[2] = {theta_y2r_, spd_y2r_};
        pid->calc(theta_ref, theta_fdb, nullptr, &cmd.w);
    }

    void Chassis::calcMotorRef()
    {
        // 底盘坐标系下，x轴正方向为底盘正前方，y轴正方向为底盘正左方，z轴正方向为底盘正上方
        // 轮子顺序按照象限顺序进行编号：左前，左后，右后，右前
        HW_ASSERT(ik_solver_ptr_ != nullptr, "pointer to IK solver is nullptr", ik_solver_ptr_);
        hello_world::chassis_ik_solver::MoveVec move_vec(cmd_.v_x, cmd_.v_y, cmd_.w);
        ik_solver_ptr_->solve(move_vec, theta_y2r_, nullptr);
        ik_solver_ptr_->getRotSpdAll(wheel_speed_ref_);
    };

    void Chassis::calcMotorLimitedRef() {
        pwr_limiter_ptr_->updateWheelModel(wheel_speed_ref_, wheel_speed_fdb_, nullptr, wheel_current_fdb_);
        PwrLimiter::RuntimeParams runtime_params = {
            .p_ref_max = rfr_data_.pwr_limit * 1.2f,
            .p_referee_max = 80.0f,/*static_cast<float>(rfr_data_.pwr_limit),*/
            .p_ref_min = rfr_data_.pwr_limit * 0.8f,
            .remaining_energy =static_cast<float>(rfr_data_.pwr_buffer),/*cap_remaining_energy_,*/
            .energy_converge = 20.0f,
            .p_slope = 1.5f,
            .danger_energy = 5.0f,
        };
        if(rfr_data_.current_remaining_energy <= 0x10){
            runtime_params.p_ref_max = rfr_data_.pwr_limit * 1.2f;
            runtime_params.energy_converge = 20.0f;
            runtime_params.p_ref_min = rfr_data_.pwr_limit * 0.8f;
            runtime_params.p_referee_max = static_cast<float>(rfr_data_.pwr_limit);
            runtime_params.remaining_energy = static_cast<float>(rfr_data_.pwr_buffer);
            runtime_params.p_slope=3.0f;
        }
        pwr_limiter_ptr_->calc(runtime_params, wheel_limited_speed_ref_);
        auto model = pwr_limiter_ptr_->getWheelMotorModel(0);
    };

    void Chassis::calcWheelCurrentRef()
    {
        PidIdx wpis[4] = {
            kPidIdxWheelMotorLeftFront,
            kPidIdxWheelMotorLeftRear,
            kPidIdxWheelMotorRightRear,
            kPidIdxWheelMotorRightFront};
        MultiNodesPid *pid_ptr = nullptr;
        for (size_t i = 0; i < 4; i++)
        {
            pid_ptr = pid_ptr_[wpis[i]];
            HW_ASSERT(pid_ptr != nullptr, "pointer to PID %d is nullptr", wpis[i]);
            //#TODO 轮电机前馈
            float ffd =0.0f;
            if (wheel_limited_speed_ref_[i] > 0.0f)
            {
                ffd = 0.4f * 16384.0f / 20.0f;
            }
            else if (wheel_limited_speed_ref_[i] < 0.0f)
            {
                ffd = -0.4f * 16384.0f / 20.0f;
            }
            pid_ptr->calc(&wheel_limited_speed_ref_[i], &wheel_speed_fdb_[i], nullptr, &wheel_current_ref_[i]);
            // pid_ptr->calc(&wheel_speed_ref_[i], &wheel_speed_fdb_[i], nullptr, &wheel_current_ref_[i]);
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            debug_spd_ref[i] = wheel_limited_speed_ref_[i];
            debug_spd_fdb[i] = wheel_speed_fdb_[i];
            debug_spd_out[i] = wheel_current_ref_[i];
        }
    }

#pragma endregion

#pragma region 数据重置函数

    void Chassis::reset()
    {
        pwr_state_ = PwrState::kDead;      //< 电源状态
        last_pwr_state_ = PwrState::kDead; //< 上一次电源状态

        // 由robot设置的数据
        use_cap_flag_ = false;           //< 是否使用超级电容
        gyro_dir_ = GyroDir::kNotRotate; //< 小陀螺方向，正为绕 Z 轴逆时针，负为顺时针，
        norm_cmd_.reset();               //< 原始控制指令，基于图传坐标系
        rfr_data_ = RfrData();           //< 底盘 RFR 数据

        working_mode_ = WorkingMode::kDepart;      //< 工作模式
        last_working_mode_ = WorkingMode::kDepart; //< 上一次工作模式

        // 在 update 函数中更新的数据
        is_power_on_ = false; //< 底盘电源是否开启

        // 在 runOnWorking 函数中更新的数据
        cmd_.reset();      //< 控制指令，基于图传坐标系
        last_cmd_.reset(); //< 上一次的控制指令

        memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));                 //< 轮电机的速度参考值 单位 rad/s
        memset(wheel_limited_speed_ref_, 0, sizeof(wheel_limited_speed_ref_)); //< 轮电机的速度参考值(限幅后) 单位 rad/s
        memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));             //< 轮电机的电流参考值 单位 A [-20, 20]

        chassis_composite_vel_ref_ = 0.0f; //< 底盘合成速度参考值 单位 m/s

        // motor fdb data 在 update 函数中更新
        is_all_motor_online_ = false;                                //< 所有轮电机是否都处于就绪状态
        is_any_motor_online_ = false;                                //< 任意电机是否处于就绪状态
        memset(wheel_speed_fdb_, 0, sizeof(wheel_speed_fdb_));       //< 轮速反馈数据
        memset(wheel_current_fdb_, 0, sizeof(wheel_current_fdb_));   //< 轮电流反馈数据

        theta_y2r_ = 0.0f; //< 图传坐标系绕 Z 轴到底盘坐标系的旋转角度，右手定则判定正反向，单位 rad
        spd_y2r_ = 0.0f;   //< 图传坐标系绕 Z 轴到底盘坐标系的旋转速度，右手定则判定正反向，单位 rad/s
        
        // cap fdb data 在 update 函数中更新
        is_high_spd_enabled_ = false; //< 是否开启了高速模式 （开启意味着从电容取电）
        cap_remaining_energy_ = 0.0f; //< 剩余电容能量百分比，单位 %

        resetPids();
    };

    void Chassis::resetDataOnDead()
    {
        cmd_.reset();
        last_cmd_.reset();

        memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
        memset(wheel_limited_speed_ref_, 0, sizeof(wheel_limited_speed_ref_));
        memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));

        resetPids();
    };

    void Chassis::resetDataOnResurrection()
    {
        cmd_.reset();
        last_cmd_.reset();

        memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
        memset(wheel_limited_speed_ref_, 0, sizeof(wheel_limited_speed_ref_));
        memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));

        resetPids();
    };

    void Chassis::resetDataOnStandby()
    {
        cmd_.reset();
        last_cmd_.reset();

        memset(wheel_speed_ref_, 0, sizeof(wheel_speed_ref_));
        memset(wheel_limited_speed_ref_, 0, sizeof(wheel_limited_speed_ref_));
        memset(wheel_current_ref_, 0, sizeof(wheel_current_ref_));

        resetPids();
    };

    void Chassis::resetPids()
    {
        for (size_t i = 0; i < kPidNum; i++)
        {
            HW_ASSERT(pid_ptr_[i] != nullptr, "pointer to chassis PID %d is nullptr", i);
            pid_ptr_[i]->reset();
        }
    };
#pragma endregion

#pragma region 通信数据设置函数

    void Chassis::setCommDataWheels(bool working_flag)
    {
        // 轮电机根据期望电流输入发送数据
        Motor *motor_ptr = nullptr;
        for (size_t i = 0; i < WheelMotorIdx::kWheelMotorNum; i++)
        {
            motor_ptr = wheel_motor_ptr_[i];
            HW_ASSERT(motor_ptr != nullptr, "pointer to motor %d is nullptr", i);
            if (!working_flag || motor_ptr->isOffline())
            {
                motor_ptr->setInput(0);
            }
            else
            {
                motor_ptr->setInput(wheel_current_ref_[i]);
                // motor_ptr->setInput(0);
            }
        }
    };

    // TODO: 超电组件库更新后还需要修改
    void Chassis::setCommDataCap(bool working_flag)
    {
        // 超电根据超电充电状态发送数据
        HW_ASSERT(cap_ptr_ != nullptr, "pointer to cap is nullptr", cap_ptr_);
        if (working_flag)
        {
            cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, rfr_data_.current_hp);
        }
        else
        {
            cap_ptr_->setRfrData(rfr_data_.pwr_buffer, rfr_data_.pwr_limit, 0);
        }
    };

#pragma endregion

#pragma region 注册函数

    void Chassis::registerIkSolver(ChassisIkSolver *ptr)
    {
        HW_ASSERT(ptr != nullptr, "pointer to IK solver is nullptr", ptr);
        ik_solver_ptr_ = ptr;
    };

    void Chassis::registerWheelMotor(Motor *ptr, uint8_t idx)
    {
        HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
        HW_ASSERT(idx >= 0 && idx < kWheelMotorNum, "index %d out of range", idx);
        wheel_motor_ptr_[idx] = ptr;
    };

    void Chassis::registerYawMotor(Motor *ptr)
    {
        HW_ASSERT(ptr != nullptr, "pointer to yaw motor is nullptr", ptr);
        yaw_motor_ptr_ = ptr;
    };

    void Chassis::registerCap(Cap *ptr)
    {
        HW_ASSERT(ptr != nullptr, "pointer to cap is nullptr", ptr);
        cap_ptr_ = ptr;
    };

    void Chassis::registerPwrLimiter(PwrLimiter *ptr)
    {
        HW_ASSERT(ptr != nullptr, "pointer to power limiter is nullptr", ptr);
        pwr_limiter_ptr_ = ptr;
    };

    void Chassis::registerPid(MultiNodesPid *ptr, uint8_t idx)
    {
        HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
        HW_ASSERT(idx >= 0 && idx < kPidNum, "index %d out of range", idx);
        pid_ptr_[idx] = ptr;
    };

#pragma endregion

#pragma region 工具函数

void Chassis::setCmdSmoothly(const Cmd &cmd, float beta)
{
    beta = hello_world::Bound(beta, 0.0f, 1.0f);
    cmd_ = cmd * beta + (1.0f - beta) * last_cmd_;
};

void Chassis::iksolver_forward(){
    actual_vx = actual_vy = 0.0f;
    for (int i = 0; i < 4; i++) {
        float alpha = ik_solver_ptr_->getWheel(i)->getAlpha();
        float push_dir = alpha + M_PI_2;
        float v_wheel = wheel_motor_ptr_[i]->vel() * 0.122f / 2.0f;
        float vx_c = v_wheel * cosf(push_dir);
        float vy_c = v_wheel * sinf(push_dir);
        float vx_g = vx_c * cosf(theta_y2r_) - vy_c * sinf(theta_y2r_);
        float vy_g = vx_c * sinf(theta_y2r_) + vy_c * cosf(theta_y2r_);
        actual_vx += vx_g;
        actual_vy += vy_g;
    }
    actual_vx /= 4.0f;
    actual_vy /= 4.0f;
}

#pragma endregion

}