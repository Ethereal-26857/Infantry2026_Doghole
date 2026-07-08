#include "ins_chassis_iksolver.hpp"

static const hw_chassis_iksolver::PosVec kCenterPos = hw_chassis_iksolver::PosVec(0, 0);

hw_chassis_iksolver::ChassisIkSolver unique_chassis_iksolver = hw_chassis_iksolver::ChassisIkSolver(kCenterPos);
bool is_chassis_iksolver_inited = false;

const float kWheelRadius = 154 * 0.001 / 2;  ///< 轮子半径 [m]
const float kWheelGamma = PI / 4;            ///< Swidish 轮的 gamma 角
const float kFrontWheelBase = 0.3441;               ///< 前左右轮距 [m]
const float kRearWheelBase = 0.4023;                ///< 后左右轮距 [m]
const float kWheelTrack = 0.35371;              ///< 前后轮距 [m]


typedef hw_chassis_iksolver::SteeredStandardWheel::OptMask SteerOptMask;

// const float kSteerOptMak = SteerOptMask::kOptMaskMinThetaVelDelta | SteerOptMask::kOptMaskUseThetaVelFdb | SteerOptMask::kOptMaskCosRotSpd;  ///< 舵机最小转角，单位：rad

static void InitChassisIkSolver(void)
{
    hw_chassis_iksolver::ChassisIkSolver& iksolver = unique_chassis_iksolver;
  if (iksolver.size() == 0) { 
    hw_chassis_iksolver::WheelParams ik_solver_params[4] = {0};
    // 左前轮
    ik_solver_params[0].theta_vel_fdb = 0;
    ik_solver_params[0].gamma = -kWheelGamma;
    ik_solver_params[0].radius = kWheelRadius;
    ik_solver_params[0].wheel_pos = hw_chassis_iksolver::PosVec(kWheelTrack / 2, kFrontWheelBase / 2);
    // 左后轮
    ik_solver_params[1].theta_vel_fdb = 0;
    ik_solver_params[1].gamma = kWheelGamma;
    ik_solver_params[1].radius = kWheelRadius;
    ik_solver_params[1].wheel_pos = hw_chassis_iksolver::PosVec(-kWheelTrack / 2, kRearWheelBase / 2);
    // 右后轮
    ik_solver_params[2].theta_vel_fdb = 0;
    ik_solver_params[2].gamma = -kWheelGamma;
    ik_solver_params[2].radius = kWheelRadius;
    ik_solver_params[2].wheel_pos = hw_chassis_iksolver::PosVec(-kWheelTrack / 2, -kRearWheelBase / 2);

    // 右前轮
    ik_solver_params[3].theta_vel_fdb = 0;
    ik_solver_params[3].gamma = kWheelGamma;
    ik_solver_params[3].radius = kWheelRadius;
    ik_solver_params[3].wheel_pos = hw_chassis_iksolver::PosVec(kWheelTrack / 2, -kFrontWheelBase / 2);

    for (int i = 0; i < 4; i++) {
      iksolver.append(hw_chassis_iksolver::WheelType::kSwedish, ik_solver_params[i]);
    }
  }
}

hw_chassis_iksolver::ChassisIkSolver* GetChassisIkSolverIns(void)
{
    if(!is_chassis_iksolver_inited)
    {
        InitChassisIkSolver();
        is_chassis_iksolver_inited = true;
    }
    return &unique_chassis_iksolver;
}