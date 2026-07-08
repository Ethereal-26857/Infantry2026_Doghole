#include "ins_pwr_limiter.hpp"

const float moter_k = 268.0f / 17.0f / 14.0f;

const hw_pwr_limiter::PowerLimiterStaticParams kPwrLimiterStaticParams = {
    .wheel_motor_params =
    {
        .k1 = 0.23f * moter_k,
        .k2 = 0.106f,
        .k3 = 0.15f * moter_k,
        .kp = 1600.0f / 16384.0f * 20.0f,
        .out_limit = 20.0f,
        .motor_cnt = 4,
    },
    .p_bias = 5.3f,
};

hw_pwr_limiter::PowerLimiter unique_pwr_limiter = hw_pwr_limiter::PowerLimiter(kPwrLimiterStaticParams);

hw_pwr_limiter::PowerLimiter* GetPwrLimiterIns(void) { return &unique_pwr_limiter; }