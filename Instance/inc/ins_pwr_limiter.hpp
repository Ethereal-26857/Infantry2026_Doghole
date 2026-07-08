#ifndef INS_PWR_LIMITER_HPP_
#define INS_PWR_LIMITER_HPP_

#include "power_limiter.hpp"

namespace hw_pwr_limiter = hello_world::power_limiter;

hw_pwr_limiter::PowerLimiter* GetPwrLimiterIns(void);

#endif  /* INS_PWR_LIMITER_HPP_ */