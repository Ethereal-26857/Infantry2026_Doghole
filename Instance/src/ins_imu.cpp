#include "ins_imu.hpp"
#include "spi.h"

static hw_imu::ImuConfig kImuInitConfig = {
    .acc_threshold = 10.0f,
    .gyro_stationary_threshold = 0.1f,
    .sample_num = 0,
    .default_gyro_offset = {-0.00107172318, -0.00156000536, -0.0012},
    .samp_freq = 1000.0f,
    .kp = 1.0f,
    .ki = 0.0f,
    .rot_mat_flatten = {
        -1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 
        0.0, 1.0, 0.0,},
    .bmi088_hw_config = {
        .hspi = &hspi2,
        .acc_cs_port = GPIOC,
        .acc_cs_pin = GPIO_PIN_0,
        .gyro_cs_port = GPIOC,
        .gyro_cs_pin = GPIO_PIN_3,
    },
    .bmi088_config = {
        .acc_range = hw_imu::kBMI088AccRange3G,
        .acc_odr = hw_imu::kBMI088AccOdr1600,
        .acc_osr = hw_imu::kBMI088AccOsr4,
        .gyro_range = hw_imu::kBMI088GyroRange1000Dps,
        .gyro_odr_fbw = hw_imu::kBMI088GyroOdrFbw1000_116,
    },
};

hw_imu::Imu *GetImuIns(void)
{
    static hw_imu::Imu unique_imu = hw_imu::Imu(kImuInitConfig);
    return &unique_imu;
}