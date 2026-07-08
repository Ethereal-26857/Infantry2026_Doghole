#include "ins_vision.hpp"

const hw_vision::Vision::Config kVisionConfig = {
    .default_blt_spd = 22.0f,     ///< 默认弹速，单位：m/s
    .blt_spd_filter_beta = 0.9f,  ///< 弹速滤波系数
    .hfov = 0.52f,                ///< 水平视场角，单位：rad
    .vfov = 0.52f,                ///< 垂直视场角，单位：rad
};

static hw_vision::Vision unique_vision = []() {
    hw_vision::Vision v(kVisionConfig);
    v.setOfflineThreshold(500);  ///< 视觉离线阈值，单位：ms
    return v;
}();

hw_vision::Vision *GetVisionIns(void)
{
    return &unique_vision;
}