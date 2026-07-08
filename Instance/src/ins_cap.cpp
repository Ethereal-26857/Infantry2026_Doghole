#include "ins_cap.hpp"

typedef hello_world::cap::SuperCap Cap;

static const Cap::Config kCapConfig = {
    .max_charge_volt = 26.0f,
    .min_valid_volt = 16.0f,
    .auto_disable_power = 20.0f,
    .min_enable_power = 40.0f,
    .pwr_filter_beta = 0.5f,
};

static Cap unique_cap = Cap(Cap::Version::kVer2024);

Cap* GetCapIns(void) { 
    unique_cap.setConfig(kCapConfig);
    return &unique_cap; 
}