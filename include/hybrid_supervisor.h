#pragma once
#include <stdint.h>

enum HybridState {
    STATE_IDLE = 0,
    STATE_LIGHT_CRUISE,
    STATE_NORMAL_DRIVE,
    STATE_HIGH_LOAD,
    STATE_REGEN,
    STATE_SOC_RECOVERY,
    STATE_FAULT
};

struct HybridInputs {
    float pedal;          // 0.0 - 1.0
    float brake;          // 0.0 - 1.0
    float vehicle_speed;  // km/h
    float engine_rpm;
    float mg1_rpm;
    float mg2_rpm;
    float battery_soc;    // 0.0 - 1.0
};

struct HybridOutputs {
    float engine_rpm_target;  // preferred engine speed
    float mg1_torque_cmd;     // commanded torque for MG1
    float mg2_torque_cmd;     // commanded torque for MG2
};

class HybridSupervisor {
public:
    HybridSupervisor();
    void update(const HybridInputs& in, HybridOutputs& out);

private:
    HybridState state;
    float socAssistScale(float soc);
    float selectEngineRPM(float torque_ratio);
    float baseEAF(float vehicle_speed);
    float pedalEquilibrium(float vehicle_speed);
    void updateState(const HybridInputs& in, float torque_request);
};
