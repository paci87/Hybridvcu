#include "hybrid_supervisor.h"
#include <algorithm>
#include <math.h>

// Engine params
static constexpr float ENGINE_MAX_TORQUE = 339.0f;  // Nm (~250 lb-ft)
static constexpr float MG2_MAX_TORQUE    = 270.0f;
static constexpr float MG2_MAX_REGEN     = -200.0f;

// Battery power limits (kW)
static constexpr float BATTERY_DISCHARGE_LIMIT = 60.0f;
static constexpr float BATTERY_CHARGE_LIMIT    = -40.0f;

// Planetary gear ratios
static constexpr float Ns = 30.0f;
static constexpr float Nr = 78.0f;

HybridSupervisor::HybridSupervisor() {
    state = STATE_IDLE;
}

float HybridSupervisor::socAssistScale(float soc) {
    if      (soc > 0.70f) return 1.2f;
    else if (soc > 0.55f) return 1.0f;
    else if (soc > 0.45f) return 0.7f;
    else if (soc > 0.35f) return 0.4f;
    else if (soc > 0.25f) return 0.1f;
    return 0.0f;
}

float HybridSupervisor::baseEAF(float speed) {
    if (speed < 10.0f) return 0.5f;
    if (speed < 25.0f) return 0.4f;
    if (speed < 40.0f) return 0.3f;
    return 0.2f;
}

float HybridSupervisor::selectEngineRPM(float torque_ratio) {
    if      (torque_ratio < 0.05f) return 800.0f;
    else if (torque_ratio < 0.15f) return 1800.0f;
    else if (torque_ratio < 0.35f) return 2200.0f;
    else if (torque_ratio < 0.65f) return 2800.0f;
    else if (torque_ratio < 0.85f) return 3600.0f;
    return 5200.0f;
}

float HybridSupervisor::pedalEquilibrium(float speed) {
    if (speed < 5.0f) return 0.15f;
    if (speed < 20.0f) return 0.20f;
    return 0.25f;
}

void HybridSupervisor::updateState(const HybridInputs& in, float torque_request) {
    if      (in.battery_soc < 0.25f)      state = STATE_SOC_RECOVERY;
    else if (torque_request < MG2_MAX_REGEN) state = STATE_REGEN;
    else if (fabs(torque_request) < 5.0f)    state = STATE_IDLE;
    else if (torque_request < ENGINE_MAX_TORQUE*0.3f) state = STATE_LIGHT_CRUISE;
    else if (torque_request < ENGINE_MAX_TORQUE*0.75f) state = STATE_NORMAL_DRIVE;
    else state = STATE_HIGH_LOAD;
}

void HybridSupervisor::update(const HybridInputs& in, HybridOutputs& out) {
    float pedal_eq = pedalEquilibrium(in.vehicle_speed);
    float deltaPedal = in.pedal - pedal_eq;
    float torque_request = deltaPedal * ENGINE_MAX_TORQUE;

    updateState(in, torque_request);

    float eaf = baseEAF(in.vehicle_speed) * socAssistScale(in.battery_soc);

    float mg2_torque  = 0.0f;
    float engine_torque = 0.0f;

    if (state == STATE_REGEN) {
        mg2_torque = std::max(deltaPedal * ENGINE_MAX_TORQUE, MG2_MAX_REGEN);
        engine_torque = 0.0f;
        out.engine_rpm_target = 900.0f;
    }
    else if (state == STATE_SOC_RECOVERY) {
        eaf = 0.0f;
        engine_torque = std::max(torque_request, 0.0f);
        out.engine_rpm_target = 2500.0f;
    }
    else {
        mg2_torque = torque_request * eaf;
        mg2_torque = std::clamp(mg2_torque, MG2_MAX_REGEN, MG2_MAX_TORQUE);
        engine_torque = torque_request - mg2_torque;

        float torque_ratio = std::clamp(engine_torque/ENGINE_MAX_TORQUE, 0.0f, 1.0f);

        out.engine_rpm_target = selectEngineRPM(torque_ratio);
    }

    float mg2_power = (mg2_torque * in.mg2_rpm * 2.0f * M_PI)/60000.0f;
    if (mg2_power > BATTERY_DISCHARGE_LIMIT)
        mg2_torque *= 0.8f;
    if (mg2_power < BATTERY_CHARGE_LIMIT)
        mg2_torque *= 0.7f;

    out.mg2_torque_cmd = mg2_torque;
    out.mg1_torque_cmd = 0.0f;
}
