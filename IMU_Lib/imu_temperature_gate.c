/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_temperature_gate.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_temperature_gate.h"

#include <math.h>

void imu_temperature_gate_init(imu_temperature_gate *gate,
                               const imu_temperature_gate_config *config) {
    *gate = (imu_temperature_gate){
        .config = *config,
    };
}

void imu_temperature_gate_advance(imu_temperature_gate *gate, float dt_s) {
    if (gate->has_temperature && dt_s > 0.0f) {
        gate->age_s += dt_s;
    }
}

void imu_temperature_gate_record(imu_temperature_gate *gate, bool valid,
                                 float temperature_degc) {
    if (!valid || !isfinite(temperature_degc)) {
        gate->last_sample_valid = false;
        return;
    }

    if (gate->has_temperature && gate->age_s > 0.0f) {
        gate->slope_degc_per_s =
            (temperature_degc - gate->temperature_degc) / gate->age_s;
        gate->has_slope = true;
    }
    gate->temperature_degc = temperature_degc;
    gate->age_s = 0.0f;
    gate->has_temperature = true;
    gate->last_sample_valid = true;
}

bool imu_temperature_gate_is_valid(const imu_temperature_gate *gate) {
    return gate->has_temperature && gate->has_slope &&
           gate->last_sample_valid &&
           gate->age_s <= gate->config.max_temperature_age_s &&
           fabsf(gate->slope_degc_per_s) <=
               gate->config.max_temperature_slope_degc_per_s;
}
