/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_temperature_gate.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Temperature evidence gate for bias refinement; it never models bias. */
#ifndef MIDDLEWARE_IMU_TEMPERATURE_GATE_H
#define MIDDLEWARE_IMU_TEMPERATURE_GATE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float max_temperature_age_s;
    float max_temperature_slope_degc_per_s;
} imu_temperature_gate_config;

typedef struct {
    imu_temperature_gate_config config;
    float temperature_degc;
    float age_s;
    float slope_degc_per_s;
    bool has_temperature;
    bool has_slope;
    bool last_sample_valid;
} imu_temperature_gate;

void imu_temperature_gate_init(imu_temperature_gate *gate,
                               const imu_temperature_gate_config *config);

/* Advance once for every task-cycle tick delta, including sensor errors. */
void imu_temperature_gate_advance(imu_temperature_gate *gate, float dt_s);

/* A failed read deliberately closes the gate instead of reusing stale data. */
void imu_temperature_gate_record(imu_temperature_gate *gate, bool valid,
                                 float temperature_degc);

bool imu_temperature_gate_is_valid(const imu_temperature_gate *gate);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_TEMPERATURE_GATE_H */
