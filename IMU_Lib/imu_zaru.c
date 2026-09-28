/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_zaru.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_zaru.h"

#include <math.h>

static imu_gyro_dps gyro_scale(imu_gyro_dps value, float scale) {
    return (imu_gyro_dps){
        .x_dps = value.x_dps * scale,
        .y_dps = value.y_dps * scale,
        .z_dps = value.z_dps * scale,
    };
}

static imu_gyro_dps gyro_subtract(imu_gyro_dps left, imu_gyro_dps right) {
    return (imu_gyro_dps){
        .x_dps = left.x_dps - right.x_dps,
        .y_dps = left.y_dps - right.y_dps,
        .z_dps = left.z_dps - right.z_dps,
    };
}

static imu_gyro_dps gyro_add(imu_gyro_dps left, imu_gyro_dps right) {
    return (imu_gyro_dps){
        .x_dps = left.x_dps + right.x_dps,
        .y_dps = left.y_dps + right.y_dps,
        .z_dps = left.z_dps + right.z_dps,
    };
}

static bool gyro_is_finite(imu_gyro_dps gyro_dps) {
    return isfinite(gyro_dps.x_dps) && isfinite(gyro_dps.y_dps) &&
           isfinite(gyro_dps.z_dps);
}

static void reset_window(imu_zaru_observer *observer) {
    observer->window_mean_dps = (imu_gyro_dps){0};
    observer->window_m2_dps2 = (imu_gyro_dps){0};
    observer->window_duration_s = 0.0f;
    observer->window_sample_count = 0u;
}

static float window_stddev(const imu_zaru_observer *observer) {
    if (observer->window_sample_count == 0u) {
        return 0.0f;
    }
    const float inverse_count = 1.0f /
                                (float)observer->window_sample_count;
    const float variance =
        (observer->window_m2_dps2.x_dps + observer->window_m2_dps2.y_dps +
         observer->window_m2_dps2.z_dps) * inverse_count;
    return sqrtf(fmaxf(0.0f, variance));
}

static void record_sample(imu_zaru_observer *observer,
                          imu_gyro_dps gyro_dps, float dt_s) {
    observer->window_sample_count++;
    const float inverse_count = 1.0f /
                                (float)observer->window_sample_count;
    const imu_gyro_dps delta = gyro_subtract(gyro_dps,
                                             observer->window_mean_dps);
    observer->window_mean_dps = gyro_add(
        observer->window_mean_dps, gyro_scale(delta, inverse_count));
    const imu_gyro_dps delta_after_mean =
        gyro_subtract(gyro_dps, observer->window_mean_dps);
    observer->window_m2_dps2 = gyro_add(
        observer->window_m2_dps2,
        (imu_gyro_dps){
            .x_dps = delta.x_dps * delta_after_mean.x_dps,
            .y_dps = delta.y_dps * delta_after_mean.y_dps,
            .z_dps = delta.z_dps * delta_after_mean.z_dps,
        });
    observer->window_duration_s += dt_s;
}

void imu_zaru_init(imu_zaru_observer *observer,
                   const imu_zaru_config *config) {
    *observer = (imu_zaru_observer){.config = *config};
}

void imu_zaru_set_bias(imu_zaru_observer *observer, imu_gyro_dps bias_dps) {
    observer->bias_dps = bias_dps;
    observer->last_window_bias_dps = (imu_gyro_dps){0};
    observer->last_window_stddev_dps = 0.0f;
    observer->last_window_duration_s = 0.0f;
    observer->last_window_sample_count = 0u;
    observer->cumulative_sum_x_dps_s = 0.0;
    observer->cumulative_sum_y_dps_s = 0.0;
    observer->cumulative_sum_z_dps_s = 0.0;
    observer->cumulative_sample_count = 0u;
    reset_window(observer);
    observer->bias_valid = gyro_is_finite(bias_dps);
}

imu_zaru_update_result imu_zaru_update(imu_zaru_observer *observer,
                                       const imu_zaru_input *input) {
    if (input->dt_s <= 0.0f || input->skipped_cycles != 0u ||
        !input->sample_valid || !input->confirmed_stationary ||
        !gyro_is_finite(input->gyro_dps)) {
        if (observer->window_sample_count != 0u) {
            observer->rejected_window_count++;
        }
        reset_window(observer);
        return IMU_ZARU_WINDOW_REJECTED;
    }

    record_sample(observer, input->gyro_dps, input->dt_s);
    const bool window_complete =
        observer->window_duration_s >= observer->config.minimum_window_s &&
        observer->window_sample_count >= observer->config.minimum_samples;
    if (!window_complete) {
        return IMU_ZARU_WINDOW_IN_PROGRESS;
    }

    observer->last_window_bias_dps = observer->window_mean_dps;
    observer->last_window_stddev_dps = window_stddev(observer);
    observer->last_window_duration_s = observer->window_duration_s;
    observer->last_window_sample_count = observer->window_sample_count;
    const bool window_is_quiet =
        observer->last_window_stddev_dps <= observer->config.max_window_stddev_dps;
    if (!window_is_quiet) {
        observer->rejected_window_count++;
        reset_window(observer);
        return IMU_ZARU_WINDOW_REJECTED;
    }

    float update_gain = observer->accepted_window_count < 4u
                            ? 0.75f
                            : observer->config.bias_update_gain;
    update_gain = fminf(1.0f, fmaxf(0.0f, update_gain));
    if (!observer->bias_valid) {
        observer->bias_dps = observer->last_window_bias_dps;
        observer->bias_valid = true;
    } else {
        observer->bias_dps = gyro_add(
            observer->bias_dps,
            gyro_scale(gyro_subtract(observer->last_window_bias_dps,
                                      observer->bias_dps),
                       update_gain));
    }
    observer->accepted_window_count++;
    reset_window(observer);
    return IMU_ZARU_WINDOW_ACCEPTED;
}
