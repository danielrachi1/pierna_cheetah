#include "motion_profile.h"
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"

#define LOG_TAG "MOTION_PROFILE"

// For simplicity, this is a simplified S-curve generator. A full S-curve involves solving
// multiple phases of motion (increasing acceleration, constant acceleration, decreasing acceleration,
// etc.) with jerk-limited transitions. Here we implement a simplified version that may need
// refinement for complex scenarios.

// This example code splits the trajectory into three phases:
// 1. Acceleration phase with limited jerk up to max_acc.
// 2. Constant velocity phase (if needed).
// 3. Deceleration phase with limited jerk down to end_vel.
// For a full S-curve, jerk transitions (jerk ramp-ups and ramp-downs) are considered. Here we'll do a
// simplified approach.
//
// NOTE: In a real system, you'd implement a more detailed s-curve planner. This code is a placeholder
// providing a conceptual example and should be adapted for actual s-curve computations.

bool motion_profile_generate_s_curve(float start_pos,
                                     float start_vel,
                                     float end_pos,
                                     float end_vel,
                                     float max_vel,
                                     float max_acc,
                                     float max_jerk,
                                     float time_step,
                                     motion_profile_point_t **trajectory,
                                     int *num_points)
{
    if (!trajectory || !num_points || time_step <= 0.0f) {
        ESP_LOGE(LOG_TAG, "Invalid arguments to motion_profile_generate_s_curve");
        return false;
    }

    // Simplified approach:
    // 1. Compute total distance.
    float distance = end_pos - start_pos;
    int direction = (distance >= 0.0f) ? 1 : -1;

    // For simplicity, ignore jerk in this minimal example and just produce a trapezoidal
    // or pseudo-s curve by ramping acceleration linearly. A real s-curve involves more complex math.
    // We'll just make a short jerk-based ramp: accelerate up, cruise, then decelerate.

    // Let's define a simple profile:
    // - Accelerate from start_vel to max_vel (if reachable) with max_acc.
    // - Cruise at max_vel.
    // - Decelerate to end_vel with max_acc.

    // Check if we can reach max velocity within the given distance.
    // A quick feasibility check:
    float accel_dist = (powf(max_vel, 2) - powf(start_vel, 2)) / (2.0f * max_acc);
    float decel_dist = (powf(max_vel, 2) - powf(end_vel, 2)) / (2.0f * max_acc);
    float cruise_dist = distance - (accel_dist + decel_dist);

    if (cruise_dist < 0.0f) {
        // Can't reach max_vel. Adjust max_vel downward so we end up with a "triangle" profile.
        // Solve for v_max given start_vel, end_vel, and distance:
        // distance = (v_max² - start_vel²)/(2*max_acc) + (v_max² - end_vel²)/(2*max_acc)
        // 2*distance*max_acc = 2*v_max² - start_vel² - end_vel²
        // v_max² = distance*max_acc + (start_vel² + end_vel²)/2
        float new_vmax_sq = direction * distance * max_acc + (powf(start_vel, 2) + powf(end_vel, 2)) / 2.0f;
        if (new_vmax_sq < 0.0f) new_vmax_sq = 0.0f; 
        float new_vmax = sqrtf(fabsf(new_vmax_sq));
        max_vel = new_vmax;
        accel_dist = (powf(max_vel, 2) - powf(start_vel, 2)) / (2.0f * max_acc);
        decel_dist = (powf(max_vel, 2) - powf(end_vel, 2)) / (2.0f * max_acc);
        cruise_dist = 0.0f; // no cruising
    }

    // Compute durations
    float accel_time = (max_vel - start_vel) / max_acc;
    float decel_time = (max_vel - end_vel) / max_acc;
    float cruise_time = (cruise_dist > 0.0f) ? (cruise_dist / max_vel) : 0.0f;

    // Total time
    float total_time = accel_time + cruise_time + decel_time;
    *num_points = (int)(total_time / time_step) + 1;
    if (*num_points < 2) {
        // If time steps are too large, fallback
        *num_points = 2;
    }

    motion_profile_point_t *traj = (motion_profile_point_t *)malloc(sizeof(motion_profile_point_t) * (*num_points));
    if (!traj) {
        ESP_LOGE(LOG_TAG, "Memory allocation failed in motion_profile_generate_s_curve");
        return false;
    }

    // Generate trajectory
    float t = 0.0f;
    for (int i = 0; i < *num_points; i++) {
        // Determine which phase we are in
        float pos, vel, acc;
        if (t <= accel_time) {
            // Acceleration phase
            acc = max_acc * direction; 
            vel = start_vel + acc * t;
            pos = start_pos + start_vel * t + 0.5f * acc * t * t;
        } else if (t <= (accel_time + cruise_time)) {
            // Cruise phase
            float t_cruise = t - accel_time;
            acc = 0.0f;
            vel = max_vel * direction;
            pos = start_pos + direction*accel_dist + vel * t_cruise;
        } else {
            // Deceleration phase
            float t_decel = t - accel_time - cruise_time;
            acc = -max_acc * direction;
            vel = max_vel * direction + acc * t_decel;
            float pos_decel_start = start_pos + direction*accel_dist + direction*cruise_dist;
            pos = pos_decel_start + max_vel * direction * t_decel + 0.5f * acc * t_decel * t_decel;
        }

        traj[i].position = pos;
        traj[i].velocity = vel;
        traj[i].acceleration = acc;

        t += time_step;
    }

    *trajectory = traj;
    return true;
}

void motion_profile_free_trajectory(motion_profile_point_t *trajectory)
{
    if (trajectory) {
        free(trajectory);
    }
}
