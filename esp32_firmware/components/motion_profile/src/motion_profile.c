#include "motion_profile.h"
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"

#define LOG_TAG "MOTION_PROFILE"

// A simplified S-curve (really more like a trapezoidal profile with constant jerk phases ignored).
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
    if (!trajectory || !num_points || time_step <= 0.0f)
    {
        ESP_LOGE(LOG_TAG, "Invalid arguments to motion_profile_generate_s_curve");
        return false;
    }

    // Distance to move and its sign
    float distance = end_pos - start_pos;
    int direction = (distance >= 0.0f) ? 1 : -1;
    float distance_abs = fabsf(distance);

    // Attempt a trapezoidal profile (ignoring jerk for simplicity).
    float accel_dist = (powf(max_vel, 2) - powf(start_vel, 2)) / (2.0f * max_acc);
    if (accel_dist < 0.0f)
        accel_dist = 0.0f;

    float decel_dist = (powf(max_vel, 2) - powf(end_vel, 2)) / (2.0f * max_acc);
    if (decel_dist < 0.0f)
        decel_dist = 0.0f;

    // Cruising distance is what's left after accelerating and decelerating
    float cruise_dist = distance_abs - (accel_dist + decel_dist);

    // If cruise_dist < 0, we never actually reach max_vel => solve for the real peak velocity
    if (cruise_dist < 0.0f)
    {
        float new_vmax_sq = distance_abs * max_acc + (powf(start_vel, 2) + powf(end_vel, 2)) / 2.0f;
        if (new_vmax_sq < 0.0f)
        {
            new_vmax_sq = 0.0f;
        }
        float v_peak = sqrtf(new_vmax_sq);

        // Recompute accel/dist so there's no cruise phase
        accel_dist = (powf(v_peak, 2) - powf(start_vel, 2)) / (2.0f * max_acc);
        if (accel_dist < 0.0f)
            accel_dist = 0.0f;

        decel_dist = (powf(v_peak, 2) - powf(end_vel, 2)) / (2.0f * max_acc);
        if (decel_dist < 0.0f)
            decel_dist = 0.0f;

        cruise_dist = 0.0f;
        max_vel = v_peak; // This is the actual peak velocity we can achieve
    }

    // Compute times for each phase
    float accel_time = (max_vel - start_vel) / max_acc;
    if (accel_time < 0.0f)
        accel_time = 0.0f;

    float decel_time = (max_vel - end_vel) / max_acc;
    if (decel_time < 0.0f)
        decel_time = 0.0f;

    float cruise_time = 0.0f;
    if (cruise_dist > 0.0f && max_vel > 0.0f)
    {
        cruise_time = cruise_dist / max_vel; // distance / speed
    }

    // Total duration
    float total_time = accel_time + cruise_time + decel_time;
    *num_points = (int)(total_time / time_step) + 1;
    if (*num_points < 2)
    {
        *num_points = 2;
    }

    motion_profile_point_t *traj = (motion_profile_point_t *)malloc(sizeof(motion_profile_point_t) * (*num_points));
    if (!traj)
    {
        ESP_LOGE(LOG_TAG, "Memory allocation failed in motion_profile_generate_s_curve");
        return false;
    }

    // Precompute distances for the phases in user-space
    float accel_phase_dist = (start_vel * accel_time) + 0.5f * max_acc * accel_time * accel_time;
    float cruise_phase_dist = cruise_dist; // already in absolute terms
    // decel_phase_dist not needed explicitly, we'll compute on the fly if needed

    float t = 0.0f;
    for (int i = 0; i < *num_points; i++)
    {
        float pos = 0.0f;
        float vel = 0.0f;
        float acc = 0.0f;

        if (t <= accel_time)
        {
            // Acceleration phase
            acc = max_acc * direction;
            vel = (start_vel * direction) + (acc * t);
            pos = start_pos + (start_vel * direction) * t + 0.5f * (max_acc * direction) * t * t;
        }
        else if (t <= (accel_time + cruise_time))
        {
            // Cruise phase
            float t_cruise = t - accel_time;
            acc = 0.0f;
            vel = max_vel * direction;

            // Distance covered in accel phase (in user space, applying direction)
            float accel_dist_signed = accel_phase_dist * direction;

            // Now add cruise segment
            pos = start_pos + accel_dist_signed + (vel * t_cruise);
        }
        else
        {
            // Deceleration phase
            float t_decel = t - accel_time - cruise_time;
            acc = -max_acc * direction;
            vel = (max_vel * direction) + (acc * t_decel);

            // Distance covered in accel + cruise phases
            float accel_dist_signed = accel_phase_dist * direction;
            float cruise_dist_signed = cruise_phase_dist * direction;
            float pos_decel_start = start_pos + accel_dist_signed + cruise_dist_signed;

            pos = pos_decel_start + (max_vel * direction) * t_decel + 0.5f * (acc)*t_decel * t_decel;
        }

        traj[i].position = pos;
        traj[i].velocity = vel;
        traj[i].acceleration = acc; // Not a true S-curve, but enough to illustrate trapezoid
        t += time_step;
    }

    *trajectory = traj;
    return true;
}

void motion_profile_free_trajectory(motion_profile_point_t *trajectory)
{
    if (trajectory)
    {
        free(trajectory);
    }
}
