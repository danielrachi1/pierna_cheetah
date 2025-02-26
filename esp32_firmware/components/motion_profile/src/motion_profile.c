#include "motion_profile.h"
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"

#define LOG_TAG "MOTION_PROFILE"

// A simplified S-curve (really more like trapezoid with quick jerk transitions).
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

    // distance to move
    float distance = end_pos - start_pos;
    int direction = (distance >= 0.0f) ? 1 : -1;

    // Attempt a trapezoidal profile ignoring jerk for simplicity.
    float accel_dist = (powf(max_vel, 2) - powf(start_vel, 2)) / (2.0f * max_acc);
    float decel_dist = (powf(max_vel, 2) - powf(end_vel, 2)) / (2.0f * max_acc);
    float cruise_dist = distance - (accel_dist + decel_dist);

    float v_peak = max_vel;
    if (cruise_dist < 0.0f)
    {
        // can't reach max_vel, solve for new v_peak
        float new_vmax_sq = direction * distance * max_acc + (powf(start_vel, 2) + powf(end_vel, 2)) / 2.0f;
        if (new_vmax_sq < 0.0f)
            new_vmax_sq = 0.0f;
        v_peak = sqrtf(fabsf(new_vmax_sq));
        accel_dist = (powf(v_peak, 2) - powf(start_vel, 2)) / (2.0f * max_acc);
        decel_dist = (powf(v_peak, 2) - powf(end_vel, 2)) / (2.0f * max_acc);
        cruise_dist = 0.0f;
    }

    float accel_time = (v_peak - start_vel) / max_acc;
    float decel_time = (v_peak - end_vel) / max_acc;
    float cruise_time = (cruise_dist > 0.0f) ? (cruise_dist / v_peak) : 0.0f;

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

    float t = 0.0f;
    for (int i = 0; i < *num_points; i++)
    {
        float pos, vel, acc;
        if (t <= accel_time)
        {
            // Accel phase
            acc = max_acc * direction;
            vel = start_vel + acc * t;
            pos = start_pos + start_vel * t + 0.5f * acc * t * t;
        }
        else if (t <= (accel_time + cruise_time))
        {
            // Cruise
            float t_cruise = t - accel_time;
            acc = 0.0f;
            vel = v_peak * direction;
            pos = start_pos + direction * accel_dist + vel * t_cruise;
        }
        else
        {
            // Decel
            float t_decel = t - accel_time - cruise_time;
            acc = -max_acc * direction;
            vel = v_peak * direction + acc * t_decel;
            float pos_decel_start = start_pos + direction * accel_dist + direction * cruise_dist;
            pos = pos_decel_start + v_peak * direction * t_decel + 0.5f * acc * t_decel * t_decel;
        }

        traj[i].position = pos;
        traj[i].velocity = vel;
        traj[i].acceleration = 0.0f; // For simplicity, not fully correct for real S-curve

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
