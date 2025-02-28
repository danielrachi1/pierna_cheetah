#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdbool.h>

/* Configuration Parameters (can be adjusted as needed) */
#define MP_DEFAULT_MAX_ACC 20.0f  ///< Default maximum acceleration (rad/s^2)
#define MP_DEFAULT_MAX_JERK 50.0f ///< Default maximum jerk (rad/s^3)
#define MP_TIME_STEP 0.01f        ///< Default time step for trajectory generation (seconds)

/**
 * @brief Structure representing a single point in a motion trajectory.
 *
 * - position: position in radians.
 * - velocity: velocity in radians per second.
 * - acceleration: acceleration in radians per second squared.
 */
typedef struct
{
    float position;
    float velocity;
    float acceleration;
} motion_profile_point_t;

/**
 * @brief Generates a simplified S-curve trajectory.
 *
 * This function produces a trajectory that moves from a starting position (start_pos) with an
 * initial velocity (start_vel) to an ending position (end_pos) with a final velocity (end_vel), while
 * respecting specified limits on maximum velocity (max_vel) and maximum acceleration (max_acc). The
 * max_jerk parameter is provided for interface consistency but is ignored in this simplified version.
 *
 * The trajectory is split into three phases:
 *   1. Acceleration phase: ramping up from start_vel to max_vel (or a lower peak if a triangular profile is needed),
 *   2. Optional cruise phase: moving at constant max_vel,
 *   3. Deceleration phase: ramping down from max_vel to end_vel.
 *
 * The function automatically computes a new peak velocity if the total distance is too short to reach
 * the provided max_vel. All internal calculations are performed using absolute values, and the sign
 * (direction) is applied only when assigning the final trajectory points.
 *
 * @param start_pos Starting position (radians).
 * @param start_vel Starting velocity (radians per second).
 * @param end_pos Ending position (radians).
 * @param end_vel Ending velocity (radians per second).
 * @param max_vel Maximum allowed velocity (radians per second).
 * @param max_acc Maximum allowed acceleration (radians per second squared).
 * @param max_jerk Maximum allowed jerk (radians per second cubed) (ignored in this simplified implementation).
 * @param time_step Sampling interval for the trajectory (seconds).
 * @param trajectory Output pointer which will point to an allocated array of trajectory points.
 *                   The caller must free this memory using motion_profile_free_trajectory().
 * @param num_points Output pointer for the number of trajectory points generated.
 * @return true if the trajectory is successfully generated, false otherwise.
 */
bool motion_profile_generate_s_curve(float start_pos,
                                     float start_vel,
                                     float end_pos,
                                     float end_vel,
                                     float max_vel,
                                     float max_acc,
                                     float max_jerk,
                                     float time_step,
                                     motion_profile_point_t **trajectory,
                                     int *num_points);

/**
 * @brief Frees the memory allocated for a trajectory.
 *
 * @param trajectory Pointer to the trajectory array to be freed.
 */
void motion_profile_free_trajectory(motion_profile_point_t *trajectory);

#endif // MOTION_PROFILE_H
