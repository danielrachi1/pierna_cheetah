#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file motion_profile.h
 * @brief S-curve motion profile generation for smooth motor control.
 *
 * This component provides functions to generate a series of setpoints for
 * position, velocity, and optionally acceleration, using an S-curve motion
 * profile. The S-curve profile ensures smooth starting and stopping of the
 * motor by controlling jerk (the derivative of acceleration), resulting in
 * reduced mechanical stress and smoother motion.
 *
 * The typical use-case is:
 * 1. Determine the start and end conditions of the motion (start position, end position,
 *    start velocity, end velocity).
 * 2. Define the maximum velocity, maximum acceleration, and maximum jerk allowed.
 * 3. Call the motion_profile_generate_s_curve() function to generate a trajectory
 *    consisting of discrete time steps. Each step includes position, velocity, and
 *    acceleration setpoints.
 * 4. The calling code can then send these setpoints to the motor controller at
 *    regular intervals to achieve smooth motion.
 *
 * @note This module is configurable at compile time via the #defines below. Adjust
 * these parameters as needed for your system.
 */

/* Configuration Parameters (can be adjusted as needed) */
#define MP_DEFAULT_MAX_VEL   5.0f   ///< Default maximum velocity (rad/s)
#define MP_DEFAULT_MAX_ACC   10.0f  ///< Default maximum acceleration (rad/s^2)
#define MP_DEFAULT_MAX_JERK  50.0f  ///< Default maximum jerk (rad/s^3)
#define MP_TIME_STEP         0.01f  ///< Default time step for trajectory generation (seconds)

/**
 * @brief A single trajectory point in the S-curve profile.
 */
typedef struct {
    float position;   ///< Position setpoint (radians)
    float velocity;   ///< Velocity setpoint (rad/s)
    float acceleration; ///< Acceleration setpoint (rad/s^2)
} motion_profile_point_t;

/**
 * @brief Generate an S-curve motion profile trajectory.
 *
 * This function generates a series of setpoints (position, velocity, acceleration) for
 * an S-curve motion from a given start state to an end state. The profile adheres to
 * the specified maximum velocity, acceleration, and jerk constraints.
 *
 * @param start_pos     Starting position (radians).
 * @param start_vel     Starting velocity (rad/s).
 * @param end_pos       Target ending position (radians).
 * @param end_vel       Target ending velocity (rad/s).
 * @param max_vel       Maximum allowed velocity (rad/s).
 * @param max_acc       Maximum allowed acceleration (rad/s^2).
 * @param max_jerk      Maximum allowed jerk (rad/s^3).
 * @param time_step     Time increment for each trajectory point (seconds).
 * @param trajectory    Pointer to a pointer. On success, this function allocates memory
 *                      and sets *trajectory to point to the newly allocated array of
 *                      motion_profile_point_t. The caller is responsible for freeing
 *                      this memory when done.
 * @param num_points    Pointer to an integer that will be set to the number of points
 *                      generated in the trajectory.
 * @return true if the trajectory was successfully generated, false otherwise.
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
 * @brief Free the memory allocated for a trajectory.
 *
 * @param trajectory Pointer to the trajectory array allocated by motion_profile_generate_s_curve().
 */
void motion_profile_free_trajectory(motion_profile_point_t *trajectory);

#endif // MOTION_PROFILE_H
