#include "unity.h"
#include "motion_profile.h"

/**
 * @file test_motion_profile.c
 * @brief Test cases for the motion_profile component.
 */

TEST_CASE("S-curve motion profile generation", "[motion_profile]")
{
    motion_profile_point_t *traj = NULL;
    int num_points = 0;
    bool result = motion_profile_generate_s_curve(0.0f, 0.0f, 10.0f, 0.0f,
                                                  MP_DEFAULT_MAX_VEL,
                                                  MP_DEFAULT_MAX_ACC,
                                                  MP_DEFAULT_MAX_JERK,
                                                  MP_TIME_STEP,
                                                  &traj,
                                                  &num_points);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_NOT_NULL(traj);
    TEST_ASSERT(num_points > 0);

    // Check some basic properties: start and end conditions
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, traj[0].position);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, traj[0].velocity);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, traj[num_points - 1].position); // close to end position
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, traj[num_points - 1].velocity);  // end velocity near zero

    // Check that acceleration does not exceed max_acc significantly
    float max_acc_observed = 0.0f;
    for (int i = 0; i < num_points; i++) {
        if (fabsf(traj[i].acceleration) > max_acc_observed) {
            max_acc_observed = fabsf(traj[i].acceleration);
        }
    }
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(MP_DEFAULT_MAX_ACC + 0.01f, max_acc_observed);

    motion_profile_free_trajectory(traj);
}
