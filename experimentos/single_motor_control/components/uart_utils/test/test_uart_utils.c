#include "unity.h"
#include "uart_utils.h"

TEST_CASE("Parse command with valid input", "[uart_utils]")
{
    const char *input = "P1.2345_V2.3456_KP3.4567_KD4.5678_TFF5.6789";
    motor_command_t command = {0};
    parse_command(input, &command);

    TEST_ASSERT_EQUAL_FLOAT(1.2345f, command.p_des);
    TEST_ASSERT_EQUAL_FLOAT(2.3456f, command.v_des);
    TEST_ASSERT_EQUAL_FLOAT(3.4567f, command.kp);
    TEST_ASSERT_EQUAL_FLOAT(4.5678f, command.kd);
    TEST_ASSERT_EQUAL_FLOAT(5.6789f, command.t_ff);
}

TEST_CASE("Parse command with missing parameters", "[uart_utils]")
{
    const char *input = "P1.0_V2.0_KD0.5";
    motor_command_t command = {0};
    parse_command(input, &command);

    TEST_ASSERT_EQUAL_FLOAT(1.0f, command.p_des);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, command.v_des);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.kp); // Default or zero
    TEST_ASSERT_EQUAL_FLOAT(0.5f, command.kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.t_ff); // Default or zero
}

TEST_CASE("Parse command with invalid format", "[uart_utils]")
{
    const char *input = "Invalid_Input_String";
    motor_command_t command = {0};
    parse_command(input, &command);

    // All values should remain at their initialized state (zero)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.p_des);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.v_des);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.t_ff);
}