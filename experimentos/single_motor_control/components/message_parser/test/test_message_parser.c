#include "unity.h"
#include "message_parser.h"

TEST_CASE("Parse command with valid input", "[message_parser]")
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

TEST_CASE("Parse command with missing parameters", "[message_parser]")
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

TEST_CASE("Parse command with invalid format", "[message_parser]")
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

TEST_CASE("float_to_uint standard values", "[message_parser]")
{
    float x = 0.0f;
    float x_min = -10.0f;
    float x_max = 10.0f;
    uint8_t bits = 8;
    uint16_t expected = 127; // Midpoint of 0-255

    uint16_t result = float_to_uint(x, x_min, x_max, bits);
    TEST_ASSERT_EQUAL_UINT16(expected, result);
}

TEST_CASE("float_to_uint edge values", "[message_parser]")
{
    float x_min = -10.0f;
    float x_max = 10.0f;
    uint8_t bits = 8;

    // Test minimum value
    float x_below = x_min;
    uint16_t min_result = float_to_uint(x_below, x_min, x_max, bits);
    TEST_ASSERT_EQUAL_UINT16(0, min_result);

    // Test maximum value
    float x_above = x_max;
    uint16_t max_result = float_to_uint(x_above, x_min, x_max, bits);
    TEST_ASSERT_EQUAL_UINT16(255, max_result);
}

TEST_CASE("uint_to_float standard values", "[message_parser]")
{
    uint16_t x_int = 127;
    float x_min = -10.0f;
    float x_max = 10.0f;
    uint8_t bits = 8;

    float expected = -0.0392156862745097f;

    float result = uint_to_float(x_int, x_min, x_max, bits);
    TEST_ASSERT_EQUAL_FLOAT(expected, result);
}

TEST_CASE("uint_to_float edge values", "[message_parser]")
{
    float x_min = -10.0f;
    float x_max = 10.0f;
    uint8_t bits = 8;

    // Test minimum value
    uint16_t min_int = 0;
    float min_result = uint_to_float(min_int, x_min, x_max, bits);
    TEST_ASSERT_EQUAL_FLOAT(x_min, min_result);

    // Test maximum value
    uint16_t max_int = (1 << bits) - 1; // 255
    float max_result = uint_to_float(max_int, x_min, x_max, bits);
    TEST_ASSERT_EQUAL_FLOAT(x_max, max_result);
}

TEST_CASE("pack_cmd", "[message_parser]")
{
    float p_des = 0.0f;
    float v_des = 0.0f;
    float kp = 250.0f;
    float kd = 2.5f;
    float t_ff = 0.0f;
    uint8_t msg_data[CAN_CMD_LENGTH] = {0};

    pack_cmd(p_des, v_des, kp, kd, t_ff, msg_data);

    /*
     * Calculations, using the spreadsheet. decimals are truncated when converted to integer:
     *
     * p_des = 0.0f mapped to 16 bits:
     * float_to_uint(0.0f, -12.5f, 12.5f, 16) =  32767.5 = 32767 = 0x7FFF
     *
     * v_des = 0.0f mapped to 12 bits:
     * float_to_uint(0.0f, -65.0f, 65.0f, 12) = 2047.5 = 2047 = 0x7FF
     *
     * kp = 250.0f mapped to 12 bits:
     * float_to_uint(250.0f, 0.0f, 500.0f, 12) = 2047.5 = 2047 = 0x7FF
     *
     * kd = 2.5f mapped to 12 bits:
     * float_to_uint(2.5f, 0.0f, 5.0f, 12) = 2047.5 = 2047 = 0x7FF
     *
     * t_ff = 0.0f mapped to 12 bits:
     * float_to_uint(0.0f, -18.0f, 18.0f, 12) = 2047.5 = 2047 = 0x7FF
     */

    // Expected packed data based on truncation
    uint8_t expected_msg_data[CAN_CMD_LENGTH] = {
        0x7F, // Byte 0: High byte of p_int (0x7FFF >> 8 = 0x7F)
        0xFF, // Byte 1: Low byte of p_int (0x7FFF & 0xFF = 0xFF)
        0x7F, // Byte 2: High 8 bits of v_int (0x7FF >> 4 = 0x7F)
        0xF7, // Byte 3: Lower 4 bits of v_int (0x7FF & 0xF = 0xF) << 4 | High 4 bits of kp_int (0x7FF >> 8 & 0xF = 0x7)
        0xFF, // Byte 4: Lower 8 bits of kp_int (0x7FF & 0xFF = 0xFF)
        0x7F, // Byte 5: High 8 bits of kd_int (0x7FF >> 4 = 0x7F)
        0xF7, // Byte 6: Lower 4 bits of kd_int (0x7FF & 0xF = 0xF) << 4 | High 4 bits of t_int (0x7FF >> 8 & 0xF = 0x7)
        0xFF  // Byte 7: Lower 8 bits of t_int (0x7FF & 0xFF = 0xFF)
    };

    // Verify each byte in the packed CAN message
    for (int i = 0; i < CAN_CMD_LENGTH; i++)
    {
        TEST_ASSERT_EQUAL_HEX(expected_msg_data[i], msg_data[i]);
    }
}

TEST_CASE("unpack_reply", "[message_parser]")
{
    /*
     * Simulated CAN reply message: (Calculated on spreadsheet)
     * - p_int = 0x7FFF (32767) mapped to float: -0.000190737773708705
     *
     * - v_int = 0x7FF (2047) mapped to float: -0.0158730158730123
     *
     * - t_int = 0x7FF (2047) mapped to float: -0.0158730158730123
     */

    uint8_t msg_data[CAN_REPLY_LENGTH] = {0x7F, 0xFF, 0x7F, 0xF7, 0xFF};
    motor_reply_t reply = {0};

    unpack_reply(msg_data, &reply);

    float expected_position = -0.000190737773708705f;
    float expected_velocity = -0.0158730158730123f;
    float expected_torque = -0.0158730158730123f;

    // Values will be on order of magnitude 1E1. A 1% tolerance is taken.
    float tolerance = 1.0E-1f;

    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected_position, reply.position);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected_velocity, reply.velocity);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected_torque, reply.torque);
}