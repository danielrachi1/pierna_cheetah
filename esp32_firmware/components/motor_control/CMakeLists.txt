idf_component_register(SRCS "src/motor_control.c"
                       INCLUDE_DIRS "include"
                       REQUIRES motion_profile message_parser can_bus robot_controller)
