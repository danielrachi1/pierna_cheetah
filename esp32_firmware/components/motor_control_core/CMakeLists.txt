idf_component_register(SRCS "src/motor_control_core.c"
                       INCLUDE_DIRS "include"
                       REQUIRES motion_profile robot_controller)
