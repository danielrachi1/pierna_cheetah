idf_component_register(SRCS "src/robot_controller.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash can_bus motor_control_core motor_command_handlers)
