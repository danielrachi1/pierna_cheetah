idf_component_register(SRCS "src/http_server.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server spiffs message_parser motor_control_core motor_command_handlers robot_controller)
