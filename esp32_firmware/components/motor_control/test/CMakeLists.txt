idf_component_register(
    SRCS "test_motor_control.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity motor_control
)
