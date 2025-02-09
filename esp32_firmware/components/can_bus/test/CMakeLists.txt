idf_component_register(
    SRCS "test_can_bus.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity can_bus json
)
