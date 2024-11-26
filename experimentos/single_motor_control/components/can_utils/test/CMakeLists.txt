idf_component_register(
    SRCS "test_can_utils.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity can_utils
)
