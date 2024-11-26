idf_component_register(
    SRCS "test_uart_utils.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity uart_utils
)
