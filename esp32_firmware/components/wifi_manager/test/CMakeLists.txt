idf_component_register(
    SRCS "test_wifi_manager.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity wifi_manager json
)
