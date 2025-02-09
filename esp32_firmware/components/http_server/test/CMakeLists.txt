idf_component_register(
    SRCS "test_http_server.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity http_server json
)
