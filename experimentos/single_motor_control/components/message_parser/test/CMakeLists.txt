idf_component_register(
    SRCS "test_message_parser.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES unity message_parser
)
