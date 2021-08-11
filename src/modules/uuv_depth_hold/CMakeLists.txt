px4_add_module(
        MODULE modules__uuv_depth_hold
        MAIN uuv_depth_hold
        STACK_MAIN 1200
        COMPILE_FLAGS
        SRCS
        uuv_depth_hold.cpp
)