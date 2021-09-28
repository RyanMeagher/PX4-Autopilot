px4_add_module(
        MODULE modules__depth_meter
        MAIN uuv_att_control
        STACK_MAIN 1200
        COMPILE_FLAGS
        SRCS
        DepthMeter.cpp
)