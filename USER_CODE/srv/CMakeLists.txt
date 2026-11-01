add_library(srv STATIC)

target_sources(srv
PRIVATE
    chirp_pid_test.c
    joint_calc.c
PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES
        chirp_pid_test.h
        joint_calc.h
)

# 中间层：PID、扫频来自 common，电机设备来自 device，USB 发送来自 hal
target_link_libraries(srv PUBLIC
    stm32cubemx
    common
    device
    hal
)

target_include_directories(srv PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
