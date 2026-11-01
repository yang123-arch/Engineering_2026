add_library(common STATIC)

target_sources(common
PRIVATE
    chirp.c
    controller.c
    CRC8_CRC16.c
    Function.c
    kalman_filter.c
    PID.c
    QuaternionEKF.c
    RingBuffer.c
    user_lib.c
PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES
        chirp.h
        controller.h
        CRC8_CRC16.h
        Function.h
        kalman_filter.h
        PID.h
        QuaternionEKF.h
        RingBuffer.h
        user_lib.h
        Vofa.h
)

# controller.h / PID.c 依赖 DWT.h（hal）；其余继承 CubeMX / DSP
target_link_libraries(common PUBLIC
    stm32cubemx
    CMSISDSP
    hal
)

# hal/*.c 依赖 common 头文件（如 Vofa.h），在此回链形成静态库环（CMake 允许）
target_link_libraries(hal PRIVATE common)

target_include_directories(common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
