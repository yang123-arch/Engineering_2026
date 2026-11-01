add_library(hal STATIC)

target_sources(hal
PRIVATE
    CAN_Driver.c
    DWT.c
    Flash.c
    PWM.c
    USART_Driver.c
    USB_Driver.c
PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES
        CAN_Driver.h
        DWT.h
        Flash.h
        PWM.h
        USART_Driver.h
        USB_Driver.h
)

# .c 里还用到 common（Vofa 等），在 common 的 CMakeLists 里再回链，避免循环创建顺序问题
target_link_libraries(hal PUBLIC
    stm32cubemx
    device
    
)

target_include_directories(hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
PRIVATE
    ${PROJECT_SOURCE_DIR}/USER_CODE/application
)
