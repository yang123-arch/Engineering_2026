add_library(device STATIC)

target_sources(device
PRIVATE
    error_core.c
    super_cup/SuperCap_Driver.c

    motor/Motor_DAMIAO_Driver.c
    motor/Motor_DJI_Driver.c
    
    motor/Motor_Driver.c
    motor/Motor_LINGKONG_Driver.c
    motor/motor_core.c
    motor/motor_init.c

    remote_control/Referee_Unpack.c
    remote_control/Remote_Control.c

    Buzzer/Buzzer_init.c

    Bmi088/BMI088driver.c
    Bmi088/BMI088Middleware.c

    Key/Key.c

    Led/LED.c

PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES
        error_core.h
        motor/Motor_DAMIAO_Driver.h
        motor/Motor_DMAIAO_Type.h
        motor/Motor_DJI_Driver.h
        
        motor/Motor_Driver.h
        motor/Motor_LINGKONG_Driver.h
        motor/motor_core.h
        motor/motor_init.h

        remote_control/Referee_Unpack.h
        remote_control/Remote_Control.h
        super_cup/SuperCap_Driver.h

        Buzzer/Buzzer_init.h

        Bmi088/BMI088driver.h
        Bmi088/BMI088reg.h
        Bmi088/BMI088Middleware.h

        Define.h

        Key/Key.h

        Led/LED.h

)

# 用 link 继承 HAL / DSP 的头文件与宏，不要把目标名写进 include_directories
target_link_libraries(device PUBLIC
    stm32cubemx
    CMSISDSP
    common
)

# 头文件在子目录里，外部用 #include "Motor_xxx.h" 等形式，需把子目录也暴露出去
target_include_directories(device PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_control
    ${CMAKE_CURRENT_SOURCE_DIR}/Bmi088
    ${CMAKE_CURRENT_SOURCE_DIR}/Buzzer
    ${CMAKE_CURRENT_SOURCE_DIR}/Key
    ${CMAKE_CURRENT_SOURCE_DIR}/Led
    ${CMAKE_CURRENT_SOURCE_DIR}/super_cup
)
