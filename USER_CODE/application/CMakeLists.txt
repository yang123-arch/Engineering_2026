add_library(application OBJECT)

target_sources(application
PRIVATE
    Buzzer_Task.c
    error_solve_Task.c
    Power_Limit.c
    RoboControl.c
    Key_LED.c
    INS.c
    Gimbal.c
    Chassis.c
    Communicate.c
    Aim.c
    Shoot.c
    UI/UI_Task.c
    UI/ui_g.c
    UI/ui_interface.c
    UI/ui_stdio.c
    ../common/Compensation.c
PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES
        freertos.h
        Communicate.h
        UI/UI_Task.h
        UI/ui.h
        UI/ui_g.h
        UI/ui_interface.h
        UI/ui_types.h
)

target_link_libraries(application PUBLIC
    stm32cubemx
    common
    device
    srv
    hal
)

target_include_directories(application PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/UI
)
