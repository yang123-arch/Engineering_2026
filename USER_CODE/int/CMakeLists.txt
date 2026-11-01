# OBJECT：HAL weak 回调必须作为 .obj 直接参与链接，
# 放进 STATIC 库时 --gc-sections / 已有 weak 定义会导致覆盖失败。
add_library(int OBJECT)

target_sources(int
PRIVATE
    can_callback.c
PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES
        
)

target_link_libraries(int PUBLIC
    stm32cubemx
    device
    srv
    common
)

target_include_directories(int PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/USER_CODE/application
)
