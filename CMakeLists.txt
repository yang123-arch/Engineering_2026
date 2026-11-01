# 要求最低 CMake 版本
cmake_minimum_required(VERSION 3.22)

# ============================================================
# 交叉编译工具链配置（ARM 裸机嵌入式）
# ============================================================

# 目标系统为通用裸机（无操作系统）
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# 指定 ARM 交叉编译工具链路径
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

# 交叉编译时不尝试运行生成的可执行文件
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ============================================================
# 构建类型配置
# ============================================================

# 如果未指定构建类型，默认使用 Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 定义项目名称和支持的语言
project(g474vet6_cotroler_arm C CXX ASM)

# ============================================================
# 语言标准配置
# ============================================================

# C 标准：C11，并允许 GCC 扩展
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# C++ 标准：C++20（兼容 consteval 等新特性，适配嵌入式开发）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# ============================================================
# C++ 嵌入式编译选项：关闭异常和 RTTI
# ============================================================

# 嵌入式环境必须关闭异常和 RTTI，避免链接时缺少符号
add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
)

# 导出 compile_commands.json，供 clangd 等 IDE 工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 打印当前构建类型
message("Build type: " ${CMAKE_BUILD_TYPE})

# ============================================================
# CPU 架构选项（STM32G474 = Cortex-M4F）
# ============================================================

# 指定 CPU 为 Cortex-M4，Thumb 指令集
add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)

# 启用硬件浮点单元（FPU），使用硬浮点 ABI
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# ============================================================
# 通用编译选项
# ============================================================

# 每个函数/数据段独立存放，便于链接器垃圾回收（-gc-sections）
# -fno-common：禁止公共符号，避免重复定义
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# 汇编文件预处理支持（允许在 .s 文件中使用宏）
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# ============================================================
# 优化级别（根据构建类型选择）
# ============================================================

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g3)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g3)
endif ()

# ============================================================
# 创建可执行目标
# ============================================================

add_executable(${CMAKE_PROJECT_NAME})

# ============================================================
# 添加 STM32CubeMX 子项目
# ============================================================

# CubeMX 生成的代码包含 HAL 库、FreeRTOS、USB 库、启动文件等
add_subdirectory(cmake/stm32cubemx)
# Add CMSIS-DSP sources
add_subdirectory(Drivers/CMSIS-DSP)

# 用户代码层（hal / common / device / srv / application / INT）
# 必须在此加入，否则这些库既不参与编译，其头文件也不会进入
# compile_commands.json，导致 clangd/IDE 中用户头文件全部报错。
# 顺序要求：hal 必须先于 common 定义——common/CMakeLists.txt 里有
# target_link_libraries(hal PRIVATE common) 这条反向链接。
add_subdirectory(USER_CODE/hal)
add_subdirectory(USER_CODE/common)
add_subdirectory(USER_CODE/device)
add_subdirectory(USER_CODE/srv)
add_subdirectory(USER_CODE/application)
add_subdirectory(USER_CODE/int)

# ============================================================
# 链接目录配置
# ============================================================

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此处添加用户自定义库搜索路径
)

# ============================================================
# 用户源文件列表
# ============================================================

target_sources(${CMAKE_PROJECT_NAME} PRIVATE

)

# ============================================================
# 用户头文件路径
# ============================================================

# 注意：这里只补 CMSIS Core 头文件。
# CMSIS-DSP 头文件由 add_subdirectory(Drivers/CMSIS-DSP) 生成的 CMSISDSP 目标
# 通过 target_link_libraries 传递（见下方链接段），不要再写 ${CMSISDSP}/Include：
# CMSISDSP 是一个 target 名而非变量，展开为空会得到错误的绝对路径 "/Include"。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMSISCORE}/Include
)

# ============================================================
# 项目全局宏定义
# ============================================================

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 启用 HAL 库驱动
    USE_HAL_DRIVER
    # 目标 MCU 型号：STM32G474xx 系列
    STM32G474xx
    # Cortex-M4 内核标识
    CORE_CM4

    # ---------- CMSIS-DSP 宏定义（Cortex-M4F + DSP 指令集） ----------
    # 指定 Cortex-M4 架构使能对应的性能优化
    ARM_MATH_CM4
    # 启用 DSP 指令扩展（单周期 MAC、SIMD 等）
    ARM_MATH_DSP
    # 矩阵运算参数合法性检查（调试时建议开启）
    ARM_MATH_MATRIX_CHECK
    # 启用舍入模式提高计算精度
    ARM_MATH_ROUNDING
)

# ============================================================
# 修复 C++ 链接 libobjc 错误（ARM 嵌入式不需要）
# ============================================================

# 移除隐式链接的 libob（C 和 C++ 都需要）
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_LINK_LIBRARIES ob)

# ============================================================
# 链接器选项
# ============================================================

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    # CPU 架构选项
    -mcpu=cortex-m4 -mthumb -mthumb-interwork
    # --gc-sections：移除未引用的代码段和数据段
    # --print-memory-usage：链接完成后打印内存使用情况
    # -Map：生成 map 文件用于分析内存布局
    -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map
    # 链接 C++ 标准库（嵌入式 C++ 项目必须）
    -lstdc++
    -lsupc++
)

# ============================================================
# 链接 CubeMX 生成的 HAL / FreeRTOS / USB 库
# ============================================================

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    # 可在此处添加用户自定义库
    CMSISDSP

    # 最上层（无人被依赖）放最前：静态库解析按顺序单遍进行
    application
    int

    # 分层依赖：srv -> device -> hal / common
    srv
    device
    hal
    common
    m
)

# 强制先编完被依赖层再编上层，保证增量构建（-j 并行）不会用到过期头文件
add_dependencies(application srv device common hal)
add_dependencies(int srv device common hal)

# ============================================================
# 构建后处理：生成 HEX 和 BIN 文件
# ============================================================

# 定义输出文件路径
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

# 在编译完成后，使用 objcopy 生成 HEX 和 BIN 烧录文件
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
    COMMENT "Generating ${HEX_FILE} and ${BIN_FILE}"
)
