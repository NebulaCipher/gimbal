cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Tasks/main_task.cpp
    Tasks/imu_task.cpp
    Resources/Src/HW_can.cpp
    Resources/Src/PID.cpp
    Resources/Src/dm4310_drv.cpp
    Resources/Src/dm4340_drv.cpp
    components/imu/src/BMI088.cpp
    components/tick/tick.cpp
    components/ahrs/src/ahrs_base.cpp
    components/ahrs/src/mahony.cpp
    components/CMSIS-DSP/Source/BasicMathFunctions/arm_add_f32.c
    components/CMSIS-DSP/Source/BasicMathFunctions/arm_scale_f32.c
    components/CMSIS-DSP/Source/FastMathFunctions/arm_atan2_f32.c
    components/CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    components/CMSIS-DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c
    components/CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Tasks
    Resources/Inc
    components/imu/inc
    components/imu/prv
    components/tick
    components/ahrs/inc
    components/CMSIS-DSP/Include
    components/CMSIS-DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    VERBATIM
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
