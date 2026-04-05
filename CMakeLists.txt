cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Stm32Demo)

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
add_subdirectory(${CMAKE_SOURCE_DIR}/../../UserSDK ${CMAKE_BINARY_DIR}/UserSDK)

# Suppress vendor CMSIS/HAL unknown optimize attribute warning locally for Stm32Demo.
# Apply on stm32cubemx interface so generated object-library units inherit it.
target_compile_options(stm32cubemx INTERFACE
    $<$<COMPILE_LANGUAGE:C>:-Wno-unknown-attributes>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unknown-attributes>
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ../../UserSDK/bsp/stm32/Delay.cpp
        ../../UserSDK/bsp/stm32/Spi.cpp
        ../../UserSDK/bsp/stm32/Gpio.cpp
        ../../UserSDK/bsp/stm32/Exti.cpp
        User/App.cpp
        User/Entry.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Suppress vendor CMSIS/HAL unknown optimize attribute warning on application target.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wno-unknown-attributes>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unknown-attributes>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
        ExRobotSDK
    # Add user defined libraries
)
