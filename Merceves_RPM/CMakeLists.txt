# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Merceves_RPM C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Merceves_RPM Merceves_RPM.c )

pico_set_program_name(Merceves_RPM "Merceves_RPM")
pico_set_program_version(Merceves_RPM "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Merceves_RPM 0)
pico_enable_stdio_usb(Merceves_RPM 1)

# Add the standard library to the build
target_link_libraries(Merceves_RPM
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Merceves_RPM PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        hardware_spi
        hardware_adc
        hardware_pwm
        pico_multicore
        pico_time
        pico_stdlib
)

# Add any user requested libraries
target_link_libraries(Merceves_RPM 
        hardware_spi
        )

pico_add_extra_outputs(Merceves_RPM)

