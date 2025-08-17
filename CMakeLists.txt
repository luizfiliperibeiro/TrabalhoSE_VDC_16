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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(medidor_luz_cor C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories( ${CMAKE_SOURCE_DIR}/lib ) # Inclui os files .h na pasta lib

# Add executable. Default name is the project name, version 0.1

add_executable(medidor_luz_cor
        main.c
        #main2.c
        #gy_33_disp.c
        lib/ssd1306.c
        #ilumi_bh1750.c
        lib/bh1750_light_sensor.c
        lib/ws2812.c
        )

pico_set_program_name(medidor_luz_cor "medidor_luz_cor")
pico_set_program_version(medidor_luz_cor "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(medidor_luz_cor 1)
pico_enable_stdio_usb(medidor_luz_cor 1)

# Add the standard library to the build
target_link_libraries(medidor_luz_cor
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_pio
        hardware_clocks
        hardware_gpio
        hardware_adc
        hardware_irq
        )

# Add the standard include files to the build
target_include_directories(medidor_luz_cor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Adiciona o .pio ao projeto
pico_generate_pio_header(medidor_luz_cor ${CMAKE_CURRENT_LIST_DIR}/lib/ws2812.pio)

# Add any user requested libraries
target_link_libraries(medidor_luz_cor
        )

pico_add_extra_outputs(medidor_luz_cor)