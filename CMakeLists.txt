cmake_minimum_required(VERSION 3.0.0)
project(OmniDrone)

# ESP32 specific settings
set(CMAKE_C_COMPILER xtensa-esp32-elf-gcc)
set(CMAKE_CXX_COMPILER xtensa-esp32-elf-g++)

# ESP32 SDK paths
set(ESP32_SDK_PATH $ENV{IDF_PATH})

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/lib
    ${ESP32_SDK_PATH}/components/driver/include
    ${ESP32_SDK_PATH}/components/hal/include
    ${ESP32_SDK_PATH}/components/esp_common/include
    ${ESP32_SDK_PATH}/components/freertos/include
    ${ESP32_SDK_PATH}/components/esp_timer/include
    ${ESP32_SDK_PATH}/components/esp_system/include
    ${ESP32_SDK_PATH}/components/soc/include
    ${ESP32_SDK_PATH}/components/esp_rom/include
)

# Source files
file(GLOB SOURCES 
    "src/*.cpp"
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-unused-variable
    -Wno-unused-function
    -DESP32
    -DCORE_DEBUG_LEVEL=5
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    Wire
    SPI
    MPU6050
    RF24
    MAX17043
    VL53L0X
    driver
    hal
    esp_system
    freertos
    esp_timer
    esp_rom
)

# Add custom targets for flashing
add_custom_target(flash
    COMMAND esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 
            --before default_reset --after hard_reset 
            write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 
            0x1000 ${CMAKE_BINARY_DIR}/bootloader/bootloader.bin
            0x8000 ${CMAKE_BINARY_DIR}/partition_table/partition-table.bin
            0x10000 ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    DEPENDS ${PROJECT_NAME}
) 