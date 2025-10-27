# INF2004 Robotic Car Project
# Team IS21 - Line Following Robot with Barcode Navigation

cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(robotic_car C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Set include directories
include_directories(include)

# ============================================================================
# COMPONENT TESTS - Individual driver testing
# ============================================================================

# Motor and Encoder Test
add_executable(test_motor_encoder
    tests/test_motor_encoder.c
    src/control/motor_control.c
    src/control/wheel_encoder.c
)
target_link_libraries(test_motor_encoder
    pico_stdlib
    hardware_pwm
    hardware_gpio
)
pico_enable_stdio_usb(test_motor_encoder 1)
pico_enable_stdio_uart(test_motor_encoder 0)
pico_add_extra_outputs(test_motor_encoder)

# IR Sensor Basic Test
add_executable(test_ir_basic
    tests/test_ir_basic.c
    src/sensors/ir_sensor.c
)
target_link_libraries(test_ir_basic
    pico_stdlib
    hardware_adc
)
pico_enable_stdio_usb(test_ir_basic 1)
pico_enable_stdio_uart(test_ir_basic 1)
pico_add_extra_outputs(test_ir_basic)

# Line Following Test
add_executable(test_line_following
    tests/test_line_following.c
    src/sensors/ir_sensor.c
    src/sensors/line_following.c
)
target_link_libraries(test_line_following
    pico_stdlib
    hardware_adc
)
pico_enable_stdio_usb(test_line_following 1)
pico_enable_stdio_uart(test_line_following 1)
pico_add_extra_outputs(test_line_following)

# Barcode Scanner Test
add_executable(test_barcode_scanner
    tests/test_barcode_scanner.c
    src/sensors/ir_sensor.c
    src/sensors/barcode_scanner.c
)
target_link_libraries(test_barcode_scanner
    pico_stdlib
    hardware_adc
)
pico_enable_stdio_usb(test_barcode_scanner 1)
pico_enable_stdio_uart(test_barcode_scanner 1)
pico_add_extra_outputs(test_barcode_scanner)

# Ultrasonic + Servo Test
add_executable(test_ultrasonic
    tests/test_ultrasonic.c
    src/sensors/ultrasonic.c
)
target_link_libraries(test_ultrasonic
    pico_stdlib
    hardware_gpio
    hardware_pwm
)
pico_enable_stdio_usb(test_ultrasonic 1)
pico_enable_stdio_uart(test_ultrasonic 0)
pico_add_extra_outputs(test_ultrasonic)

# ============================================================================
# INTEGRATION TESTS - Multi-component testing
# ============================================================================

# Motor + PID Control Integration Test
add_executable(test_motor_pid
    tests/test_motor_pid.c
    src/control/motor_control.c
    src/control/wheel_encoder.c
    src/control/pid_controller.c
)
target_link_libraries(test_motor_pid
    pico_stdlib
    hardware_pwm
    hardware_gpio
)
pico_enable_stdio_usb(test_motor_pid 1)
pico_enable_stdio_uart(test_motor_pid 0)
pico_add_extra_outputs(test_motor_pid)

# Line Following + Motor Integration
add_executable(test_line_motor
    tests/test_line_motor.c
    src/sensors/ir_sensor.c
    src/sensors/line_following.c
    src/control/motor_control.c
    src/control/wheel_encoder.c
    src/control/pid_controller.c
)
target_link_libraries(test_line_motor
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_gpio
)
pico_enable_stdio_usb(test_line_motor 1)
pico_enable_stdio_uart(test_line_motor 1)
pico_add_extra_outputs(test_line_motor)

# ============================================================================
# MAIN ROBOT APPLICATION
# ============================================================================

add_executable(robot_main
    main.c
    src/sensors/ir_sensor.c
    src/sensors/line_following.c
    src/sensors/barcode_scanner.c
    src/sensors/ultrasonic.c
    src/control/motor_control.c
    src/control/wheel_encoder.c
    src/control/pid_controller.c
    # Add IMU and WiFi when ready:
    # src/sensors/imu_sensor.c
    # src/communication/wifi_module.c
    # src/communication/mqtt_client.c
)

target_link_libraries(robot_main
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_gpio
    hardware_i2c
    # Add when WiFi is integrated:
    # pico_cyw43_arch_none
)

pico_enable_stdio_usb(robot_main 1)
pico_enable_stdio_uart(robot_main 1)
pico_add_extra_outputs(robot_main)