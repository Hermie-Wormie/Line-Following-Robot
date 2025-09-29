# project/CMakeLists.txt

# Set include directories so headers can be found
include_directories(include)

# Individual test executables for each component
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

# Line following test
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

# Barcode scanner test
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

# Combined IR sensor test (both line and barcode)
add_executable(test_ir_combined
    tests/test_ir_basic.c
    src/sensors/ir_sensor.c
    src/sensors/line_following.c
    src/sensors/barcode_scanner.c
)

target_link_libraries(test_ir_combined 
    pico_stdlib 
    hardware_adc
)

pico_enable_stdio_usb(test_ir_combined 1)
pico_enable_stdio_uart(test_ir_combined 1)
pico_add_extra_outputs(test_ir_combined)

# Main robot application (for future integration)
add_executable(robot_main
    main.c
    src/sensors/ir_sensor.c
    src/sensors/line_following.c
    src/sensors/barcode_scanner.c
    # Add other teammate modules here as they become available:
    # src/sensors/ultrasonic_sensor.c
    # src/control/motor_control.c
    # src/communication/mqtt_client.c
)

target_link_libraries(robot_main 
    pico_stdlib 
    hardware_adc
    # Add other libraries as needed:
    # hardware_pwm
    # hardware_i2c
)

pico_enable_stdio_usb(robot_main 1)
pico_enable_stdio_uart(robot_main 1)
pico_add_extra_outputs(robot_main)