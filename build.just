firmware-build := "cd penfw && cargo build --release --bin"

pengui:
    cargo build --release -p pengui

penproxy:
    cargo build --release -p penproxy

pensim:
    cargo build --release -p pensim

penfw:
    {{firmware-build}} penfw

blink:
    {{firmware-build}} blink

current_read:
    {{firmware-build}} current_read

hall_read:
    {{firmware-build}} hall_read

imu_read:
    {{firmware-build}} imu_read

motor_driver_integration_test:
    {{firmware-build}} motor_driver_integration_test

motor_open_loop:
    {{firmware-build}} motor_open_loop

motor_plus_current:
    {{firmware-build}} motor_plus_current

serial_echo:
    {{firmware-build}} serial_echo
