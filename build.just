firmware-build := "cd penfw && cargo build --release --bin"

pengui:
    cargo build --release -p pengui

penproxy:
    cargo build --release -p penproxy

pensim:
    cargo build --release -p pensim

penfw:
    {{ firmware-build }} penfw

blink:
    {{ firmware-build }} blink

current_read:
    {{ firmware-build }} current_read

hall_read:
    {{ firmware-build }} hall_read

imu_read:
    {{ firmware-build }} imu_read

sensor_telemetry:
    {{ firmware-build }} sensor_telemetry

motor_board_test:
    {{ firmware-build }} motor_board_test

motor_open_loop:
    {{ firmware-build }} motor_open_loop

serial_echo:
    {{ firmware-build }} serial_echo
