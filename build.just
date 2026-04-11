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

motor_hall_lock:
    {{ firmware-build }} motor_hall_lock

calibrate_motor:
    {{ firmware-build }} calibrate_motor

wheel_torque_probe:
    {{ firmware-build }} wheel_torque_probe

motor_max_spin:
    {{ firmware-build }} motor_max_spin

serial_echo:
    {{ firmware-build }} serial_echo

pendulum_telemetry:
    {{ firmware-build }} pendulum_telemetry
