firmware-dir := "target/xtensa-esp32-none-elf/release"

penfw:
    espflash flash {{ firmware-dir }}/penfw

blink:
    espflash flash --monitor {{ firmware-dir }}/blink

current_read:
    espflash flash --monitor {{ firmware-dir }}/current_read

hall_read:
    espflash flash --monitor {{ firmware-dir }}/hall_read

imu_read:
    espflash flash --monitor {{ firmware-dir }}/imu_read

sensor_telemetry:
    espflash flash --monitor {{ firmware-dir }}/sensor_telemetry

motor_board_test:
    espflash flash --monitor {{ firmware-dir }}/motor_board_test

motor_open_loop:
    espflash flash --monitor {{ firmware-dir }}/motor_open_loop

motor_hall_lock:
    espflash flash --monitor {{ firmware-dir }}/motor_hall_lock

calibrate_motor:
    espflash flash --monitor {{ firmware-dir }}/calibrate_motor

wheel_torque_probe:
    espflash flash --monitor {{ firmware-dir }}/wheel_torque_probe

motor_max_spin:
    espflash flash --monitor {{ firmware-dir }}/motor_max_spin

serial_echo:
    espflash flash --monitor {{ firmware-dir }}/serial_echo

pendulum_telemetry:
    espflash flash --monitor {{ firmware-dir }}/pendulum_telemetry
