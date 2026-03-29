firmware-dir := "target/xtensa-esp32-none-elf/release"

penfw:
    espflash flash --monitor {{ firmware-dir }}/penfw

blink:
    espflash flash --monitor {{ firmware-dir }}/blink

current_read:
    espflash flash --monitor {{ firmware-dir }}/current_read

hall_read:
    espflash flash --monitor {{ firmware-dir }}/hall_read

imu_read:
    espflash flash --monitor {{ firmware-dir }}/imu_read

motor_integration_test:
    espflash flash --monitor {{ firmware-dir }}/motor_integration_test

motor_open_loop:
    espflash flash --monitor {{ firmware-dir }}/motor_open_loop

serial_echo:
    espflash flash --monitor {{ firmware-dir }}/serial_echo
