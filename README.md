# reaction wheel pendulum

## the physics

The spinning mass (reaction wheel) applies a torque to the body via motor back-reaction. When the motor accelerates the wheel, Newton's 3rd law pushes the body the opposite direction.

### equations of motion

- $\theta$ is the angle of the body from vertical.
- $\varphi$ is the angle of the wheel.
- $I_\textrm{b}$ is the moment of inertia of the body.
- $I_\textrm{w}$ is the moment of inertia of the wheel.
- $\tau_\textrm{b} = I_\textrm{b} \ddot{\theta}$ is the torque on the body.
- $\tau_\textrm{w} = I_\textrm{w} \ddot{\varphi}$ is the torque from the wheel.

Newton's second law for rotation is $\tau = I \alpha$. Two things are trying to change $\theta$:

**Gravity** wants to pull the pendulum down. That torque is $mgl \sin(\theta)$, where $m$ is the mass of the body, $g$ is acceleration due to gravity, and $l$ is the distance to the center of mass.

**The motor** fights gravity by spinning a wheel. When the motor accelerates the wheel, Newton's third law shoves the body the opposite direction.

Putting them together, the net torque on the body is:

$$ I_\textrm{b} \ddot{\theta} = m g l \sin(\theta) - I_\textrm{w} \ddot{\varphi} $$

In English, the body's inertia is gravity's pull minus the motor's counteraction. The controller's job is to make the motor acceleration $\ddot{\varphi}$ cancel out gravity so that we keep $\theta$ near zero. The control input is the torque $\tau_\textrm{w}$, which accelerates the wheel and applies a restoring torque to the body. The wheel will spin up over time, so the controller must manage the wheel speed to prevent saturation.

### ode

The equation of motion is an ODE, because we are relating the angles and their second derivatives.

We will use the ODE to step forward in time to simulate the system's behavior, treating each step as an IVP.

### linearization

The equation above is nonlinear due to the $\sin(\theta)$ term. For small angles, we can approximate $\sin(\theta) \approx \theta$, which gives us a linearized model:

$$ I_\textrm{b} \ddot{\theta} = m g l \theta - I_\textrm{w} \ddot{\varphi} $$

## step 1: simulate the physics with no controller

- write the equation of motion that takes the current state $\theta$, $\dot{\theta}$, $\dot{\varphi}$ and returns the derivatives $\ddot{\theta}$ and $\ddot{\varphi}$.
- use an ODE solver to step forward in time
- plot $\theta$ over time to see the pendulum fall and swing
- verify equilibria

## step 2: add a controller

- start with a simple PD controller
- applies torque based on the angle and angular velocity of the body
- plot the response to see if it stabilizes
- should look like a damped oscillator

## step 3: rewrite as a daemon process in rust

- read from a simulated IMU
- use the IMU data to update the state of the pendulum
- compute the control torque based on the current state
- apply the torque to the wheel
- repeat in a loop to maintain balance

## sensing and actuation model

To move from a pure ODE to a real control system, we separate the problem into:

- the **plant dynamics** (how the pendulum and wheel move),
- the **measurement model** (what sensors tell us about that motion), and
- the **actuator model** (what torque the motor can actually produce).

### measurement model

The state we care about is approximately:

$$
x = [\theta,\ \dot{\theta},\ \omega_w]
$$

where $\theta$ and $\dot{\theta}$ are body angle/rate, and $\omega_w = \dot{\varphi}$ is wheel speed.

In practice, these come from different sensors:

- IMU gives body attitude/rate, used for $(\theta, \dot{\theta})$,
- hall sensor gives wheel kinematics, used for $\omega_w$,
- current sensor gives electrical feedback $i$, used for limits/diagnostics and later torque estimation.

So a simple measurement vector is:

$$
y = [\theta_m,\ \dot{\theta}_m,\ \omega_{w,m},\ i_m]
$$

### actuator model

The controller computes a desired torque:

$$
\tau_{cmd} = k_p\,\theta + k_d\,\dot{\theta}
$$

but the motor driver applies a limited torque:

$$
\tau = \mathrm{clamp}(\tau_{cmd}, -\tau_{max}(\omega_w, i), +\tau_{max}(\omega_w, i))
$$

This captures practical limits such as speed-dependent torque rolloff and current constraints.

### closed-loop ode view

With sensing + actuation included, each step is:

1. read $y$ from sensors,
2. estimate/use states $(\theta, \dot{\theta}, \omega_w)$,
3. compute $\tau_{cmd}$,
4. apply actuator limits to get $\tau$,
5. step the plant ODE with input $\tau$.

So the controlled system remains the same ODE framework, but now with a realistic measurement path and actuator saturation path.

## hardware bring-up plan

The safest way to start on real hardware is to bring the system up one interface at a time, from lowest risk to highest risk.

For this SparkFun IoT Brushless Motor Driver board, the first wire to use is the onboard USB-C connection. SparkFun's guide says the USB connection is used for both programming and serial communication, so that should be the default host link for early bring-up. The board also exposes a primary I2C bus on `GPIO 21`/`GPIO 22`, shared by the onboard TMAG5273 hall sensor and the Qwiic connector, which is the natural path for the external IMU and other low-speed sensors.

There is a documented address discrepancy around the onboard TMAG5273. SparkFun's IoT Motor Driver hardware guide lists the TMAG5273 I2C address as `0x35` in its hardware overview, while SparkFun's TMAG5273 Arduino library docs describe the sensor's configurable I2C address with a default value of `0x22`. In bring-up on this board, an I2C scan found a stable responder at `0x22` and nothing at `0x35`, so the firmware examples use `0x22` as the expected hall-sensor address.

Sources:
- SparkFun IoT Brushless Motor Driver hardware guide: https://docs.sparkfun.com/SparkFun_IoT_Brushless_Motor_Driver/hardware_overview/
- SparkFun TMAG5273 Arduino library docs: https://docs.sparkfun.com/SparkFun_TMAG5273_Arduino_Library/

### stage 1: prove firmware loading works

Goal: confirm we can reliably flash the ESP32 and reboot into our code.

- connect only USB-C
- build the smallest possible firmware that boots and stays alive
- if needed, use `BOOT` plus `RST` to enter the ESP32 serial bootloader
- repeat flashing more than once so we know it is not a one-off success

Exit criteria:

- firmware upload works repeatedly
- the board reboots into the expected program every time

### stage 2: prove code is visibly running

Goal: confirm our program executes without fault and gives obvious signs of life.

- blink or toggle the onboard status LED if we have access to it cleanly
- also print a heartbeat over USB serial once per second
- include a boot banner with firmware version and build timestamp or git hash

Exit criteria:

- visible LED behavior is stable
- USB serial shows a clean heartbeat for a few minutes with no resets or garbage

### stage 3: prove host communication works

Goal: confirm we can move bytes between the ESP32 and the development machine.

- use USB serial first (well documented), not an external UART wire
- keep the protocol trivial at first: text lines or a tiny command byte parser
- verify both directions:
  - ESP32 -> host: heartbeat, counters, sensor dumps
  - host -> ESP32: simple commands like `ping`, `led on`, `led off`

Exit criteria:

- host can receive logs reliably
- ESP32 can receive and act on simple host commands

### stage 4: read the onboard hall sensor

Goal: verify the primary I2C bus is working and we can read the TMAG5273.

- start by reading the hall sensor device ID or a known configuration register
- then stream raw magnetic field or angle-related readings over USB serial
- rotate the motor by hand and confirm the readings change smoothly

Exit criteria:

- device is detected on I2C every boot
- hand rotation produces repeatable changing measurements

### stage 5: read the onboard current sensor

Goal: verify INA240A1 readings are sane before we depend on them.

- first read with motor disabled and confirm the baseline is near zero
- then command a very small bounded motor action and verify current changes in the expected direction and magnitude
- treat this as a diagnostic signal first, not as a control input

Exit criteria:

- near-zero reading at idle
- nonzero response when the motor is gently driven
- no obviously saturated or stuck values

### stage 6: move the motor open-loop

Goal: prove we can command the TMC6300 path and get predictable motor motion.

- start with the smallest useful open-loop command
- use short bursts only
- log hall sensor and current sensor data while commanding motion
- verify spin direction, startup behavior, and whether the wheel coasts as expected

Exit criteria:

- wheel moves on command
- direction matches the commanded sign convention
- current and hall readings both react consistently

### stage 7: read the external IMU

Goal: verify the MPU is reachable on the same I2C bus and produces plausible data.

- wire the IMU onto the board's primary I2C bus through Qwiic or the `SDA`/`SCL` pins
- confirm voltage compatibility first; the board I/O is `3.3V` only
- begin with WHOAMI or equivalent identity reads
- then stream accel/gyro values over USB serial
- tilt the pendulum by hand and check that gravity and rate readings make sense

Exit criteria:

- IMU is detected reliably
- accel and gyro readings change in the expected axes and signs

### stage 8: combine sensing without control

Goal: verify that all measurements can run together at the intended loop rate.

- sample hall, current, and IMU in one loop
- timestamp the loop and print a compact telemetry line over USB serial
- check for I2C contention, timing jitter, and resets

Exit criteria:

- stable multi-sensor loop
- no missed devices or watchdog-like failures during extended runs

### stage 9: command torque with the pendulum restrained

Goal: validate sign conventions and scaling before any free-balance attempt.

- physically restrain or support the pendulum
- command tiny torques
- verify:
  - motor torque sign
  - hall sensor sign
  - IMU angle/rate sign
  - controller sign assumptions

Exit criteria:

- every sign convention is confirmed experimentally

### stage 10: closed-loop balancing trials

Goal: move from open-loop hardware validation to controlled balancing.

- begin with low gains and explicit torque limits
- add a dead-man timeout or explicit stop command
- keep telemetry streaming over USB serial during every run
- only after short stable runs should we try untethered or longer balancing tests

Exit criteria:

- bounded, repeatable stabilization attempts
- clear telemetry for every failure and success case

### recommended implementation order in this repo

If we want the software to follow the same sequence:

1. add tiny bring-up binaries under `penfw/src/bin`, starting with a boot-and-print test
2. add a simple host-link test such as `serial_echo`
3. add `hall_read`
4. add `current_read`
5. add `motor_open_loop`
6. add `imu_read`
7. add a combined telemetry binary
8. enable the PD control loop in the main `penfw` binary

## flashing penfw

The first firmware binary we have working for the ESP32 board is `blink` in `penfw/src/bin/blink.rs`. It drives the board's `STAT` RGB LED on `GPIO 2`.

### build the firmware

From the repo root, use `just` to build any firmware binary:

```bash
just blink
```

This compiles the firmware and produces the ELF at:

```bash
target/xtensa-esp32-none-elf/release/blink
```

Under the hood, this runs `cd penfw && cargo build --release --bin blink` using the Xtensa toolchain from the Nix shell.

### one-time host setup

The ESP firmware toolchain is provided by the repo-local Nix shell:

```bash
nix develop
```

That shell puts the Xtensa Rust toolchain, the Xtensa GCC linker toolchain, `ldproxy`, `espflash`, and `just` on `PATH`. It does not depend on any preinstalled Rustup or ESP host setup.

You may also need the CH340 serial driver on the host so the board shows up as a serial device over USB-C.

### connect the board

1. Plug the SparkFun IoT Brushless Motor Driver into your computer with USB-C.
2. Do not connect the motor or pendulum hardware yet for the first blink test.
3. Find the serial port name.

On macOS:

```bash
ls /dev/cu.*
```

You are looking for a device that appears when the board is plugged in, often something like `/dev/cu.wchusbserial*` or `/dev/cu.usbserial*`.

### normal flash attempt

Once the firmware is built, flash it with:

```bash
just flash blink
```

If the board enumerates normally and auto-detection works, this should:

- reset the ESP32 into the serial bootloader
- flash the `blink` firmware
- reboot into the new program
- open a serial monitor afterward

### if auto-bootloader does not work

SparkFun documents manual firmware download mode with the `BOOT` and `RST` buttons:

1. Hold down `BOOT`.
2. While holding `BOOT`, press `RST` if the board is already powered, or plug in USB-C if it is not.
3. Release `BOOT`.
4. Run the flash command again.
5. After flashing finishes, press `RST` once to reboot into the new firmware if needed.

So the manual recovery flow is:

```bash
just flash blink
```

If it fails to connect:

- hold `BOOT`
- tap `RST`
- release `BOOT`
- immediately rerun the flash command

### what success should look like

For this `blink` binary, success means:

- flashing completes without errors
- the board resets
- the `STAT` RGB LED begins blinking red on and off

### useful commands

Rebuild the `blink` firmware:

```bash
just blink
```

Flash it to the board:

```bash
just flash blink
```

Check compilation without flashing:

```bash
cd penfw && cargo check --release --bin blink
```

Build and flash any other binary (e.g., `serial_echo`, `hall_read`, `imu_read`):

```bash
just serial_echo
just flash serial_echo
```

See all available build and flash targets:

```bash
just --list
```
