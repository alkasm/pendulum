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
