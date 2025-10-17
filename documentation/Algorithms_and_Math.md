# Algorithms and Mathematics

This document explains some of the key algorithms and mathematical principles employed in the project, particularly within the firmware.

## 1. Ballistic Targeting

A core feature of the fire control system is its ability to account for gravity. Simply aiming directly at a distant target will result in the water stream falling short. To hit the target, the turret must aim slightly higher. Calculating the correct angle is a classic physics problem.

### The Physics

The trajectory of a projectile under gravity can be described by a set of parametric equations. However, solving for the launch angle required to hit a specific point `(y, z)` (where `y` is horizontal distance and `z` is height difference) involves solving a complex equation.

The direct analytical solution is computationally expensive and involves trigonometric functions and square roots, which are slow on microcontrollers without a dedicated Floating-Point Unit (FPU).

### The Solution: Numerical Approximation

Instead of solving the equation directly, the firmware uses a numerical approximation method to find the answer. This approach iteratively refines a guess until it is "good enough."

**`Approximate::small_root` function (`aproximate_math.hpp`)**

This function is the heart of the ballistic calculation. Its goal is to find the *root* of a function, which is the input value `x` for which `f(x) = 0`. The ballistic problem is reframed into a root-finding problem.

The `small_root` function implements a hybrid of two common numerical methods:

1.  **Bisection Method:** This method is robust. It starts with an interval `[a, b]` where `f(a)` and `f(b)` have opposite signs, guaranteeing a root exists between them. It then repeatedly cuts the interval in half, always keeping the half that still contains the root.

2.  **Secant Method:** This method is typically faster than bisection. It draws a line (a secant) through the points `(a, f(a))` and `(b, f(b))` and uses the x-intercept of this line as the next guess.

**How it works:**
1.  **Interval Finding:** The function first searches for an interval `[leftInput, rightInput]` where the sign of the function's output changes, which indicates a root is present. It does this by starting with a small interval and rapidly expanding it.
2.  **Iterative Refinement:** It enters a loop that runs for a fixed number of `rounds` (e.g., 16).
3.  In each round, it calculates the next guess using the faster **Secant Method**.
4.  A crucial check is performed: if the secant method's guess falls *outside* the current valid interval (which can happen), it falls back to the more reliable **Bisection Method** (by simply taking the midpoint of the interval).
5.  It then narrows the interval based on the sign of the function at the new guess.
6.  **Convergence Check:** The loop terminates if the proportional error `(rightInput - leftInput) / rightInput` is smaller than a predefined threshold, or if the maximum number of rounds is reached.

This hybrid approach combines the speed of the secant method with the guaranteed convergence of the bisection method, making it an excellent choice for a real-time embedded system. It provides a good-enough answer in a predictable amount of time.

### `Target::interceptPosition`
This method in the `Target` class formulates the ballistic problem for a *moving* target as a quartic equation (an equation with `t^4` as the highest power) and uses the `Approximate::small_root` solver to find the time-to-intercept (`t`). Once `t` is known, it's straightforward to calculate where the target *will be* at that time and aim there.

#### Moving Target Intercept Equations
The implementation is based on the work of Forrest Smith, as detailed in his blog post "[Solving Ballistic Trajectories](https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/)." The core of the problem is to solve for `t` (time) in the following system of equations, which results in a quartic polynomial.

**Variable Definitions:**
- `proj_pos`: Projectile initial position vector `(x, y, z)`
- `target_pos`: Target initial position vector `(x, y, z)`
- `target_velocity`: Target velocity vector `(vx, vy, vz)`
- `proj_speed`: Projectile's initial speed (a scalar)
- `Gv`: Gravity vector `(gx, gy, gz)`
- `G`: Magnitude of gravity (a scalar)

**Derived Variables (for simplicity):**
- `diff = target_pos - proj_pos`
- `H = diff.x`, `J = diff.z`, `K = diff.y`
- `P = target_velocity.x`, `Q = target_velocity.z`, `R = target_velocity.y`
- `L = -0.5 * G`
- `S = proj_speed`

**The Quartic Equation:**
The problem is reduced to solving `c0*t^4 + c1*t^3 + c2*t^2 + c3*t + c4 = 0`, where the coefficients are:

-   `c0 = L^2`
-   `c1 = -2 * Q * L`
-   `c2 = (target_velocity . target_velocity) - 2 * J * L - S^2`
-   `c3 = 2 * (diff . target_velocity)`
-   `c4 = diff . diff`

The `Approximate::small_root` function is used to find the smallest, positive, real root of this quartic equation, which represents the time to intercept.

## 2. Mechanical Control and Motor Calculation

### Differential Wrist Mechanism

The physical turret mechanism is a pan-and-tilt system driven by two stationary stepper motors. The motion is mechanically coupled through a set of gears in a configuration analogous to a [differential wrist](https://ewhiteowls.com/2023/01/very-effective-wrist-design-with-in-built-planetary-gearboxes-for-an-light-weight-and-compact-wrist-for-6dof-robotic-arm/). This design allows two motors, which are fixed to the base, to control two separate axes of rotation (pan and tilt) at the end of the mechanism.

The key advantage is that the motors do not have to carry each other's weight, leading to a lighter and potentially faster assembly.

### From Pan/Tilt to Motor Rotations

The relationship between the desired pan (`α`) and tilt (`β`) angles and the required rotation of the two motors (`θ₁` and `θ₂`) is the core of the differential mechanism.

-   **Panning (`α`):** The pan (horizontal) angle of the turret is controlled by the *average* rotation of the two motors. If both motors turn in the same direction by the same amount, the turret will pan left or right.
    `α = k * (θ₁ + θ₂) / 2`

-   **Tilting (`β`):** The tilt (vertical) angle of the turret is controlled by the *difference* in rotation between the two motors. If the motors turn in opposite directions by the same amount, the turret will tilt up or down.
    `β = k * (θ₁ - θ₂) / 2`

Where `k` is a scaling constant determined by the gearing of the system.

### From Angles to Motor Steps

To aim at a specific target, the firmware must perform the inverse calculation: convert the desired pan and tilt angles into target step counts for each motor.

1.  **Angle Calculation:** The `PositionVector::Pitch()` (tilt) and `PositionVector::Yaw()` (pan) methods first calculate the required angles from the target's 3D coordinates.
2.  **Inverse Kinematics:** The firmware then solves the system of equations for `θ₁` and `θ₂`:
    -   `θ₁ = (α + β) / k`
    -   `θ₂ = (α - β) / k`
3.  **Angle to Steps:** The resulting motor angles (`θ₁` and `θ₂`) are then converted into motor steps using a calibrated `angleToStep` constant. This constant accounts for the motor's properties (e.g., 1.8° per step), microstepping settings, and overall gear ratio.
    -   `Motor1_Steps = θ₁ * angleToStep`
    -   `Motor2_Steps = θ₂ * angleToStep`
4.  **Execution:** These final step counts are passed to the `AccelStepper` library, which handles the smooth movement of each motor to its target position.

## 3. Target Selection and Management

The firmware needs to manage lists of potential targets from different sources (`CV`, `RADAR`).

### `SystemState::updateTargetById`
When the CV system sends an update for a target with a specific ID, this function is called.
1.  It first tries to find an existing target in the `cvTarget` array with the matching `id`.
2.  If no match is found, it assumes this is a new target. It then looks for an "invalid" slot in the array (a `Target` object where `valid == false`).
3.  If there are no invalid slots, it falls back to overwriting the "oldest" target, determined by the `seen` timestamp. This ensures that the system can track new targets even when the maximum number of tracked objects is reached.

### `SystemState::updateNearestTarget`
This logic is used for the radar system, which doesn't provide stable IDs.
1.  It takes a new position and finds the `Target` in the `radarTarget` array that is geometrically closest to this new position.
2.  It then updates that nearest `Target` with the new coordinates. This is a simple but effective way to track objects without persistent identities.

### Indifference Margin
A key parameter in the update logic is `indifferenceMargin`. This prevents the system from making tiny, unnecessary adjustments. A target's position is only updated if the angular distance the turret would have to travel is greater than this margin. This helps to smooth out motor movements and reduce jitter caused by noisy sensor data.