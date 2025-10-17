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

## 2. Target Selection and Management

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