#include "target.h"

#include <climits>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include <math.h>
#include <stdint.h>

/**
 * @brief Updates the target's state with a new position measurement.
 *
 * This method recalculates the target's velocity based on the change in position
 * and time since the last update. It also updates the `seen` and `position` history.
 *
 * @param P The new position vector.
 */
void Target::Update(PositionVector P) {
	TimePoint    new_seen = Clock::now();
	TimeInterval time_delta = TimeInterval(new_seen - seen);

	if (time_delta.count() > 0) {
		// Calculate velocity from the displacement and time delta.
		velocity = (P - position) / time_delta;
	}
	// If no time has passed, velocity remains unchanged.

	// Update the target's history.
	last_position = position;
	position = P;
	last_seen = seen;
	seen = new_seen;
}

const PositionVector Target::interceptPosition() const {
	const PositionVector       proj_pos = PositionVector(0, 0, 1.5);
	const PositionVector       target_pos = position;
	const VelocityVector       target_velocity = velocity;
	const fixed_24_8           proj_speed = 20;
	const Vector3D<fixed_24_8> Gv(0, 0, 9.814);
	const fixed_24_8           G = Gv.magnitude();

	const fixed_24_8 P = target_velocity.X_coord;
	const fixed_24_8 Q = target_velocity.Z_coord;
	const fixed_24_8 R = target_velocity.Y_coord;

	const auto       diff = target_pos - proj_pos;
	const fixed_24_8 H = diff.X_coord;
	const fixed_24_8 J = diff.Z_coord;
	const fixed_24_8 K = diff.Y_coord;

	const fixed_24_8 L = fixed_24_8(-0.5) * G;
	const fixed_24_8 S = proj_speed;

	// Quartic Coefficients
	const fixed_24_8 c0 = L * L;
	const fixed_24_8 c1 = -2 * Q * L;
	const fixed_24_8 c2 = -2 * J * L + fixed_24_8(target_velocity.dot(target_velocity)) - (S * S);
	const fixed_24_8 c3 = 2 * (diff.dot(target_velocity));
	const fixed_24_8 c4 = diff.dot(diff);

	const std::function<fixed_24_8(const fixed_24_8)> movingTargetInterceptQuartic =
		[=](const fixed_24_8 t) -> fixed_24_8 {
		const fixed_24_8 t2 = t * t;
		return c0 * (t2 * t2) + c1 * (t2 * t) + c2 * t2 + c3 * t + c4;
	};

	const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

	if (!converged || intercept == 0) {
		return PositionVector(H, K, J);
	}

	auto pos = diff + target_velocity * intercept;
	pos.Z_coord = fixed_24_8(pos.Z_coord) - L * (intercept * intercept);

	return PositionVector(
		(H + P * intercept) / intercept,
		(K + R * intercept) / intercept,
		(J + Q * intercept - L * (intercept * intercept)) / intercept
	);
};
