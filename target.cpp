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
	using localFixed = fixed_16_16;
	const PositionVector       proj_pos = PositionVector(0, 0, 1.5);
	const PositionVector       target_pos = position;
	const VelocityVector       target_velocity = velocity;
	const localFixed           proj_speed = 20;
	const Vector3D<localFixed> Gv(0, 0, 9.814);
	const localFixed           G = Gv.magnitude();

	const localFixed P = target_velocity.X_coord;
	const localFixed Q = target_velocity.Z_coord;
	const localFixed R = target_velocity.Y_coord;

	const auto       diff = target_pos - proj_pos;
	const localFixed H = diff.X_coord;
	const localFixed J = diff.Z_coord;
	const localFixed K = diff.Y_coord;

	const localFixed L = localFixed(-0.5) * G;
	const localFixed S = proj_speed;

	// Quartic Coefficients
	const localFixed c0 = L * L;
	const localFixed c1 = -2 * Q * L;
	const localFixed c2 = -2 * J * L + localFixed(target_velocity.dot(target_velocity)) - pow(S, 2);
	const localFixed c3 = 2 * (diff.dot(target_velocity));
	const localFixed c4 = diff.dot(diff);

	const std::function<localFixed(const localFixed)> movingTargetInterceptQuartic =
		[=](const localFixed t) -> localFixed {
		auto t2 = t * t;
		return c0 * (t2 * t2) + c1 * (t2 * t) + c2 * t2 + c3 * t + c4;
	};

	const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

	if (!converged || intercept == 0) {
		return PositionVector(H, K, J);
	}

	auto pos = diff + target_velocity * intercept;
	pos.Z_coord = localFixed(pos.Z_coord) - L * pow(intercept, 2);

	return PositionVector(
		(H + P * intercept) / intercept,
		(K + R * intercept) / intercept,
		(J + Q * intercept - L * pow(intercept, 2)) / intercept
	);
};
