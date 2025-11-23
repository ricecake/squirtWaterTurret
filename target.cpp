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
	last_aimpoint.reset();
}

std::shared_ptr<const PositionVector> Target::interceptPosition() const {
	using localFixed = fixed_16_16;

	const PositionVector       proj_pos = PositionVector(0, 0, 1.5);
	const PositionVector       target_pos = position;
	const VelocityVector       target_velocity = velocity;
	const localFixed           proj_speed = 20;
	const Vector3D<localFixed> Gv(0, 0, 9.814);
	const localFixed           G = Gv.magnitude();

	if (position.magnitude() > (proj_speed * proj_speed) / G) {
		return std::make_shared<PositionVector>(position);
	}

	const int64_t P = target_velocity.X_coord.raw_value();
	const int64_t Q = target_velocity.Z_coord.raw_value();
	const int64_t R = target_velocity.Y_coord.raw_value();

	const auto    diff = target_pos - proj_pos;
	const int64_t H = diff.X_coord.raw_value();
	const int64_t J = diff.Z_coord.raw_value();
	const int64_t K = diff.Y_coord.raw_value();

	const int64_t L = (localFixed(-0.5) * G).raw_value();
	const int64_t S = proj_speed.raw_value();

	const int64_t v_mag_sq = (P * P + Q * Q + R * R) >> localFixed::FixedBits;
	const int64_t s_mag_sq = (S * S) >> localFixed::FixedBits;
	const int64_t p_d_dot_v = (H * P + J * Q + K * R) >> localFixed::FixedBits;
	const int64_t p_d_mag_sq = (H * H + J * J + K * K) >> localFixed::FixedBits;

	// Quartic Coefficients
	const localFixed c4 = localFixed::from_raw_value(p_d_mag_sq);
	const localFixed c3 = localFixed::from_raw_value(p_d_dot_v * 2);
	const localFixed c2 = localFixed::from_raw_value(v_mag_sq - s_mag_sq - ((2 * J * L) >> localFixed::FixedBits));
	const localFixed c1 = localFixed::from_raw_value((-2 * Q * L) >> localFixed::FixedBits);
	const localFixed c0 = localFixed::from_raw_value((L * L) >> localFixed::FixedBits);

	const std::function<localFixed(const localFixed&)> movingTargetInterceptQuartic =
		[=](const localFixed& t) -> localFixed {
		const auto t2 = t * t;
		const auto t3 = t2 * t;
		const auto t4 = t3 * t;
		return c0 * t4 + c1 * t3 + c2 * t2 + c3 * t + c4;
	};

	const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

	if (!converged || intercept == 0) {
		return std::make_shared<PositionVector>(diff.X_coord, diff.Y_coord, diff.Z_coord);
	}

	const localFixed intercept_t = intercept;

	const auto x_vel_num = diff.X_coord + target_velocity.X_coord * intercept_t;
	const auto y_vel_num = diff.Y_coord + target_velocity.Y_coord * intercept_t;

	// We want Vp.z = (diff.z + V_t.z*t - 0.5*g*t^2)/t
	// L = -0.5*g
	// So we want (diff.z + V_t.z*t + L*t^2)/t
	const auto z_vel_num = diff.Z_coord + target_velocity.Z_coord * intercept_t -
		localFixed::from_raw_value(L) * intercept_t * intercept_t;

	return std::make_shared<PositionVector>(x_vel_num / intercept_t, y_vel_num / intercept_t, z_vel_num / intercept_t);
};
