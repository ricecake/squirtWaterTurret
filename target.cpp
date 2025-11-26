#include "target.h"

#include <chrono>
#include <climits>
#include <memory>

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

	if (time_delta.count() > 0 && valid) {
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

	const int64_t P = target_velocity.x.raw_value();
	const int64_t Q = target_velocity.z.raw_value();
	const int64_t R = target_velocity.y.raw_value();

	const auto    diff = target_pos - proj_pos;
	const int64_t H = diff.x.raw_value();
	const int64_t J = diff.z.raw_value();
	const int64_t K = diff.y.raw_value();

	const int64_t L = (localFixed(-0.5) * G).raw_value();
	const int64_t S = proj_speed.raw_value();

	const int64_t v_mag_sq = (P * P + Q * Q + R * R) >> localFixed::FixedBits;
	const int64_t s_mag_sq = (S * S) >> localFixed::FixedBits;
	const int64_t p_d_dot_v = (H * P + J * Q + K * R) >> localFixed::FixedBits;
	const int64_t p_d_mag_sq = (H * H + J * J + K * K) >> localFixed::FixedBits;

	int64_t c4_raw = p_d_mag_sq;
	int64_t c3_raw = p_d_dot_v * 2;
	int64_t c2_raw = v_mag_sq - s_mag_sq - ((2 * J * L) >> localFixed::FixedBits);
	int64_t c1_raw = (-2 * Q * L) >> localFixed::FixedBits;
	int64_t c0_raw = (L * L) >> localFixed::FixedBits;

	// Find the maximum absolute magnitude to determine required shift
	auto    safe_abs = [](int64_t v) { return v < 0 ? -v : v; };
	int64_t max_val = std::max(
		{safe_abs(c4_raw), safe_abs(c3_raw), safe_abs(c2_raw), safe_abs(c1_raw), safe_abs(c0_raw)}
	);

	// Max allowable value in 32-bit fixed point (leaving a bit of headroom for safety)
	const int64_t MAX_FIXED = 0x3FFFFFFF;

	int shift_amount = 0;
	while (max_val > MAX_FIXED) {
		max_val >>= 1;
		shift_amount++;
	}

	// Quartic Coefficients
	// Apply scaling shift to all coefficients uniformly
	// This preserves the roots of the polynomial f(t) = 0
	const localFixed c4 = localFixed::from_raw_value(static_cast<localFixed::base_type>(c4_raw >> shift_amount));
	const localFixed c3 = localFixed::from_raw_value(static_cast<localFixed::base_type>(c3_raw >> shift_amount));
	const localFixed c2 = localFixed::from_raw_value(static_cast<localFixed::base_type>(c2_raw >> shift_amount));
	const localFixed c1 = localFixed::from_raw_value(static_cast<localFixed::base_type>(c1_raw >> shift_amount));
	const localFixed c0 = localFixed::from_raw_value(static_cast<localFixed::base_type>(c0_raw >> shift_amount));

	const std::function<localFixed(const localFixed&)> movingTargetInterceptQuartic =
		[=](const localFixed& t1) -> localFixed {
		// auto t1 = localFixed::from_raw_value(t);
		// Parentheses are crucial here to enforce order of operations
		return (((c0 * t1 + c1) * t1 + c2) * t1 + c3) * t1 + c4;
	};

	const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

	if (!converged || intercept == 0) {
		return std::make_shared<PositionVector>(diff.x, diff.y, diff.z);
	}

	const localFixed intercept_t = intercept;

	const auto x_vel_num = diff.x + target_velocity.x * intercept_t;
	const auto y_vel_num = diff.y + target_velocity.y * intercept_t;

	// We want Vp.z = (diff.z + V_t.z*t - 0.5*g*t^2)/t
	// L = -0.5*g
	// So we want (diff.z + V_t.z*t + L*t^2)/t
	const auto z_vel_num = diff.z + target_velocity.z * intercept_t -
		localFixed::from_raw_value(static_cast<fixed::base_type>(L)) * intercept_t * intercept_t;

	auto result = std::make_shared<PositionVector>(
		x_vel_num / intercept_t,
		y_vel_num / intercept_t,
		z_vel_num / intercept_t
	);
	// *result = result->normalize();
	return result;
};
