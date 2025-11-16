#pragma once

#include <concepts>
#include <type_traits>

/**
 * @brief Concept to check if a type supports basic arithmetic operations.
 */
template <typename T, typename U>
concept Number = requires(T obj, U thi) {
	requires std::is_arithmetic_v<T> || requires {
		obj + thi;
		obj - thi;
		obj * thi;
		obj / thi;
	};
};

template <typename Numeric, typename Derived>
class Vector3D;

/**
 * @brief Concept to check if a type is compatible with Vector3D for operations.
 */
template <typename T, typename Numeric>
concept VectorCompatible = requires(T vec) {
	{ vec.X_coord } -> std::same_as<Numeric&>;
	{ vec.Y_coord } -> std::same_as<Numeric&>;
	{ vec.Z_coord } -> std::same_as<Numeric&>;
};

/**
 * @brief A type alias to select either an explicit type or a default type.
 */
template <typename ExplicitType, typename Default>
using Either = std::conditional_t<std::is_same_v<ExplicitType, void>, Default, ExplicitType>;

/**
 * @brief A generic 3D vector class.
 *
 * This class provides a foundation for 3D vector operations. It uses the
 * Curiously Recurring Template Pattern (CRTP) to allow derived classes
 * to customize behavior while reusing the base implementation.
 *
 * @tparam Numeric The numeric type for the vector's components (e.g., float, fixed-point).
 * @tparam Derived The derived class type for CRTP.
 */
template <typename Numeric, typename Derived = void>
class Vector3D {
public:
	using NumericType = Numeric;
	using ClassType = Either<Derived, Vector3D<NumericType>>;
	constexpr static NumericType rad2DegFactor = NumericType(57.2957795131);

public:
	constexpr static Vector3D<Numeric> Up = Vector3D<Numeric>(0, 0, 1);
	constexpr static Vector3D<Numeric> Down = Vector3D<Numeric>(0, 0, -1);
	constexpr static Vector3D<Numeric> Left = Vector3D<Numeric>(-1, 0, 0);
	constexpr static Vector3D<Numeric> Right = Vector3D<Numeric>(1, 0, 0);
	constexpr static Vector3D<Numeric> Forward = Vector3D<Numeric>(0, 1, 0);
	constexpr static Vector3D<Numeric> Backward = Vector3D<Numeric>(0, -1, 0);

	NumericType X_coord; ///< The X-coordinate of the vector.
	NumericType Y_coord; ///< The Y-coordinate of the vector.
	NumericType Z_coord; ///< The Z-coordinate of the vector.

	constexpr explicit Vector3D(): X_coord(0), Y_coord(0), Z_coord(0) {}

	constexpr Vector3D(NumericType X_coord, NumericType Y_coord, NumericType Z_coord):
		X_coord(X_coord), Y_coord(Y_coord), Z_coord(Z_coord) {}

	constexpr ClassType FromPolarDegrees(NumericType Pitch, NumericType Yaw, NumericType Radius) {
		return Vector3D(Pitch, Yaw, Radius);
	}

	friend std::ostream& operator<<(std::ostream& os, const ClassType& obj) {
		os << "Vec<" << obj.X_coord << ", " << obj.Y_coord << ", " << obj.Z_coord << ">";
		return os;
	}

	/**
	 * @brief Checks if the vector is non-zero.
	 */
	operator bool() const { return X_coord || Y_coord || Z_coord; }

	/**
	 * @brief Adds two vectors.
	 */
	ClassType operator+(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(X_coord + other.X_coord, Y_coord + other.Y_coord, Z_coord + other.Z_coord);
	}

	/**
	 * @brief Subtracts one vector from another.
	 */
	ClassType operator-(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(X_coord - other.X_coord, Y_coord - other.Y_coord, Z_coord - other.Z_coord);
	}

	/**
	 * @brief Multiplies the vector by a scalar.
	 */
	constexpr ClassType operator*(const NumericType& scalar) const {
		return ClassType(X_coord * scalar, Y_coord * scalar, Z_coord * scalar);
	}

	/**
	 * @brief Divides the vector by a scalar.
	 */
	ClassType operator/(NumericType scalar) const {
		return ClassType(X_coord / scalar, Y_coord / scalar, Z_coord / scalar);
	}

	/**
	 * @brief Computes the dot product of two vectors.
	 */
	NumericType dot(const VectorCompatible<NumericType> auto& other) const {
		return X_coord * other.X_coord + Y_coord * other.Y_coord + Z_coord * other.Z_coord;
	}

	/**
	 * @brief Computes the cross product of two vectors.
	 */
	ClassType cross(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(
			Y_coord * other.Z_coord - Z_coord * other.Y_coord,
			Z_coord * other.X_coord - X_coord * other.Z_coord,
			X_coord * other.Y_coord - Y_coord * other.X_coord
		);
	}

	/**
	 * @brief Computes the magnitude (length) of the vector.
	 */
	NumericType magnitude() const {
		auto xs = X_coord * X_coord;
		auto ys = Y_coord * Y_coord;
		auto zs = Z_coord * Z_coord;
		return sqrt(xs + ys + zs);
	}

	/**
	 * @brief Computes the magnitude in the XY plane.
	 */
	NumericType magnitudeXY() const { return sqrt(X_coord * X_coord + Y_coord * Y_coord); }

	/**
	 * @brief Computes the magnitude in the XZ plane.
	 */
	NumericType magnitudeXZ() const { return sqrt(X_coord * X_coord + Z_coord * Z_coord); }

	/**
	 * @brief Computes the magnitude in the YZ plane.
	 */
	NumericType magnitudeYZ() const { return sqrt(Y_coord * Y_coord + Z_coord * Z_coord); }

	/**
	 * @brief Computes the normalized vector (unit vector).
	 */
	ClassType normalize() const {
		NumericType mag = magnitude();
		if (mag != 0) {
			return ClassType(X_coord / mag, Y_coord / mag, Z_coord / mag);
		}
		return ClassType(X_coord, Y_coord, Z_coord);
	}

	/**
	 * @brief Computes the pitch angle of the vector.
	 */
	NumericType pitch() const { return atan2(Z_coord, magnitudeXY()) * rad2DegFactor; }

	/**
	 * @brief Computes the yaw angle of the vector.
	 */
	NumericType yaw() const { return atan2(X_coord, Y_coord) * rad2DegFactor; }

	/**
	 * @brief Computes the angle to another vector.
	 */
	NumericType angleTo(const VectorCompatible<NumericType> auto& other) const {
		if (!(*this && other)) {
			return NumericType(0);
		}
		auto normalThis = normalize();
		auto normalOther = other.normalize();

		auto dotted = normalThis.dot(normalOther);
		return acos(dotted) * rad2DegFactor;
	}
};
