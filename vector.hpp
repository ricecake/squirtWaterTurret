/**
 * @file vector.hpp
 * @brief Provides a generic 3D vector class with basic vector arithmetic.
 */
#pragma once

#include <concepts>
#include <type_traits>

/**
 * @brief A concept to identify types that support basic arithmetic operations.
 *
 * This concept is used to constrain template parameters to numeric types
 * or custom types that overload the standard arithmetic operators.
 */
template <typename T, typename U>
concept Number = requires(T obj, U thi) {
	requires std::is_arithmetic_v<T> || requires {
		obj + thi;
		obj - thi;
		obj* thi;
		obj / thi;
	};
};

template <typename Numeric, typename Derived>
class Vector3D;

/**
 * @brief A concept to identify types that are compatible with `Vector3D` for operations.
 *
 * A type is considered vector-compatible if it has public members `X_coord`, `Y_coord`,
 * and `Z_coord` of the specified `Numeric` type. This allows operations between
 * different but structurally similar vector types.
 */
template <typename T, typename Numeric>
concept VectorCompatible = requires(T vec) {
	{ vec.X_coord } -> std::same_as<Numeric&>;
	{ vec.Y_coord } -> std::same_as<Numeric&>;
	{ vec.Z_coord } -> std::same_as<Numeric&>;
};

/**
 * @brief A type alias to select either an explicit type or a default type.
 *
 * This is used in `Vector3D` to allow the derived class type to be explicitly
 * provided or to default to the base `Vector3D` itself.
 */
template <typename ExplicitType, typename Default>
using Either = std::conditional_t<std::is_same_v<ExplicitType, void>, Default, ExplicitType>;

/**
 * @brief A generic 3D vector class.
 *
 * This class provides a foundation for 3D vector mathematics. It uses the
 * Curiously Recurring Template Pattern (CRTP), where the `Derived` template
 * parameter is the class that inherits from `Vector3D`. This allows derived
 * classes (like `PositionVector`, `VelocityVector`) to return objects of their
 * own type from the base class's methods, ensuring type correctness.
 *
 * @tparam Numeric The numeric type for the vector's components (e.g., `float`, `fixed_16_16`).
 * @tparam Derived The derived class type for CRTP. Defaults to `void`, in which case
 *                 the methods return `Vector3D<Numeric>`.
 */
template <typename Numeric, typename Derived = void>
class Vector3D {
public:
	using NumericType = Numeric;
	using ClassType = Either<Derived, Vector3D<NumericType>>;
	constexpr static NumericType rad2DegFactor = NumericType(57.2957795131);

public:
	// -- Static Directional Vectors --
	constexpr static Vector3D<Numeric> Up = Vector3D<Numeric>(0, 0, 1);
	constexpr static Vector3D<Numeric> Down = Vector3D<Numeric>(0, 0, -1);
	constexpr static Vector3D<Numeric> Left = Vector3D<Numeric>(-1, 0, 0);
	constexpr static Vector3D<Numeric> Right = Vector3D<Numeric>(1, 0, 0);
	constexpr static Vector3D<Numeric> Forward = Vector3D<Numeric>(0, 1, 0);
	constexpr static Vector3D<Numeric> Backward = Vector3D<Numeric>(0, -1, 0);

	// -- Public Attributes --
	NumericType X_coord; ///< The X-coordinate (e.g., left/right) of the vector.
	NumericType Y_coord; ///< The Y-coordinate (e.g., forward/backward) of the vector.
	NumericType Z_coord; ///< The Z-coordinate (e.g., up/down) of the vector.

	// -- Constructors --
	/**
	 * @brief Default constructor, initializes to a zero vector.
	 */
	constexpr explicit Vector3D() : X_coord(0), Y_coord(0), Z_coord(0) {}
	/**
	 * @brief Constructs a vector with specified x, y, and z components.
	 */
	constexpr explicit Vector3D(NumericType X_coord, NumericType Y_coord, NumericType Z_coord) :
		X_coord(X_coord), Y_coord(Y_coord), Z_coord(Z_coord) {}

	// -- Operators --
	/**
	 * @brief Checks if the vector is non-zero.
	 * @return `true` if any component is non-zero.
	 */
	operator bool() const { return X_coord || Y_coord || Z_coord; }

	/**
	 * @brief Adds two vectors component-wise.
	 * @return A new vector of the `ClassType` (the derived type).
	 */
	ClassType operator+(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(X_coord + other.X_coord, Y_coord + other.Y_coord, Z_coord + other.Z_coord);
	}

	/**
	 * @brief Subtracts one vector from another component-wise.
	 * @return A new vector of the `ClassType`.
	 */
	ClassType operator-(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(X_coord - other.X_coord, Y_coord - other.Y_coord, Z_coord - other.Z_coord);
	}

	/**
	 * @brief Multiplies the vector by a scalar.
	 * @return A new vector of the `ClassType`, scaled by the scalar.
	 */
	constexpr ClassType operator*(const NumericType& scalar) const {
		return ClassType(X_coord * scalar, Y_coord * scalar, Z_coord * scalar);
	}

	/**
	 * @brief Divides the vector by a scalar.
	 * @return A new vector of the `ClassType`, scaled by the inverse of the scalar.
	 */
	ClassType operator/(NumericType scalar) const {
		return ClassType(X_coord / scalar, Y_coord / scalar, Z_coord / scalar);
	}

	// -- Vector Operations --
	/**
	 * @brief Computes the dot product of this vector with another.
	 * @return The scalar dot product.
	 */
	NumericType dot(const VectorCompatible<NumericType> auto& other) const {
		return X_coord * other.X_coord + Y_coord * other.Y_coord + Z_coord * other.Z_coord;
	}

	/**
	 * @brief Computes the cross product of this vector with another.
	 * @return A new vector of the `ClassType` that is orthogonal to both input vectors.
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
	 * @return The scalar magnitude.
	 */
	NumericType magnitude() const { return sqrt((X_coord * X_coord) + (Y_coord * Y_coord) + (Z_coord * Z_coord)); }

	/**
	 * @brief Computes the magnitude of the vector's projection onto the XY plane.
	 * @return The 2D magnitude in the XY plane.
	 */
	NumericType magnitudeXY() const { return sqrt(X_coord * X_coord + Y_coord * Y_coord); }

	/**
	 * @brief Computes the magnitude of the vector's projection onto the XZ plane.
	 * @return The 2D magnitude in the XZ plane.
	 */
	NumericType magnitudeXZ() const { return sqrt(X_coord * X_coord + Z_coord * Z_coord); }

	/**
	 * @brief Computes the magnitude of the vector's projection onto the YZ plane.
	 * @return The 2D magnitude in the YZ plane.
	 */
	NumericType magnitudeYZ() const { return sqrt(Y_coord * Y_coord + Z_coord * Z_coord); }

	/**
	 * @brief Computes the normalized vector (a unit vector with the same direction).
	 * @return A new vector of the `ClassType` with a magnitude of 1.
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
	 * Pitch is the angle between the vector and the XY plane.
	 * @return The pitch angle in degrees.
	 */
	NumericType pitch() const { return atan2(Z_coord, magnitudeXY()) * rad2DegFactor; }

	/**
	 * @brief Computes the yaw angle of the vector.
	 * Yaw is the angle of the vector's projection on the XY plane from the Y-axis.
	 * @return The yaw angle in degrees.
	 */
	NumericType yaw() const { return atan2(X_coord, Y_coord) * rad2DegFactor; }

	/**
	 * @brief Computes the angle between this vector and another.
	 * @return The angle in degrees.
	 */
	NumericType angleTo(const VectorCompatible<NumericType> auto& other) const {
		return atan2(cross(other).magnitude(), dot(other)) * rad2DegFactor;
	}
};