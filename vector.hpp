/**
 * @file vector.hpp
 * @brief A generic 3D vector class template.
 *
 * This header provides a `Vector3D` class template that serves as a foundation
 * for 3D vector mathematics. It uses the Curiously Recurring Template Pattern (CRTP)
 * to allow for the creation of specialized vector types (like Position, Velocity)
 * while reusing the core arithmetic and geometric operations.
 */
#pragma once

#include <concepts>
#include <type_traits>

/**
 * @brief Concept to check if a type supports basic arithmetic operations.
 *
 * This concept is satisfied if the type is either a standard arithmetic type
 * or if it overloads the basic arithmetic operators (+, -, *, /).
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
 * @brief Concept to check if a type is compatible with Vector3D for operations.
 *
 * A type is vector-compatible if it has public members `X_coord`, `Y_coord`, and `Z_coord`
 * of the specified `Numeric` type. This allows operations between different, but
 * structurally similar, vector types.
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
 * This is used in the `Vector3D` class to allow it to be used either as a
 * standalone class or as a base class in the CRTP pattern. If `Derived` is `void`,
 * it defaults to `Vector3D<Numeric>`, otherwise it uses the provided `Derived` type.
 */
template <typename ExplicitType, typename Default>
using Either = std::conditional_t<std::is_same_v<ExplicitType, void>, Default, ExplicitType>;

/**
 * @brief A generic 3D vector class.
 *
 * This class provides a foundation for 3D vector operations, including arithmetic
 * (+, -, *, /), dot product, cross product, magnitude, and normalization. It uses the
 * Curiously Recurring Template Pattern (CRTP) to allow derived classes to return
 * their own type from these operations, enabling method chaining with the correct type.
 *
 * @tparam Numeric The numeric type for the vector's components (e.g., float, double, fixed-point).
 * @tparam Derived The derived class's type, used for CRTP. Defaults to `void`, in which
 *                 case the class methods return `Vector3D<Numeric>`.
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
	NumericType X_coord; ///< The X-coordinate of the vector.
	NumericType Y_coord; ///< The Y-coordinate of the vector.
	NumericType Z_coord; ///< The Z-coordinate of the vector.

	/**
	 * @brief Default constructor, initializes to a zero vector.
	 */
	constexpr explicit Vector3D() : X_coord(0), Y_coord(0), Z_coord(0) {}
	/**
	 * @brief Constructs a vector from its x, y, and z components.
	 */
	constexpr explicit Vector3D(NumericType X_coord, NumericType Y_coord, NumericType Z_coord) :
		X_coord(X_coord), Y_coord(Y_coord), Z_coord(Z_coord) {}

	/**
	 * @brief Checks if the vector is non-zero.
	 * @return `true` if any component is non-zero, `false` otherwise.
	 */
	operator bool() const { return X_coord || Y_coord || Z_coord; }

	/**
	 * @brief Adds two vectors component-wise.
	 * @return A new vector of `ClassType` representing the sum.
	 */
	ClassType operator+(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(X_coord + other.X_coord, Y_coord + other.Y_coord, Z_coord + other.Z_coord);
	}

	/**
	 * @brief Subtracts one vector from another component-wise.
	 * @return A new vector of `ClassType` representing the difference.
	 */
	ClassType operator-(const VectorCompatible<NumericType> auto& other) const {
		return ClassType(X_coord - other.X_coord, Y_coord - other.Y_coord, Z_coord - other.Z_coord);
	}

	/**
	 * @brief Multiplies the vector by a scalar.
	 * @return A new vector of `ClassType` with each component scaled.
	 */
	constexpr ClassType operator*(const NumericType& scalar) const {
		return ClassType(X_coord * scalar, Y_coord * scalar, Z_coord * scalar);
	}

	/**
	 * @brief Divides the vector by a scalar.
	 * @return A new vector of `ClassType` with each component scaled.
	 */
	ClassType operator/(NumericType scalar) const {
		return ClassType(X_coord / scalar, Y_coord / scalar, Z_coord / scalar);
	}

	/**
	 * @brief Computes the dot product of this vector with another.
	 * @return The scalar dot product.
	 */
	NumericType dot(const VectorCompatible<NumericType> auto& other) const {
		return X_coord * other.X_coord + Y_coord * other.Y_coord + Z_coord * other.Z_coord;
	}

	/**
	 * @brief Computes the cross product of this vector with another.
	 * @return A new vector of `ClassType` that is perpendicular to both input vectors.
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
	 * @brief Computes the normalized vector (unit vector).
	 * @return A new vector of `ClassType` with a magnitude of 1.
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
	 * Yaw is the angle in the XY plane relative to the Y-axis.
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
