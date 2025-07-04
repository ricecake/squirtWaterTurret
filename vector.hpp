#pragma once

#include <type_traits>
#include <concepts>

template <typename T, typename U>
concept Number = requires (T obj, U thi) {
	requires std::is_arithmetic_v<T> || requires {
		obj + thi;
		obj - thi;
		obj * thi;
		obj / thi;
	};
};
/*
If it feels worth it to make this actually type agnostic, can add a template parameter that accepts
a struct containing the functions that do the different sqrt/etc methods.
The default for the types could be fiX_coorded and the fiX_coorded sqrt, and it could also be filled in for float and double.
*/

template <typename Numeric, typename Derived>
// requires std::is_arithmetic_v<Numeric>
class Vector3D;

template <typename T, typename Numeric>
concept VectorCompatible = requires(T vec) {
	{ vec.X_coord } -> std::same_as<Numeric &>;
	{ vec.Y_coord } -> std::same_as<Numeric &>;
	{ vec.Z_coord } -> std::same_as<Numeric &>;
};

template <typename ExplicitType, typename Default>
using Either = std::conditional_t<std::is_same_v<ExplicitType, void>, Default, ExplicitType>;

template <typename Numeric, typename Derived = void>
// requires std::is_arithmetic_v<Numeric>
class Vector3D
{
public:
	using NumericType = Numeric;
	using ClassType = Either<Derived, Vector3D<NumericType>>;
	constexpr static NumericType rad2DegFactor = NumericType(57.2957795131);

public:
	NumericType X_coord;
	NumericType Y_coord;
	NumericType Z_coord;

	// Constructors
	constexpr Vector3D() : X_coord(0), Y_coord(0), Z_coord(0) {}
	constexpr Vector3D(NumericType X_coord, NumericType Y_coord, NumericType Z_coord) : X_coord(X_coord), Y_coord(Y_coord), Z_coord(Z_coord) {}

	operator bool() const
	{
		return X_coord && Y_coord && Z_coord;
	}

	// Vector addition
	ClassType operator+(const VectorCompatible<NumericType> auto &other) const
	{
		return ClassType(X_coord + other.X_coord, Y_coord + other.Y_coord, Z_coord + other.Z_coord);
	}

	// Vector subtraction
	ClassType operator-(const VectorCompatible<NumericType> auto &other) const
	{
		return ClassType(X_coord - other.X_coord, Y_coord - other.Y_coord, Z_coord - other.Z_coord);
	}

	// Scalar multiplication
	constexpr ClassType operator*(const NumericType& scalar) const
	{
		return ClassType(X_coord * scalar, Y_coord * scalar, Z_coord * scalar);
	}

	// Scalar division
	ClassType operator/(NumericType scalar) const
	{
		return ClassType(X_coord / scalar, Y_coord / scalar, Z_coord / scalar);
	}

	// Dot product
	NumericType dot(const VectorCompatible<NumericType> auto &other) const
	{
		return X_coord * other.X_coord + Y_coord * other.Y_coord + Z_coord * other.Z_coord;
	}

	// Cross product
	ClassType cross(const VectorCompatible<NumericType> auto &other) const
	{
		return ClassType(
			Y_coord * other.Z_coord - Z_coord * other.Y_coord,
			Z_coord * other.X_coord - X_coord * other.Z_coord,
			X_coord * other.Y_coord - Y_coord * other.X_coord);
	}

	// Magnitude
	NumericType magnitude() const
	{
		return sqrt(X_coord * X_coord + Y_coord * Y_coord + Z_coord * Z_coord);
	}

	// Magnitude XY
	NumericType magnitudeXY() const
	{
		return sqrt(X_coord * X_coord + Y_coord * Y_coord);
	}

	// Magnitude XZ
	NumericType magnitudeXZ() const
	{
		return sqrt(X_coord * X_coord + Z_coord * Z_coord);
	}

	// Magnitude YZ
	NumericType magnitudeYZ() const
	{
		return sqrt(Y_coord * Y_coord + Z_coord * Z_coord);
	}

	// Normalize
	ClassType normalize() const
	{
		NumericType mag = magnitude();
		if (mag != 0)
		{
			return ClassType(X_coord / mag, Y_coord / mag, Z_coord / mag);
		}
		return *this;
	}

	// Yaw
	NumericType yaw() const
	{
		return atan2(Z_coord, magnitudeXY()) * rad2DegFactor;
	}

	// Pitch
	NumericType pitch() const
	{
		return atan2(X_coord, Y_coord) * rad2DegFactor;
	}
};
