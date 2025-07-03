#pragma once

#include<type_traits>
/*
If it feels worth it to make this actually type agnostic, can add a template parameter that accepts
a struct containing the functions that do the different sqrt/etc methods.
The default for the types could be fiX_coorded and the fiX_coorded sqrt, and it could also be filled in for float and double.
*/

template <typename T, typename D>
using Either = std::conditional_t<std::is_same_v<T, void>, D, T>;

template <typename T, typename Der = void>
class Vector3D
{
public:
	T X_coord;
	T Y_coord;
	T Z_coord;

	// Constructors
	Vector3D() : X_coord(0), Y_coord(0), Z_coord(0) {}
	Vector3D(T X_coord, T Y_coord, T Z_coord) : X_coord(X_coord), Y_coord(Y_coord), Z_coord(Z_coord) {}

	operator bool() const
	{
		return X_coord && Y_coord && Z_coord;
	}

	// Vector addition
	Either<Der, Vector3D<T>> operator+(const Either<Der, Vector3D<T>> &other) const
	{
		return Either<Der, Vector3D<T>>(X_coord + other.X_coord, Y_coord + other.Y_coord, Z_coord + other.Z_coord);
	}

	// Vector subtraction
	Either<Der, Vector3D<T>> operator-(const Either<Der, Vector3D<T>> &other) const
	{
		return Either<Der, Vector3D<T>>(X_coord - other.X_coord, Y_coord - other.Y_coord, Z_coord - other.Z_coord);
	}

	// Scalar multiplication
	Either<Der, Vector3D<T>> operator*(T scalar) const
	{
		return Either<Der, Vector3D<T>>(X_coord * scalar, Y_coord * scalar, Z_coord * scalar);
	}

	// Scalar division
	Either<Der, Vector3D<T>> operator/(T scalar) const
	{
		return Either<Der, Vector3D<T>>(X_coord / scalar, Y_coord / scalar, Z_coord / scalar);
	}

	// Dot product
	T dot(const Either<Der, Vector3D<T>> &other) const
	{
		return X_coord * other.X_coord + Y_coord * other.Y_coord + Z_coord * other.Z_coord;
	}

	// Cross product
	Either<Der, Vector3D<T>> cross(const Either<Der, Vector3D<T>> &other) const
	{
		return Either<Der, Vector3D<T>>(
			Y_coord * other.Z_coord - Z_coord * other.Y_coord,
			Z_coord * other.X_coord - X_coord * other.Z_coord,
			X_coord * other.Y_coord - Y_coord * other.X_coord);
	}

	// Magnitude
	T magnitude() const
	{
		return sqrt(X_coord * X_coord + Y_coord * Y_coord + Z_coord * Z_coord);
	}

	// Normalize
	Either<Der, Vector3D<T>> normalize() const
	{
		T mag = magnitude();
		if (mag != 0)
		{
			return Either<Der, Vector3D<T>>(X_coord / mag, Y_coord / mag, Z_coord / mag);
		}
		return *this;
	}
};
