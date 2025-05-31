#include "fpm/math.hpp"

/*
If it feels worth it to make this actually type agnostic, can add a template parameter that accepts
a struct containing the functions that do the different sqrt/etc methods.
The default for the types could be fixed and the fixed sqrt, and it could also be filled in for float and double.
*/

template <typename T>
class Vector3D {
public:
    T x, y, z;

    // Constructors
    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(T x, T y, T z) : x(x), y(y), z(z) {}

    // Vector addition
    Vector3D<T> operator+(const Vector3D<T>& other) const {
        return Vector3D<T>(x + other.x, y + other.y, z + other.z);
    }

    // Vector subtraction
    Vector3D<T> operator-(const Vector3D<T>& other) const {
        return Vector3D<T>(x - other.x, y - other.y, z - other.z);
    }

    // Scalar multiplication
    Vector3D<T> operator*(T scalar) const {
        return Vector3D<T>(x * scalar, y * scalar, z * scalar);
    }

    // Dot product
    T dot(const Vector3D<T>& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    Vector3D<T> cross(const Vector3D<T>& other) const {
        return Vector3D<T>(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Magnitude
    T magnitude() const {
        return fpm::sqrt(x * x + y * y + z * z);
    }

    // Normalize
    Vector3D<T> normalize() const {
        T mag = magnitude();
        if (mag != 0) {
            return Vector3D<T>(x / mag, y / mag, z / mag);
        }
        return *this;
    }
};
