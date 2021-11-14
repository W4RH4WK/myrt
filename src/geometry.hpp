#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cassert>
#include <cmath>
#include <optional>
#include <ostream>

#include "random.hpp"

constexpr float PI = M_PI;

constexpr auto deg2rad = (PI * 2.0f) / 360.0f;
constexpr auto rad2deb = 1.0f / deg2rad;

//////////////////////////////////////////////////////////////////////

struct Vec3 {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

	float lengthSquared() const { return x * x + y * y + z * z; }
	float length() const { return std::sqrt(lengthSquared()); }

	Vec3 normalized() const { return Vec3{x / length(), y / length(), z / length()}; }
	void normalize() { *this = normalized(); }

	static const Vec3 zero, one;
	static const Vec3 up, right, forward;
};

inline const Vec3 Vec3::zero = {0.0f, 0.0f, 0.0f};
inline const Vec3 Vec3::one = {1.0f, 1.0f, 1.0f};
inline const Vec3 Vec3::up = {0.0f, 1.0f, 0.0f};
inline const Vec3 Vec3::right = {1.0f, 0.0f, 0.0f};
inline const Vec3 Vec3::forward = {0.0f, 0.0f, 1.0f};

static inline Vec3 operator+(const Vec3 &a, const Vec3 &b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline Vec3 operator+=(Vec3 &a, const Vec3 &b)
{
	return a = a + b;
}

static inline Vec3 operator-(const Vec3 &v)
{
	return {-v.x, -v.y, -v.z};
}

static inline Vec3 operator-(const Vec3 &a, const Vec3 &b)
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline Vec3 &operator-=(Vec3 &a, const Vec3 &b)
{
	return a = a - b;
}

static inline Vec3 operator*(float s, const Vec3 &v)
{
	return {s * v.x, s * v.y, s * v.z};
}

static inline Vec3 operator*(const Vec3 &a, const Vec3 &b)
{
	return {a.x * b.x, a.y * b.y, a.z * b.z};
}

static inline Vec3 &operator*=(Vec3 &v, float s)
{
	return v = s * v;
}

static inline Vec3 &operator*=(Vec3 &a, const Vec3 &b)
{
	return a = a * b;
}

static inline Vec3 operator/(const Vec3 &v, float s)
{
	return {v.x / s, v.y / s, v.z / s};
}

static inline Vec3 operator/(const Vec3 &a, const Vec3 &b)
{
	return {a.x / b.x, a.y / b.y, a.z / b.z};
}

static inline Vec3 pow(const Vec3 &v, float e)
{
	return {std::pow(v.x, e), std::pow(v.y, e), std::pow(v.z, e)};
}

static inline const Vec3 &max(const Vec3 &a, const Vec3 &b)
{
	return a.lengthSquared() > b.lengthSquared() ? a : b;
}

static inline float dot(const Vec3 &a, const Vec3 &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
	return {
	    a.y * b.z - a.z * b.y, //
	    a.z * b.x - a.x * b.z, //
	    a.x * b.y - a.y * b.x  //
	};
}

// static inline float determinant(const Vec3 &a, const Vec3 &b, const Vec3 &c)
// {
// 	return dot(-cross(a, b), c);
// }

static inline Vec3 mix(const Vec3 &a, const Vec3 &b, float t)
{
	return (1.0f - t) * a + t * b;
}

static inline Vec3 reflect(const Vec3 &in, const Vec3 &normal)
{
	return in - 2.0f * dot(in, normal) * normal;
}

static inline Vec3 refract(const Vec3 &i, const Vec3 &n, float eta)
{
	eta = 2.0f - eta;
	const auto cosi = dot(n, i);
	return (eta * i - (eta * cosi - cosi) * n);
}

// static inline std::optional<Vec3> refract(const Vec3 &i, const Vec3 &n, float eta)
// {
// 	const auto k = 1.0f - eta * eta * (1.0f - dot(n, i) * dot(n, i));
// 	if (k < 0.0f)
// 		return {};
// 	return eta * i - (eta * dot(n, i) + std::sqrt(k)) * n;
// }

static inline std::ostream &operator<<(std::ostream &out, const Vec3 &v)
{
	return out << v.x << " " << v.y << " " << v.z;
}

//////////////////////////////////////////////////////////////////////

struct Quat {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	float w = 1.0f;

	Quat() = default;

	Quat(float rotation_x, float rotation_y, float rotation_z)
	{
		// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code

		float cy = cos(rotation_z * 0.5f);
		float sy = sin(rotation_z * 0.5f);
		float cp = cos(rotation_y * 0.5f);
		float sp = sin(rotation_y * 0.5f);
		float cr = cos(rotation_x * 0.5f);
		float sr = sin(rotation_x * 0.5f);

		w = cr * cp * cy + sr * sp * sy;
		x = sr * cp * cy - cr * sp * sy;
		y = cr * sp * cy + sr * cp * sy;
		z = cr * cp * sy - sr * sp * cy;
	}

	static Quat random(float angle = 0.0f)
	{
		return Quat{randFloatNormal() * angle, randFloatNormal() * angle, randFloatNormal() * angle};
	}
};

static inline Vec3 operator*(const Quat &q, const Vec3 &v)
{
	const auto u = Vec3{q.x, q.y, q.z};
	return 2.0f * dot(u, v) * u + (q.w * q.w - dot(u, u)) * v + 2.0f * q.w * cross(u, v);
}

static inline std::ostream &operator<<(std::ostream &out, const Quat &q)
{
	return out << q.x << " " << q.y << " " << q.z << " " << q.w;
}

//////////////////////////////////////////////////////////////////////

struct Ray {
	Ray(Vec3 origin, Vec3 direction) : origin(origin), direction(direction.normalized()) {}

	Vec3 origin;
	Vec3 direction;
};

//////////////////////////////////////////////////////////////////////

// https://learnopengl.com/PBR/Theory

static inline float distributionGGX(const Vec3 &N, const Vec3 &H, float a)
{
	const auto a2 = a * a;
	const auto NdotH = std::max(dot(N, H), 0.0f);
	const auto NdotH2 = NdotH * NdotH;

	const auto nom = a2;
	auto denom = (NdotH2 * (a2 - 1.0f) + 1.0f);
	denom = PI * denom * denom;

	return nom / denom;
}

static inline float geometrySchlickGGX(float NdotV, float k)
{
	const auto nom = NdotV;
	const auto denom = NdotV * (1.0f - k) + k;

	return nom / denom;
}

static inline float geometrySmith(const Vec3 &N, const Vec3 &V, const Vec3 &L, float k)
{
	const auto NdotV = std::max(dot(N, V), 0.0f);
	const auto NdotL = std::max(dot(N, L), 0.0f);
	const auto ggx1 = geometrySchlickGGX(NdotV, k);
	const auto ggx2 = geometrySchlickGGX(NdotL, k);

	return ggx1 * ggx2;
}

static inline Vec3 fresnelSchlick(float cosTheta, const Vec3 &F0)
{
	return F0 + std::pow(1.0f - cosTheta, 5.0f) * (Vec3::one - F0);
}

#endif // GEOMETRY_HPP
