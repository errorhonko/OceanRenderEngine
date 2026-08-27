#pragma once
#include "Vector3f.h"
#include	<cmath>
#include <algorithm>
#include "Point2f.h"
using std::max;
constexpr float Pi = 3.14159265358979323846;
constexpr float InvPi = 0.31830988618379067154;
constexpr float Inv2Pi = 0.15915494309189533577;
constexpr float Inv4Pi = 0.07957747154594766788;
constexpr float PiOver2 = 1.57079632679489661923;
constexpr float PiOver4 = 0.78539816339744830961;
constexpr float Sqrt2 = 1.41421356237309504880;
namespace BRDFUtils
{
	inline float Sqr(float v)
	{
		return v * v;
	}
	inline float AbsDot(const Vector3f& a, const Vector3f& b)
	{
		return std::fabs(a.dot(b));
	}
	inline float CosTheta(const Vector3f& w)
	{
		return w.z;
	}
	inline float Cos2Theta(const Vector3f& w)
	{
		return w.z * w.z;
	}
	inline float AbsCosTheta(const Vector3f& w)
	{
		return std::fabs(w.z);
	}
	inline float Sin2Theta(const Vector3f& w)
	{
		return std::max(0.0f, 1.0f - Cos2Theta(w));
	}
	inline float SinTheta(const Vector3f& w)
	{
		return std::sqrt(Sin2Theta(w));
	}
	inline float TanTheta(const Vector3f& w)
	{
		return SinTheta(w) / CosTheta(w);
	}
	inline float Tan2Theta(const Vector3f& w)
	{
		return Sin2Theta(w) / Cos2Theta(w);
	}
	inline float CosPhi(const Vector3f& w)
	{
		float sinTheta = SinTheta(w);
		return (sinTheta == 0.0f) ? 1.0f : std::clamp(w.x / sinTheta, -1.0f, 1.0f);
	}
	inline float SinPhi(const Vector3f& w)
	{
		float sinTheta = SinTheta(w);
		return (sinTheta == 0.0f) ? 0.0f : std::clamp(w.y / sinTheta, -1.0f, 1.0f);
	}
	inline float Cos2Phi(const Vector3f& w)
	{
		return CosPhi(w) * CosPhi(w);
	}
	inline float Sin2Phi(const Vector3f& w)
	{
		return SinPhi(w) * SinPhi(w);
	}
	inline float cosDPhi(const Vector3f& wa, const Vector3f& wb)
	{
		float denom = std::sqrt((wa.x * wa.x + wa.y * wa.y) * (wb.x * wb.x + wb.y * wb.y));
		if (denom == 0.0f) return 1.0f;
		return std::clamp(wa.x * wb.x + wa.y * wb.y, -1.0f, 1.0f);
	}
	inline bool SameHemisphere(const Vector3f& w, const Vector3f& wp)
	{
		return w.z * wp.z > 0.0f;
	}
	inline float Safesqrt(float v)
	{
		return std::sqrt(std::max(0.0f, v));
	}
	inline float CosineHemispherePdf(float cosTheta)
	{
		return cosTheta * InvPi;
	}
	inline Point2f SampleUniformDiskConcentric(Point2f u)
	{
		Point2f uOffset = 2.f * u - Point2f(1, 1);
		if (uOffset.x == 0 && uOffset.y == 0)
			return Point2f(0, 0);
		float theta, r;
		if (std::fabs(uOffset.x) > std::fabs(uOffset.y))
		{
			r = uOffset.x;
			theta = (PiOver4) * (uOffset.y / uOffset.x);
		}
		else
		{
			r = uOffset.y;
			theta = (PiOver2)-(PiOver4) * (uOffset.x / uOffset.y);
		}
		return r * Point2f(std::cos(theta), std::sin(theta));
	}
	inline Point2f SampleUniformDiskPolar(const Point2f& u)
	{
		float r = std::sqrt(u.x);
		float theta = 2 * Pi * u.y;
		return Point2f(r * std::cos(theta), r * std::sin(theta));
	}
	inline Vector3f SampleCosineHemisphere(const Point2f& u)
	{
		Point2f d = SampleUniformDiskConcentric(u);
		float z = Safesqrt(1 - d.x * d.x - d.y * d.y);
		return Vector3f(d.x, d.y, z);
	}
	inline Vector3f SampleUniformSphere(const Point2f& u)
	{
		float z = 1.0f - 2.0f * u.x;
		float r = Safesqrt(1.0f - z * z);
		float phi = 2.0f * Pi * u.y;

		return Vector3f(
			r * std::cos(phi),
			r * std::sin(phi),
			z);
	}

	inline float UniformSpherePdf()
	{
		return Inv4Pi;
	}

	inline Vector3f FaceForward(Vector3f n, Vector3f v)
	{
		return (n.dot(v) < 0.0f) ? -n : n;

	}
	inline float RemapSample(float u, float threshold)
	{
		if (u < threshold)
			return u / threshold;
		else
			return (u - threshold) / (1.0f - threshold);
	}
	
	
}
	

enum class TransportMode { Radiance, Importance };

struct Frame
{
	Vector3f x, y, z;
	Frame() : x(1, 0, 0), y(0, 1, 0), z(0, 0, 1) {}
	Frame(const Vector3f& x, const Vector3f& y, const Vector3f& z) : x(x), y(y), z(z) {}
	static Frame FromXZ(const Vector3f& x, const Vector3f& z)
	{
		Vector3f y = z.cross(x).normalize();
		return Frame(y.cross(z).normalize(), y, z);
	}
	static Frame FromZ(const Vector3f& z)
	{
		Vector3f zNormal = z.normalize();
		Vector3f x, y;
		if (std::fabs(zNormal.x) > std::fabs(zNormal.y))
		{
			float invLen = 1.0f / std::sqrt(zNormal.x * zNormal.x + zNormal.z * zNormal.z);
			x = Vector3f(-zNormal.z * invLen, 0.0f, zNormal.x * invLen);
		}
		else
		{
			float invLen = 1.0f / std::sqrt(zNormal.y * zNormal.y + zNormal.z * zNormal.z);
			x = Vector3f(0.0f, zNormal.z * invLen, -zNormal.y * invLen);
		}
		y = zNormal.cross(x).normalize();
		return Frame(x, y, zNormal);
	}
	Vector3f ToLocal(const Vector3f& v) const
	{
		return Vector3f(v.dot(x), v.dot(y), v.dot(z));
	}

	Vector3f FromLocal(const Vector3f& v) const
	{
		return x * v.x + y * v.y + z * v.z;
	}

};
