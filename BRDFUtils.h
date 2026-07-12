#pragma once
#include "Vector3f.h"
#include	<cmath>
#include <algorithm>
using std::max;
namespace BRDFUtils
{
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
}


