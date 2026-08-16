#pragma once
#include "BxDF.h"
class TrowbridgeReitzDistribution
{
public:
	TrowbridgeReitzDistribution(float alpha_x, float alpha_y) : alpha_x(alpha_x), alpha_y(alpha_y) {}

	float  D(const Vector3f& wm) const 
	{
		float tan2Theta = BRDFUtils::Tan2Theta(wm);
		if (std::isinf(tan2Theta)) return 0.0f;
		float cos4Theta = BRDFUtils::Cos2Theta(wm) * BRDFUtils::Cos2Theta(wm);
		float e = tan2Theta * (Sqr(CosPhi(wm) / alpha_x) + Sqr(SinPhi(wm) / alpha_y));
		return 1.0f / (Pi * alpha_x * alpha_y * cos4Theta * Sqr(1.0f + e));

	}
	//VNDF
	float D(Vector3f w, Vector3f wm) const
	{
		return G1(w) / AbsCosTheta(w) * D(wm) * AbsDot(w, wm);
	}
	bool EffectivelySmooth() const
	{
		return std::max(alpha_x, alpha_y) < 1e-3f;
	}
	float G1(Vector3f w)const {
		return 1 / (1 + Lambda(w));
	}
	float Lambda(Vector3f w) const {
		float tan2Theta = BRDFUtils::Tan2Theta(w);
		if (std::isinf(tan2Theta)) return 0.0f;
		float alpha2 = Sqr(CosPhi(w) * alpha_x )+Sqr(SinPhi(w) * alpha_y);
		return (std::sqrt(1.0f + alpha2 * tan2Theta) - 1.0f) / 2.0f;
	}
	float G(Vector3f wo, Vector3f wi) const {
		return 1 / (1 + Lambda(wo) + Lambda(wi));
	}
	float PDF(Vector3f w, Vector3f wm) const
	{
		return D(w, wm);
	}
	Vector3f Sample_Wm(Vector3f w, Point2f u) const
	{
		Vector3f wh = Vector3f(alpha_x * w.x, alpha_y * w.y, w.z).normalize();
		if (wh.z < 0)
		{
			wh = -wh;
		}
		Vector3f T1 = (wh.z < 0.99999f) ? Vector3f(-wh.y, wh.x, 0).normalize() : Vector3f(1, 0, 0);
		Vector3f T2 = wh.cross(T1);

		Point2f p = SampleUniformDiskPolar(u);

		float h = std::sqrt(1 - Sqr(p.x));
		p.y = std::lerp(h, p.y, (1 + wh.z) / 2);

		float pz = std::sqrt(std::max(0.0f, 1.0f - Sqr(p.x) - Sqr(p.y)));
		Vector3f nh = p.x * T1 + p.y * T2 + pz * wh;
		return Vector3f(alpha_x * nh.x , alpha_y * nh.y, std::max(1e-6f, nh.z)).normalize();
	}
	static float RoughnessToAlpha(float roughness)
	{
		return std::sqrt(roughness);
	}
	void Regularize()
	{
		if (alpha_x < 0.3f) alpha_x = clamp(2 * alpha_x, 0.1f, 0.3f);
		if (alpha_y < 0.3f) alpha_y = clamp(2 * alpha_y, 0.1f, 0.3f);
	}
private:
	float  alpha_x, alpha_y;
};