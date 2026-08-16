#pragma once
#include "BxDF.h"
#include "TrowbridgeReitzDistribution.h"
class DielectricBxDF :public BxDF
{
public: 
	DielectricBxDF(float eta,
		const TrowbridgeReitzDistribution& distrib) 
		:BxDF(static_cast<BxDFType>(
			((eta==1)?0:BSDF_REFLECTION)|
			BSDF_TRANSMISSION|
			(distrib.EffectivelySmooth() ? BSDF_SPECULAR: BSDF_GLOSSY)
			)),
		eta(eta), mfDistrib(distrib){}
	std::optional<BSDFSample> Sample_f(const Vector3f& wo,
		const Point2f& u, TransportMode mode = TransportMode::Radiance,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All)const override
	{
		if (eta == 1 || mfDistrib.EffectivelySmooth())
		{
			float R = FrDielectric(CosTheta(wo), 1.0f, eta);
			float T = 1.0f - R;
			float pr = R, pt = T;
			if (!(sampleFlags & BxDFReflectionType::Reflection)) pr = 0;
			if (!(sampleFlags & BxDFReflectionType::Transmission))pt = 0;
			if (pr == 0 && pt == 0)
				return{};
			if (u.x < pr / (pr + pt))
			{
				Vector3f wi(-wo.x, -wo.y, wo.z);
				Spectrum fr(R / AbsCosTheta(wi));
				return BSDFSample(fr, wi, pr / (pr + pt), BxDFType::SpecularReflection);
			}
			else
			{
				Vector3f wi;
				float etap;
				Vector3f n(0, 0, 1.0f);
				bool valid = Refract(wo, n, eta, &etap, &wi);
				if (!valid) return{};
				Spectrum ft(T / AbsCosTheta(wi));
				if (mode == TransportMode::Radiance)
					ft = ft / Sqr(etap);
				return BSDFSample(ft, wi, pt / (pr + pt),
					BxDFType::SpecularTransmission, etap);
			}
		}
			else
			{
				float R_est = FrDielectric(CosTheta(wo), 1.0f, eta);
				float pr = R_est, pt = 1.0f - R_est;
				if (!(sampleFlags & BxDFReflectionType::Reflection))
					pr = 0.0f;
				if (!(sampleFlags & BxDFReflectionType::Transmission))
					pt = 0.0f;
				if (pr == 0.0f && pt == 0.0f)return std::nullopt;

				float threshod = pr / (pr + pt);
				bool doReflect = u.x < threshod;
				float ucRemap = RemapSample(u.x, threshod);
				Point2f uForWm(ucRemap, u.y);
		
				Vector3f wm = mfDistrib.Sample_Wm(wo, uForWm);
				float R = FrDielectric(wo.dot(wm), 1.0f, eta);
				float T = 1 - R;
				float pr = R, pt = T;
				if(!(sampleFlags&BxDFReflectionType::Reflection))
				{
					pr = 0;
				}
				if(!(sampleFlags&BxDFReflectionType::Transmission))
				{
					pt = 0;
				}
				if (pr == 0 && pt == 0)
					return {};


				float pdf;
				if (doReflect)
				{
					Vector3f wi = Reflect(wo, wm);
					if (!SameHemisphere(wo, wi))return{};
					pdf = mfDistrib.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);

					Spectrum f(mfDistrib.D(wm) * mfDistrib.G(wo, wi) * R
						/ (4 * CosTheta(wi) * CosTheta(wo)));
					return BSDFSample(f, wi, pdf, BxDFType::GlossyReflection);
				}
				else
				{
					float etap;
					Vector3f wi;
					bool tir = !Refract(wo, wm, eta, &etap, &wi);
					if (SameHemisphere(wo, wi) || wi.z == 0 || tir)
						return{};
					float denom = Sqr(wi.dot(wm) + wo.dot(wm) / etap);
					float dwm_dwi = AbsDot(wi, wm) / denom;
					pdf = mfDistrib.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
					Spectrum ft(T * mfDistrib.D(wm) * mfDistrib.G(wo, wi) *
						std::abs(wi.dot(wm) * wo.dot(wm) / (CosTheta(wi) * CosTheta(wo) * denom)));
					if (mode == TransportMode::Radiance)ft = ft / Sqr(etap);
					return BSDFSample(ft, wi, pdf, BxDFType::GlossyTransmission, etap);

				}
		}
	}
	
	Spectrum f(const Vector3f& wo, const Vector3f& wi ,
		TransportMode mode)const override
	{
		if (eta == 1 || mfDistrib.EffectivelySmooth())
		{
			return Spectrum(0.0f);
		}
		float cosTheta_o = CosTheta(wo);
		float cosTheta_i = CosTheta(wi);
		bool reflect = SameHemisphere(wo, wi);
		float etap = 1;
		if(!reflect)
			etap = (cosTheta_o > 0) ? eta : 1 / eta;
		Vector3f wm = wi * etap + wo;
		if(cosTheta_i==0||cosTheta_o==0||wm.near_zero())
		{
			return{};
		}
		wm = FaceForward(wm.normalize(), Vector3f(0, 0, 1));

		if (wm.dot(wi) * cosTheta_i < 0 || wm.dot(wo) * cosTheta_o < 0)
		{
			return{};
		}
		float F = FrDielectric(wo.dot(wm), 1.0f, eta);
		if (reflect)
		{
			return Spectrum(mfDistrib.D(wm) * mfDistrib.G(wo, wi) * F
				/std ::abs(4 * cosTheta_i * cosTheta_o));
		}
		else
		{
			float denom = Sqr(wi.dot(wm) + wo.dot(wm) / etap)*cosTheta_i*cosTheta_o;
			float ft = mfDistrib.D(wm) * mfDistrib.G(wo, wi) * (1 - F)
				*std::abs(wi.dot(wm) * wo.dot(wm)  / denom);
			if (mode == TransportMode::Radiance)
				ft /= Sqr(etap);
			return Spectrum(ft);
		}
	}

	float Pdf(const Vector3f& wo, const Vector3f& wi, TransportMode mode,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All)const override
	{
		if (eta == 1 || mfDistrib.EffectivelySmooth())
		{
			return (0.0f);
		}
		float cosTheta_o = CosTheta(wo);
		float cosTheta_i = CosTheta(wi);
		bool reflect = SameHemisphere(wo, wi);
		float etap = 1;
		if (!reflect)
			etap = (cosTheta_o > 0) ? eta : 1 / eta;
		Vector3f wm = wi * etap + wo;
		if (cosTheta_i == 0 || cosTheta_o == 0 || wm.near_zero())
		{
			return(0.0f);
		}
		wm = FaceForward(wm.normalize(), Vector3f(0, 0, 1));

		if (wm.dot(wi) * cosTheta_i < 0 || wm.dot(wo) * cosTheta_o < 0)
		{
			return(0.0f);
		}

		float R = FrDielectric(wo.dot(wm), 1.0f, eta);
		float T = 1 - R;
		float pr = R, pt = T;
		if (!(sampleFlags & BxDFReflectionType::Reflection))
		{
			pr = 0.0f;
		}
		if (!(sampleFlags & BxDFReflectionType::Transmission))
		{
			pt = 0.0f;
		}
		if (pr == 0.0f && pt == 0.0f)
		{
			return(0.0f);
		}
		float pdf;
		if (reflect)
		{
			pdf = mfDistrib.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);
		}
		else
		{
			float denom = Sqr(wi.dot(wm) + wo.dot(wm) / etap);
			float dwm_dwi = AbsDot(wi, wm) / denom;
			pdf = mfDistrib.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
		}
		return pdf;
	}
	void Regularize() { mfDistrib.Regularize(); }

private:
	float eta;
	TrowbridgeReitzDistribution mfDistrib;
};

