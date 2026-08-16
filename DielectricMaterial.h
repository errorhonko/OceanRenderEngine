#pragma once
#include "material.h"
#include "DielectricBxDF.h"
#include "TrowbridgeReitzDistribution.h"
#include <memory>
class DielectricMaterial : public Material
{
public:
	DielectricMaterial(float eta, float uRoughness,
		float vRoughness, bool remapRoughness =true)
		: eta(eta), uRoughness(uRoughness), 
		vRoughness(vRoughness), 
		remapRoughness(remapRoughness) {
	}

	BSDF GetBSDF(const MaterialEvalContext& ctx) const override
	{
		float ur = uRoughness, vr = vRoughness;
		if (remapRoughness)
		{
			ur = TrowbridgeReitzDistribution::RoughnessToAlpha(ur);
			vr = TrowbridgeReitzDistribution::RoughnessToAlpha(vr);
		}
		TrowbridgeReitzDistribution distrib(ur, vr);
		auto bxdfStorage = std::make_shared<DielectricBxDF>(eta, distrib);	
		return BSDF(ctx.ns, ctx.dpdu, bxdfStorage);
	}

private:
	float eta;
	float uRoughness, vRoughness;
	bool remapRoughness;

};