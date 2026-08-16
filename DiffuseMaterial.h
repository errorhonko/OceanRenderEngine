#pragma once
#include "material.h"
#include "DiffuseBxDF.h"
#include <memory> 
class DiffuseMaterial : public Material
{

public:
	DiffuseMaterial(Spectrum albedo) : albedo(albedo) {}

	BSDF GetBSDF(const MaterialEvalContext& ctx) const override
	{
		Spectrum r = albedo;
		bxdfStorage = std::make_shared<DiffuseBxDF>(r);
		return BSDF(ctx.ns, ctx.dpdu, bxdfStorage);
	}

private:
	Spectrum albedo;
	mutable std::shared_ptr<BxDF> bxdfStorage;
};

