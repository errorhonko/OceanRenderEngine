#pragma once
#include "material.h"
#include "Texture.h"
#include <memory>
#include <cstdint>
#include "HashUtils.h"
#include <algorithm> 
#include <utility> 
class MixMaterial : public Material
{
public:
	MixMaterial(std::shared_ptr<Material> m1, 
		std::shared_ptr<Material> m2,
		std::shared_ptr<FloatTexture> mix,
		std::uint64_t seed )
		:hashSeed(seed),
		materials{ std::move(m1), std::move(m2)}, amount(std::move(mix)) {}
	std::shared_ptr<Material> ChooseMaterial
	(const MaterialEvalContext& ctx) const
	{
		float amt = std::clamp(amount->value(ctx.u, ctx.v), 
			0.0f, 1.0f);
		if (amt <= 0.0f)
			return materials[0];
		else if (amt >= 1.0f)
			return materials[1];
		else
		{
			float hash = HashUtils::HashFloat(ctx.p, ctx.wo,hashSeed);
			if (hash < amt)
				return materials[1];
			else
				return materials[0];
		}
	}
	std::shared_ptr<Material> GetMaterial(int index) const
	{
		return materials[index];
	}

	BSDF GetBSDF(const MaterialEvalContext& ctx) const override
	{
		auto mat = ChooseMaterial(ctx);
		if (mat)
			return mat->GetBSDF(ctx);
		else
			return {};
	}
private:
	
	std::uint64_t hashSeed;
	std::shared_ptr<Material> materials[2];
	std::shared_ptr<FloatTexture> amount;
};
