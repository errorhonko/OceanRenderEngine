#pragma once
#include "core/math/Vector3f.h"
#include "reflection/bxdfs/BSDF.h"
#include <memory>
struct  MaterialEvalContext
{
	Vector3f p;
	Vector3f wo;
	Vector3f ns;
	Vector3f dpdu;
	float u, v;
};
class Material
{
public:
	virtual~Material() = default;
	virtual BSDF GetBSDF(const MaterialEvalContext& ctx) const = 0;
};

