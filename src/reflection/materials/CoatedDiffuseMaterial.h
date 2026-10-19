#pragma once
#include "reflection/materials/material.h"
#include "reflection/bxdfs/CoatedDiffuseBxDF.h"

// Neutral visible material, explicit scalar base albedo. Spectral LiDAR remains
// CoatedDiffuseLidarScattering; no automatic MTL or NIR-to-RGB reinterpretation.
class CoatedDiffuseMaterial final : public Material
{
public:
    CoatedDiffuseMaterial(double etaIncident,double etaCoating,float alphaX,float alphaY,
        double baseReflectance,double opticalThickness=0,LayeredReflectionOptions options={})
        : bxdf(std::make_shared<CoatedDiffuseBxDF>(etaIncident,etaCoating,alphaX,alphaY,
            baseReflectance,opticalThickness,options)) {}
    BSDF GetBSDF(const MaterialEvalContext& ctx) const override
    {return BSDF(ctx.ns,ctx.dpdu,bxdf);}
private:
    std::shared_ptr<const CoatedDiffuseBxDF> bxdf;
};
