#pragma once
#include "Vector3f.h"
#include "BxDF.h"
#include "BRDFUtils.h"

class BSDF
{
public:
	BSDF() = default;
	BSDF(const Vector3f& ns, const Vector3f& dpdus, std::shared_ptr<const BxDF> bxdf):
        bxdf(bxdf), shadingFrame(Frame::FromXZ(dpdus.normalize(), ns.normalize()))
    {
    }
    Vector3f RenderToLocal(const Vector3f& v) const
    {
        return shadingFrame.ToLocal(v);
    };

    Vector3f LocalToRender(const Vector3f& v) const
    {
        return shadingFrame.FromLocal(v);
    };
    Spectrum f(const Vector3f& woRender,const Vector3f& wiRender,
        TransportMode mode = TransportMode::Radiance) const
    {
		Vector3f woLocal = RenderToLocal(woRender);
		Vector3f wiLocal = RenderToLocal(wiRender);
        if (woLocal.z == 0)
            return{};

        return bxdf->f(woLocal, wiLocal,mode);
    };
    BSDFSample  Sample_f(const Vector3f& woRender,
        const Point2f& sample,TransportMode mode = TransportMode::Radiance 
    ,BxDFReflectionType sampledType = BxDFReflectionType::All) const
    {
		Vector3f woLocal = RenderToLocal(woRender);
        if(woLocal.z==0)
            return {};
		auto bs = bxdf->Sample_f(woLocal,sample,mode, sampledType);
		if (!bs || !bs->f||bs->pdf==0||bs->wi.z==0)
			return {};
		bs->wi = LocalToRender(bs->wi);
		return *bs;
    };
    float Pdf(const Vector3f& woRender, const Vector3f& wiRender,
        TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType sampleFlags = BxDFReflectionType::All) const
    {
		Vector3f wo = RenderToLocal(woRender);
		Vector3f wi = RenderToLocal(wiRender);
		if (wo.z == 0)
			return 0.0f;
		return bxdf->Pdf(wo, wi, mode, sampleFlags);    
    };
	bool hasFlags(BxDFType flags) const
	{
		return bxdf && bxdf->MatchesFlags(flags);
	}
	BxDFType Flags() const
	{
		return bxdf ? bxdf->type : BxDFType::Unset;
	}
private:
    std::shared_ptr<const BxDF> bxdf = nullptr;
    Frame shadingFrame;
};

