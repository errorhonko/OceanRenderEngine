#include "BxDF.h"
Spectrum BxDF::rho(const Vector3f& wo, int nSamples, const Point2f* samples) const
{
	Spectrum r(0.f);
	for (int i = 0; i < nSamples; ++i)
	{
		
		Vector3f wi;
		float pdf;
		BxDFType sampledType;
		if (auto bs = Sample_f(wo, samples[i]);bs&&bs->pdf>0.f)
		{
			
			
			r  = r+ bs->f;
		}
	}
	return r / static_cast<float>(nSamples);
}
