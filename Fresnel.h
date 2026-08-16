#pragma once
#include "RGBSpectrum.h"

using Spectrum = RGBSpectrum;
class Fresnel
{
public:
	virtual Spectrum Evaluate(float cosThetaI) const = 0;
	virtual ~Fresnel() = default;
};

