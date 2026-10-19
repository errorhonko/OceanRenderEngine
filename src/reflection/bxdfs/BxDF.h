#pragma once
#include "core/spectrum/CoefficientSpectrum.h"
#include "core/spectrum/RGBSpectrum.h"
#include "core/math/Vector3f.h"
#include "core/math/Point2f.h"
#include "reflection/microfacet/BRDFUtils.h"
#include "reflection/fresnel/FresnelUtils.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <complex>
using std::clamp;
using namespace BRDFUtils;
using Spectrum = RGBSpectrum;
enum BxDFType
{
	Unset = 0,
	BSDF_REFLECTION = 1 << 0,
	BSDF_TRANSMISSION = 1 << 1,
	BSDF_DIFFUSE = 1 << 2,
	BSDF_GLOSSY = 1 << 3,
	BSDF_SPECULAR = 1 << 4,
	DiffuseReflection = BSDF_DIFFUSE | BSDF_REFLECTION,
	DiffuseTransmission = BSDF_DIFFUSE | BSDF_TRANSMISSION,
	GlossyReflection = BSDF_GLOSSY | BSDF_REFLECTION,
	GlossyTransmission = BSDF_GLOSSY | BSDF_TRANSMISSION,
	SpecularReflection = BSDF_SPECULAR | BSDF_REFLECTION,
	SpecularTransmission = BSDF_SPECULAR | BSDF_TRANSMISSION,
	BSDF_ALL_TYPES = BSDF_DIFFUSE | BSDF_GLOSSY |
	BSDF_SPECULAR | BSDF_REFLECTION | BSDF_TRANSMISSION,
};

inline bool IsNonSpecular(BxDFType type)
{
	int value = static_cast<int>(type);
	return (value & (BSDF_DIFFUSE | BSDF_GLOSSY)) != 0;
}
enum class BxDFReflectionType
{
	Unset = 0,
	Reflection = 1 << 0,
	Transmission = 1 << 1,
	All = Reflection | Transmission
};
inline BxDFReflectionType operator|(BxDFReflectionType a, BxDFReflectionType b)
{
	return static_cast<BxDFReflectionType>(static_cast<int>(a) | static_cast<int>(b));
}
inline BxDFReflectionType operator&(BxDFReflectionType a, BxDFReflectionType b)
{
	return static_cast<BxDFReflectionType>(static_cast<int>(a) & static_cast<int>(b));
}
inline bool operator!(BxDFReflectionType a)
{
	return static_cast<int>(a) == 0;
}
struct BSDFSample
{
	BSDFSample(Spectrum f, Vector3f wi, float pdf, BxDFType flags, float eta=1.0f, bool pdfIsProportional=false)
		:f(f), wi(wi), pdf(pdf), flags(flags), eta(eta), pdfIsProportional(pdfIsProportional) {
	}
	BSDFSample();
	Spectrum  f;
	Vector3f wi;
	float pdf = 0;
	BxDFType flags;
	float eta = 1;
    // Layered samples can carry a path-wise proportional f/pdf pair.
    // Keep that pair for throughput; query Pdf() separately for external MIS.
	bool pdfIsProportional = false;
};
class BxDF
{
public:
	

	BxDF(BxDFType type) : type(type) {}
    virtual bool NeedsComponentSample() const
    { return (type & BSDF_REFLECTION)!=0 && (type & BSDF_TRANSMISSION)!=0; }
	bool MatchesFlags(BxDFType t) const
	{
		return (type & t) == type;
	}
	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi, TransportMode mode) const = 0;
	virtual std::optional<BSDFSample> Sample_f(const Vector3f& wo,
		const Point2f& sample,
		TransportMode mode= TransportMode::Radiance,
		BxDFReflectionType sampledType = BxDFReflectionType::All) const=0;
    // PBRT-style independent event dimension. Single-lobe legacy BxDFs can
    // ignore uc; mixed continuous lobes must override this overload.
    virtual std::optional<BSDFSample> Sample_f(const Vector3f& wo, float uc,
        const Point2f& sample, TransportMode mode = TransportMode::Radiance,
        BxDFReflectionType sampledType = BxDFReflectionType::All) const
    { return Sample_f(wo, sample, mode, sampledType); }
	virtual Spectrum rho(const Vector3f& wo, int nSamples, const Point2f* samples) const=0;
	virtual float Pdf(const Vector3f& wo, const Vector3f& wi , 
		TransportMode mode,
		BxDFReflectionType sampleFlags = BxDFReflectionType::All) const=0;
public:
	const BxDFType type;

};


inline float FrComplex(float cosTheta_i, std::complex<float> eta)
{
	// Compatibility adapter: eta is already relative to the incident medium.
	// The complex Fresnel math lives only in FresnelUtils.h.
	return FrConductor(cosTheta_i, 1.0f, eta.real(), eta.imag());
}
inline Spectrum FrComplex(float cosTheta_i, const Spectrum& eta,
	const Spectrum& k)
{
	Spectrum result;
	for (size_t i = 0;i < result.c.size();i++)
	{
		result.c[i] = FrComplex(cosTheta_i,
			std::complex<float>(eta.c[i], k.c[i]));
	}
	return result;
}
inline Vector3f Reflect(const Vector3f& wo, const Vector3f& n)
{
	return -wo + 2.0f * wo.dot(n) * n;
}
namespace DielectricDetail {
// Temporary arithmetic: no change to Vector3f, ray storage or public APIs.
struct Direction { double x = 0, y = 0, z = 0; };
inline Direction Unit(Direction v)
{
    const double length = std::hypot(v.x, v.y, v.z);
    if (!(length > 0) || !std::isfinite(length)) return {};
    return {v.x / length, v.y / length, v.z / length};
}
inline Direction Promote(const Vector3f& v) { return Unit({double(v.x), double(v.y), double(v.z)}); }
inline Vector3f Store(Direction v) { return {float(v.x), float(v.y), float(v.z)}; }
inline double Dot(Direction a, Direction b) { return std::fma(a.x, b.x, std::fma(a.y, b.y, a.z * b.z)); }
inline Direction Combine(Direction a, double sa, Direction b, double sb)
{
    return {std::fma(sa, a.x, sb * b.x), std::fma(sa, a.y, sb * b.y), std::fma(sa, a.z, sb * b.z)};
}
inline bool RefractUnit(Direction wi, Direction n, const Interface& optics, Direction& wt)
{
    if (optics.tir) return false;
    if (optics.flipNormal) n = {-n.x, -n.y, -n.z};
    wt = Unit(Combine(wi, -1 / optics.eta, n, optics.cosI / optics.eta - optics.cosT));
    return Dot(wt, wt) > 0;
}
}
inline bool Refract(const Vector3f& wi, Vector3f n, float eta, float* etap, Vector3f* wt)
{
    if (!wt || !(eta > 0) || !std::isfinite(eta)) return false;
    // Promote BEFORE normalization/dot/reciprocal, not after float roundoff.
    const auto w = DielectricDetail::Promote(wi), normal = DielectricDetail::Promote(n);
    if (DielectricDetail::Dot(w, w) == 0 || DielectricDetail::Dot(normal, normal) == 0) return false;
    const auto optics = DielectricDetail::EvaluateInterface(DielectricDetail::Dot(w, normal), 1.0, double(eta));
    DielectricDetail::Direction transmitted;
    if (!DielectricDetail::RefractUnit(w, normal, optics, transmitted)) return false;
    *wt = DielectricDetail::Store(transmitted);
    if (etap) *etap = float(optics.eta); // Round only the public result.
    return true;
}
