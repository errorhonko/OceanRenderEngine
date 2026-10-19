#pragma once

#include "core/spectrum/SpectralOpticalConstants.h"
#include "core/spectrum/SpectralReflectanceCurve.h"
#include "reflection/lidar/HeliosEmpiricalLidarScattering.h"
#include <utility>
#include <variant>

// Loaded, owning data. File formats, percent/unit conversion and provenance
// remain in the existing readers. No file I/O occurs during ray evaluation.
struct ConstantLidarReflectance
{
    double value; // Dimensionless rho, NOT percent, RGB or BRDF.
};
using LidarReflectionData = std::variant<ConstantLidarReflectance,
    SpectralReflectanceCurve, SpectralOpticalConstants>;

struct LambertLidarResponse {};
struct GGXConductorLidarResponse
{
    double etaIncident;
    float alphaX, alphaY; // Distribution alpha, NOT artist roughness or RMS slope.
};
using LidarResponseModel = std::variant<LambertLidarResponse,
    HeliosEmpiricalAngularResponse, GGXConductorLidarResponse>;

// Configuration-stage input, not a per-hit object or serialized material library.
// Both choices are explicit. Data and response are separate but not arbitrary:
// rho supports Lambert/HELIOS; n/k supports the existing GGX conductor model.
struct LidarMaterialConfig
{
    LidarReflectionData reflectionData;
    LidarResponseModel responseModel;

    LidarMaterialConfig(LidarReflectionData data, LidarResponseModel response)
        : reflectionData(std::move(data)), responseModel(std::move(response)) {}
};
