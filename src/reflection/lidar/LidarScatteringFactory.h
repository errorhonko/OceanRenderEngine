#pragma once

#include "reflection/lidar/LidarMaterialConfig.h"
#include "reflection/lidar/LambertianLidarScattering.h"
#include "reflection/lidar/SpectralLambertianLidarScattering.h"
#include "reflection/lidar/RoughConductorLidarScattering.h"
#include <memory>
#include <stdexcept>
#include <type_traits>

// Configuration-to-runtime bridge only. Reuses the existing model constructors,
// validation and physical kernels; never reinterprets n/k as reflectance.
class LidarScatteringFactory
{
public:
    static std::shared_ptr<const LidarScatteringModel> Create(const LidarMaterialConfig& config)
    {
        return std::visit([&](const auto& data) -> std::shared_ptr<const LidarScatteringModel>
        {
            using Data = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<Data, SpectralOpticalConstants>)
            {
                const auto* response = std::get_if<GGXConductorLidarResponse>(&config.responseModel);
                if (!response)
                    throw std::invalid_argument("Optical n/k data requires a GGX conductor response; it is not rho.");
                return std::make_shared<RoughConductorLidarScattering>(
                    response->etaIncident, data, response->alphaX, response->alphaY);
            }
            else
            {
                if (std::holds_alternative<LambertLidarResponse>(config.responseModel))
                {
                    if constexpr (std::is_same_v<Data, ConstantLidarReflectance>)
                        return std::make_shared<LambertianLidarScattering>(data.value);
                    else
                        return std::make_shared<SpectralLambertianLidarScattering>(data);
                }
                if (const auto* response = std::get_if<HeliosEmpiricalAngularResponse>(&config.responseModel))
                {
                    if constexpr (std::is_same_v<Data, ConstantLidarReflectance>)
                        return std::make_shared<HeliosEmpiricalLidarScattering>(data.value, *response);
                    else
                        return std::make_shared<HeliosEmpiricalLidarScattering>(data, *response);
                }
                throw std::invalid_argument("A GGX conductor response requires optical n/k, not reflectance data.");
            }
        }, config.reflectionData);
    }
};
