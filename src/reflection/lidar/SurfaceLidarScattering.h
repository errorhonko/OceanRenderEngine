#pragma once

#include "reflection/lidar/LidarScatteringModel.h"
#include <memory>
#include <stdexcept>
#include <utility>

// Dispatch only; does not define a new BRDF or change the return estimator.
// Bind concrete models to surfaces, not this dispatcher (avoids recursion).
class SurfaceLidarScattering final : public LidarScatteringModel
{
public:
    // No fallback = strict mode. Explicit fallback is only for missing bindings,
    // never for invalid results or out-of-band spectral queries.
    explicit SurfaceLidarScattering(
        std::shared_ptr<const LidarScatteringModel> fallback = nullptr)
        : fallback(std::move(fallback))
    {
        if (dynamic_cast<const SurfaceLidarScattering*>(this->fallback.get()))
            throw std::invalid_argument("Surface LiDAR fallback must not be a surface dispatcher.");
    }

    double Evaluate(const HitRecord& hit, const Vector3f& wi,
        const Vector3f& wo, float wavelengthNm) const override
    {
        const auto& model = hit.lidarScattering ? hit.lidarScattering : fallback;
        if (!model)
            throw std::logic_error("Hit surface has no LiDAR scattering model and no explicit fallback.");
        if (dynamic_cast<const SurfaceLidarScattering*>(model.get()))
            throw std::logic_error("Bind concrete LiDAR models to surfaces, not a surface dispatcher.");
        return model->Evaluate(hit, wi, wo, wavelengthNm);
    }

private:
    std::shared_ptr<const LidarScatteringModel> fallback;
};
