#pragma once
#include "OceanWhitecapField.h"
#include "LambertianLidarScattering.h"
#include <memory>
#include <utility>

// 有效不透明泡沫面元：f=(1-W)*fWater+W*rhoFoam/pi。
// W 为局部未解析面积比例；没有额外 cosine、光程或接收器权重。
// 仅用于与该覆盖场坐标一致的海面；不是按物体自动分配的场景材质。
// 固定波长反射率基线，无泡沫体散射、寿命及输运。
class WhitecapLidarScattering final : public LidarScatteringModel
{
public:
    WhitecapLidarScattering(std::shared_ptr<const LidarScatteringModel> water,
        std::shared_ptr<const OceanWhitecapField> field, double foamReflectance)
        : water(std::move(water)), field(std::move(field)), foam(foamReflectance)
    {
        if (!this->water || !this->field)
            throw std::invalid_argument("Whitecap scattering requires water and field.");
    }
    double Evaluate(const HitRecord& hit, const Vector3f& wi,
        const Vector3f& wo, float wavelengthNm) const override
    {
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0)
            throw std::invalid_argument("Wavelength must be finite and positive.");
        auto normalized = [](const Vector3f& v) {
            const double length = std::hypot(double(v.x), double(v.y), double(v.z));
            if (!std::isfinite(length) || length == 0)
                throw std::invalid_argument("Whitecap directions and normal must be finite and nonzero.");
            return Vector3f(float(v.x/length), float(v.y/length), float(v.z/length));
        };
        const Vector3f incident = normalized(wi), outgoing = normalized(wo);
        const Vector3f ng = normalized(hit.geometricNormal);
        const double w = field->CoverageAt(hit.point.x, hit.point.z);
        if (ng.dot(incident) <= 0 || ng.dot(outgoing) <= 0) return 0;
        // 避免 W=1 时仍求水面模型（例如不适用的局部水面法线）。
        if (w == 0) return water->Evaluate(hit, incident, outgoing, wavelengthNm);
        const double foamValue = foam.Evaluate(hit, incident, outgoing, wavelengthNm);
        if (w == 1) return foamValue;
        return (1-w) * water->Evaluate(hit, incident, outgoing, wavelengthNm) + w * foamValue;
    }
private:
    std::shared_ptr<const LidarScatteringModel> water;
    std::shared_ptr<const OceanWhitecapField> field;
    LambertianLidarScattering foam;
};
