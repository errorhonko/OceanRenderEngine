#pragma once

#include "BeckmannDistribution.h"
#include "BRDFUtils.h"
#include "OceanSlopeVariance.h"

// Maps horizontal world X/Z slope statistics to a +Z shading-local NDF.
// No BRDF or masking model is supplied here.
class OceanNormalDistribution
{
public:
    static OceanNormalDistribution FromWorldSlopeVarianceLinearized(
        const OceanSlopeVariance& slopes,
        const Vector3f& resolvedNormal)
    {
        // Validate before transforming: singular/zero covariance needs a delta model.
        const BeckmannDistribution validated(
            slopes.varianceX, slopes.varianceZ, slopes.covarianceXZ);
        const double length = std::hypot(double(resolvedNormal.x),
            double(resolvedNormal.y), double(resolvedNormal.z));
        if (!std::isfinite(length) || length == 0.0 || resolvedNormal.y <= 0.0f)
            throw std::invalid_argument("Ocean normal must be finite and point above the X/Z plane.");
        const Vector3f n(float(resolvedNormal.x / length),
            float(resolvedNormal.y / length), float(resolvedNormal.z / length));
        const Vector3f tangent = Vector3f(1.0f, 0.0f, 0.0f) - n * n.x;
        const Frame frame = Frame::FromXZ(tangent, n);

        // For N=(-p,1,-q), local slopes are -N_local.xy/N_local.z.
        // Linearize around resolved slopes (p0,q0): s_local ~= J (delta p,delta q).
        // J = n.y * [[t.x,t.z], [b.x,b.z]]. This is NOT an exact finite-slope rotation.
        const double a = double(n.y) * frame.x.x;
        const double b = double(n.y) * frame.x.z;
        const double c = double(n.y) * frame.y.x;
        const double d = double(n.y) * frame.y.z;
        const double vx = validated.VarianceX();
        const double vz = validated.VarianceY();
        const double cv = validated.CovarianceXY();
        const double localX = a * a * vx + 2.0 * a * b * cv + b * b * vz;
        const double localY = c * c * vx + 2.0 * c * d * cv + d * d * vz;
        const double localXY = a * c * vx + (a * d + b * c) * cv + b * d * vz;
        return OceanNormalDistribution(frame, BeckmannDistribution(localX, localY, localXY));
    }

    const Frame& SurfaceFrame() const { return frame; }
    const BeckmannDistribution& LocalDistribution() const { return distribution; }

    double DWorld(const Vector3f& microNormal) const
    {
        return distribution.D(frame.ToLocal(microNormal));
    }

private:
    OceanNormalDistribution(const Frame& frame, const BeckmannDistribution& distribution)
        : frame(frame), distribution(distribution) {}

    Frame frame;
    BeckmannDistribution distribution;
};
