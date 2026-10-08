#pragma once

#include "OceanSlopeVariance.h"
#include "Vector3f.h"
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

// Linear Gaussian heightfield baseline: short world-X/Z slopes are independent
// of the resolved field, but may be mutually correlated. The slope-to-normal
// map is exact; this is NOT a complete microfacet BRDF or a visible-normal PDF.
class OceanSlopeNormalDistribution
{
public:
    OceanSlopeNormalDistribution(const OceanSlopeVariance& slopes,
        const Vector3f& resolvedNormal)
    {
        if (!std::isfinite(slopes.varianceX) || !std::isfinite(slopes.varianceZ) ||
            !std::isfinite(slopes.covarianceXZ) ||
            slopes.varianceX <= 0.0 || slopes.varianceZ <= 0.0)
            throw std::invalid_argument("Exact ocean normals require positive finite slope variances.");
        sigmaX = std::sqrt(slopes.varianceX);
        sigmaZ = std::sqrt(slopes.varianceZ);
        rho = slopes.covarianceXZ / sigmaX / sigmaZ;
        if (!std::isfinite(rho) || std::fabs(rho) >= 1.0)
            throw std::invalid_argument("Slope covariance must be positive definite; delta cases are separate.");
        residual = std::sqrt((1.0-rho)*(1.0+rho));
        logNormalization = -std::log(2.0*std::numbers::pi_v<double>) -
            std::log(sigmaX) - std::log(sigmaZ) - std::log(residual);

        const double length = DirectionLength(resolvedNormal);
        if (resolvedNormal.y <= 0.0f)
            throw std::invalid_argument("Resolved normal must point above the world X/Z plane.");
        p = -double(resolvedNormal.x)/resolvedNormal.y;
        q = -double(resolvedNormal.z)/resolvedNormal.y;
        macroY = double(resolvedNormal.y)/length;
    }

    // z1,z2 must be independent N(0,1) variates for the advertised PDF.
    // No hidden RNG/seed. Samples are weighted equally per horizontal area.
    Vector3f NormalFromStandardNormal(double z1, double z2) const
    {
        if (!std::isfinite(z1) || !std::isfinite(z2))
            throw std::invalid_argument("Standard normal variates must be finite.");
        const double dp = sigmaX*z1;
        const double dq = sigmaZ*(rho*z1+residual*z2);
        const double x = -p-dp, z = -q-dq;
        const double length = std::hypot(x,1.0,z);
        if (!std::isfinite(length))
            throw std::overflow_error("Sampled ocean slope overflow.");
        const Vector3f result(float(x/length),float(1.0/length),float(z/length));
        if (result.y <= 0.0f)
            throw std::overflow_error("Sampled normal cannot be represented above the horizon in Vector3f.");
        return result;
    }

    // u1,u2 in [0,1). Box-Muller uses 1-u1 so the allowed zero endpoint is safe.
    // This samples the full world upper hemisphere, not macro-visible normals.
    Vector3f SampleNormalWorld(double u1, double u2) const
    {
        if (!std::isfinite(u1) || !std::isfinite(u2) ||
            u1 < 0.0 || u1 >= 1.0 || u2 < 0.0 || u2 >= 1.0)
            throw std::invalid_argument("Normal sampling requires two uniforms in [0,1).");
        const double radius = std::sqrt(-2.0*std::log1p(-u1));
        const double phi = 2.0*std::numbers::pi_v<double>*u2;
        return NormalFromStandardNormal(radius*std::cos(phi),radius*std::sin(phi));
    }

    // Per steradian, normalized over m.y>0. Input direction is normalized internally.
    // p_omega(m) = P(-m.x/m.y-p, -m.z/m.y-q) / m.y^3.
    double PdfNormalWorld(const Vector3f& m) const { return Density(m,3,false); }

    // Differential micro-area per resolved macro-area per steradian:
    // D_A(m) = macroY * P(delta slopes) / m.y^4.
    // Retains normals with dot(m,n0)<=0! Integral D_A*m dOmega = n0,
    // hence the SIGNED projection integrates to 1. Not a drop-in local +Z NDF:
    // clipping the macro-lower hemisphere changes projected area and the model.
    // No G, Fresnel, shading-normal correction or reflection PDF is supplied.
    double AreaDensityWorld(const Vector3f& m) const { return Density(m,4,true); }

private:
    static double DirectionLength(const Vector3f& m)
    {
        const double length = std::hypot(double(m.x),double(m.y),double(m.z));
        if (!std::isfinite(length) || length == 0.0)
            throw std::invalid_argument("Normal direction must be finite and nonzero.");
        return length;
    }

    double LogSlopePdf(double dp, double dq) const
    {
        const double a = dp/sigmaX;
        if (!std::isfinite(a)) return -std::numeric_limits<double>::infinity();
        const double b = (dq/sigmaZ-rho*a)/residual;
        if (!std::isfinite(b)) return -std::numeric_limits<double>::infinity();
        return logNormalization-0.5*(a*a+b*b);
    }

    double Density(const Vector3f& m, int cosinePower, bool perMacroArea) const
    {
        const double length = DirectionLength(m);
        if (m.y <= 0.0f) return 0.0;
        const double dp = -double(m.x)/m.y-p;
        const double dq = -double(m.z)/m.y-q;
        // Evaluate in log space: avoid 0 * infinity in the Gaussian grazing tail.
        const double logValue = LogSlopePdf(dp,dq) -
            cosinePower*std::log(double(m.y)/length) +
            (perMacroArea ? std::log(macroY) : 0.0);
        const double value = std::exp(logValue);
        if (!std::isfinite(value))
            throw std::overflow_error("Ocean normal density overflow.");
        return value;
    }

    double p, q, macroY;
    double sigmaX, sigmaZ, rho, residual, logNormalization;
};
