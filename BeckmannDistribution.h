#pragma once

#include "Vector3f.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

// Correlated zero-mean Gaussian slopes in a local frame with normal +Z.
// The covariance is per projected area. In principal axes alpha_i^2 = 2 variance_i.
// PBRT microfacet D convention: integral D(m) m.z dOmega = 1.
class BeckmannDistribution
{
public:
    BeckmannDistribution(double varianceX, double varianceY, double covarianceXY = 0.0)
        : varianceX(varianceX), varianceY(varianceY), covarianceXY(covarianceXY)
    {
        if (!std::isfinite(varianceX) || !std::isfinite(varianceY) ||
            !std::isfinite(covarianceXY) || varianceX <= 0.0 || varianceY <= 0.0)
            throw std::invalid_argument("Beckmann requires positive finite slope variances.");
        sigmaX = std::sqrt(varianceX);
        sigmaY = std::sqrt(varianceY);
        correlation = covarianceXY / sigmaX / sigmaY;
        if (!std::isfinite(correlation) || std::fabs(correlation) >= 1.0)
            throw std::invalid_argument(
                "Slope covariance must be positive definite; delta distributions need a separate path.");
        residualScale = std::sqrt((1.0 - correlation) * (1.0 + correlation));
        logNormalization = -std::log(2.0 * std::numbers::pi_v<double>) -
            std::log(sigmaX) - std::log(sigmaY) - std::log(residualScale);
    }

    double VarianceX() const { return varianceX; }
    double VarianceY() const { return varianceY; }
    double CovarianceXY() const { return covarianceXY; }

    double PrincipalAngle() const
    {
        // Angle of the larger-variance axis from local +X, modulo pi.
        const double scale = std::max(varianceX, varianceY);
        return 0.5 * std::atan2(2.0 * (covarianceXY / scale),
            varianceX / scale - varianceY / scale);
    }

    double AlphaMajor() const
    {
        const double scale = std::max(varianceX, varianceY);
        const double x = varianceX / scale, y = varianceY / scale;
        const double c = covarianceXY / scale;
        const double eigenvalue = 0.5 * (x + y) + std::hypot(0.5 * (x - y), c);
        return std::sqrt(scale) * std::sqrt(2.0 * eigenvalue);
    }

    double AlphaMinor() const
    {
        // Product of principal alphas is 2 sqrt(det(covariance)).
        return 2.0 * (sigmaX / AlphaMajor()) * sigmaY * residualScale;
    }

    double SlopePdf(double slopeX, double slopeY) const
    {
        if (!std::isfinite(slopeX) || !std::isfinite(slopeY))
            throw std::invalid_argument("Slopes must be finite.");
        return CheckedExp(LogSlopePdf(slopeX, slopeY));
    }

    // Input direction is normalized internally. This is NDF, not a solid-angle PDF.
    double D(const Vector3f& wm) const
    {
        const double length = std::hypot(double(wm.x), double(wm.y), double(wm.z));
        if (!std::isfinite(length) || length == 0.0)
            throw std::invalid_argument("Microfacet normal must be finite and nonzero.");
        if (wm.z <= 0.0f)
            return 0.0;
        const double cosine = wm.z / length;
        return CheckedExp(LogSlopePdf(-double(wm.x) / wm.z, -double(wm.y) / wm.z) -
            4.0 * std::log(cosine));
    }

    // One-sided Smith model in the local +Z frame. Positive scaling of w
    // does not change the result; finite, nonzero directions are required.
    // The horizon and lower hemisphere have Lambda = infinity, hence G1 = 0.
    double Lambda(const Vector3f& w) const
    {
        const double length = std::hypot(double(w.x), double(w.y), double(w.z));
        if (!std::isfinite(length) || length == 0.0)
            throw std::invalid_argument("Smith direction must be finite and nonzero.");

        if (w.z <= 0.0f)
            return std::numeric_limits<double>::infinity();

        const double x = double(w.x) / length;
        const double y = double(w.y) / length;
        const double z = double(w.z) / length;

        // sqrt(w_xy^T Sigma w_xy), evaluated through the covariance's
        // Cholesky factors to avoid cancellation for correlated slopes.
        const double projectedSigma = std::hypot(
            sigmaX * x + correlation * sigmaY * y,
            sigmaY * residualScale * y);

        if (projectedSigma == 0.0)
            return 0.0; // Exact normal incidence.

        const double a = z / projectedSigma;
        if (a >= 12.0)
            return 0.0; // Gaussian tail is negligible at double precision for G.

        // a uses standard deviation, not Beckmann alpha.
        // Lambda = phi(a)/a - Q(a), with phi/Q the standard Gaussian PDF/tail.
        const double phi = std::exp(-0.5 * a * a) /
            std::sqrt(2.0 * std::numbers::pi_v<double>);
        const double tail = 0.5 * std::erfc(a / std::sqrt(2.0));
        return std::max(0.0, phi / a - tail);
    }

    double G1(const Vector3f& w) const
    {
        return 1.0 / (1.0 + Lambda(w));
    }

    // Height-correlated Smith G2, not the separable product G1(wo) * G1(wi).
    double G(const Vector3f& wo, const Vector3f& wi) const
    {
        return 1.0 / (1.0 + Lambda(wo) + Lambda(wi));
    }

    // Distribution of normals obtained by sampling projected-area Gaussian slopes.
    // This is NOT a visible-normal (VNDF) PDF, which would depend on viewing direction.
    double PdfNormal(const Vector3f& wm) const
    {
        const double value = D(wm);
        if (value == 0.0)
            return 0.0;
        return value * wm.z / std::hypot(double(wm.x), double(wm.y), double(wm.z));
    }

private:
    double LogSlopePdf(double x, double y) const
    {
        const double a = x / sigmaX;
        const double b = (y / sigmaY - correlation * a) / residualScale;
        return logNormalization - 0.5 * (a * a + b * b);
    }

    static double CheckedExp(double value)
    {
        const double result = std::exp(value);
        if (!std::isfinite(result))
            throw std::overflow_error("Beckmann density overflow.");
        return result;
    }

    double varianceX, varianceY, covarianceXY;
    double sigmaX, sigmaY, correlation, residualScale, logNormalization;
};
