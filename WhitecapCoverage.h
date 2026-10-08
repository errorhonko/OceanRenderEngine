#pragma once

#include <cmath>
#include <stdexcept>

// Monahan–O'Muircheartaigh (1980), robust biweight fit.
// 输入：10 m 高度处风速 U10，单位 m/s。
// 输出：平均白冠覆盖比例，范围 [0, 1]，不是百分数。
//
// 经验海况参数化，不预测局部白冠位置。
// 不应无条件外推到强风、极端海况或不同白冠定义。
inline double MonahanWhitecapCoverage(double windSpeed10m)
{
    if (!std::isfinite(windSpeed10m) ||
        windSpeed10m < 0.0)
    {
        throw std::invalid_argument(
            "Whitecap wind speed must be finite and non-negative.");
    }

    const double coverage =
        3.84e-6 * std::pow(windSpeed10m, 3.41);

    // 不静默截断到 1，避免隐藏经验公式的不合理外推。
    if (!std::isfinite(coverage) || coverage > 1.0)
    {
        throw std::domain_error(
            "Whitecap coverage is invalid; check model extrapolation.");
    }

    return coverage;
}