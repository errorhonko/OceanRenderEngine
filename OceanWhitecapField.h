#pragma once
#include "OceanHeightField.h"
#include "WhitecapCoverage.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

// 高度排序的白冠空间代理，不是物理破碎判据。
// N*N 等水平面积单元；坐标与 OceanSurfaceMesh 一致：[-L/2,L/2)。
// 单元内常量、周期延拓，不插值掩码，以保持水平面积平均覆盖率。
class OceanWhitecapField
{
public:
    OceanWhitecapField(int resolution, double patchLength)
        : resolution(resolution), patchLength(patchLength)
    {
        if (resolution < 2 || !std::isfinite(patchLength) || patchLength <= 0)
            throw std::invalid_argument("Invalid whitecap grid configuration.");
        const auto n = static_cast<std::size_t>(resolution);
        if (n > std::numeric_limits<std::size_t>::max() / n)
            throw std::invalid_argument("Whitecap grid is too large.");
    }

    // 在 heightField.Update(time) 之后调用；内部保存快照，不引用高度数组。
    void Update(const OceanHeightField& heightField, double windSpeed10m)
    {
        if (heightField.Resolution() != resolution)
            throw std::invalid_argument("Whitecap and height grid resolutions differ.");
        const double coverage = MonahanWhitecapCoverage(windSpeed10m);
        std::vector<double> cellHeights;
        cellHeights.reserve(static_cast<std::size_t>(resolution) * resolution);
        for (int z = 0; z < resolution; ++z)
            for (int x = 0; x < resolution; ++x)
                cellHeights.push_back(0.25 * (double(heightField.Height(x,z)) +
                    heightField.Height((x+1)%resolution,z) +
                    heightField.Height(x,(z+1)%resolution) +
                    heightField.Height((x+1)%resolution,(z+1)%resolution)));
        UpdateCellHeights(cellHeights, coverage);
    }

    // 行优先 [z*N+x]。输入为单元代表高度；便于独立测试及其他高度场使用。
    // 四角平均只是排名代理，不是三角网格的精确面积平均高度。
    void UpdateCellHeights(const std::vector<double>& cellHeights, double coverage)
    {
        const auto count = static_cast<std::size_t>(resolution) * resolution;
        if (cellHeights.size() != count || !std::isfinite(coverage) || coverage < 0 || coverage > 1)
            throw std::invalid_argument("Invalid whitecap cell heights or coverage.");
        for (double h : cellHeights)
            if (!std::isfinite(h)) throw std::invalid_argument("Whitecap heights must be finite.");
        const auto bounds = std::minmax_element(cellHeights.begin(), cellHeights.end());
        const bool contrast = *bounds.first != *bounds.second;
        std::vector<double> next(count, 0.0);
        if (contrast)
        {
            std::vector<std::size_t> order(count);
            std::iota(order.begin(), order.end(), 0);
            // 同高时稳定按索引排序；不声称此任意选择具有物理意义。
            std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
                return cellHeights[a] > cellHeights[b];
            });
            double remaining = coverage * count;
            for (auto i : order)
            {
                next[i] = std::clamp(remaining, 0.0, 1.0);
                remaining -= next[i];
            }
        }
        // 平面没有波峰：实际覆盖率为零，不能冒充达到目标覆盖率。
        mask.swap(next);
        targetCoverage = coverage;
        hasHeightContrast = contrast;
        initialized = true;
    }

    double CoverageAt(double worldX, double worldZ) const
    {
        CheckInitialized();
        if (!std::isfinite(worldX) || !std::isfinite(worldZ))
            throw std::invalid_argument("Whitecap query coordinates must be finite.");
        return mask[static_cast<std::size_t>(Cell(worldZ)) * resolution + Cell(worldX)];
    }
    double MeanCoverage() const
    {
        CheckInitialized();
        return std::accumulate(mask.begin(), mask.end(), 0.0) / mask.size();
    }
    double TargetCoverage() const { CheckInitialized(); return targetCoverage; }
    bool HasHeightContrast() const { CheckInitialized(); return hasHeightContrast; }
    const std::vector<double>& Coverages() const { CheckInitialized(); return mask; }

private:
    void CheckInitialized() const
    {
        if (!initialized) throw std::logic_error("Whitecap field must be updated before use.");
    }
    int Cell(double coordinate) const
    {
        double wrapped = std::fmod(coordinate, patchLength) / patchLength + 0.5;
        wrapped -= std::floor(wrapped);
        return std::min(resolution - 1, static_cast<int>(wrapped * resolution));
    }
    int resolution;
    double patchLength;
    std::vector<double> mask;
    double targetCoverage = 0;
    bool initialized = false;
    bool hasHeightContrast = false;
};
