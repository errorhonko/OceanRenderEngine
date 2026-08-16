#pragma once
#include "Vector3f.h"
#include <memory>
#include <utility>
#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>
class Light;
struct  SampledLight
{
	std::shared_ptr<Light> light;
	float p = .0f;
};

struct  LightSampleContext
{
	Vector3f p;
	Vector3f ns;
};

class UniformLightSampler
{
public:
	UniformLightSampler() = default;
	explicit UniformLightSampler(
		std::vector<std::shared_ptr<Light>> lights)
			:lights(std::move(lights))
	{

	}

	std::optional <SampledLight> Sample(float u) const
	{
		if (lights.empty())
			return std::nullopt;
		std::size_t lightIndex = std::min<std::size_t>(
			static_cast<std::size_t>(
				u * lights.size()
				),
			lights.size() - 1
		);
		float p = 1.0f / static_cast<float> (lights.size());
		return SampledLight{
			lights[lightIndex],
			p
		};
	}
	

	float PMF(const std::shared_ptr<Light>& light)const
	{
		if (lights.empty())
			return .0f;
		return 1.0f / static_cast<float>(lights.size());

	}
	
	std::optional<SampledLight>
		Sample(const LightSampleContext& ctx,
			float u) const
	{
		return Sample(u);
	}

	float PMF(
		const LightSampleContext& ctx,
		const std::shared_ptr<Light>& light) const
	{
		return PMF(light);
	}

	std::string ToString() const
	{
		return "UniformLightSampler";
	}
private:
	std::vector<std::shared_ptr<Light>> lights;
};