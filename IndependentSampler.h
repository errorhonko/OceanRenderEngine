#pragma once
#include "Sampler.h"
#include <cstdint>
#include <random>
class IndependentSampler :public Sampler
{
public:
	explicit IndependentSampler(std::uint32_t seed = 0)
		:generator(seed), distribution{ 0.0f,1.0f }
	{
	}
	float Get1D() override
	{
		return distribution(generator);
	}
	Point2f Get2D() override
	{
		float u = Get1D();
		float v = Get1D();

		return Point2f(u, v);
	}



private:
	std::mt19937 generator;
	std::uniform_real_distribution<float> distribution;
};