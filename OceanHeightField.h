#pragma once
#include <complex>
#include <cstddef>
#include <vector>

class OceanFrequencyField;

class OceanHeightField
{
public:
	explicit OceanHeightField(
		const OceanFrequencyField& frequencyField);

	void Update(float time);

	float Height(int x, int z) const;

	const std::vector<float>& Heights() const
	{
		return heights;
	}

	int Resolution() const
	{
		return resolution;
	}

	float MaxImaginaryResidual() const
	{

		return maxImaginaryResidual;
	}

private:
	std::size_t Index(int x, int z)const;

	const OceanFrequencyField& frequencyField;

	int resolution = 0;

	std::vector<std::complex<float>>
		frequencyBuffer;

	std::vector<float> heights;

	float maxImaginaryResidual = 0.0f;
};
