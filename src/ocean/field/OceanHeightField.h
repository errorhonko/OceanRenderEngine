#pragma once
#include <complex>
#include <cstddef>
#include <vector>

class OceanFrequencyField;

class OceanHeightField
{
public:
	explicit OceanHeightField(
		const OceanFrequencyField& frequencyField,
		float cutoffFraction = 1.0f);

	float CutoffWaveNumber() const
	{
		return cutoffWaveNumber;
	}

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
	float SlopeX(int x, int z) const;
	float SlopeZ(int x, int z) const;

	
private:
	std::size_t Index(int x, int z)const;

	const OceanFrequencyField& frequencyField;

	int resolution = 0;
	float cutoffWaveNumber = 0.0f;

	std::vector<std::complex<float>>
		frequencyBuffer;

	std::vector<float> heights;
	std::vector<std::complex<float>> slopeXBuffer;
	std::vector<std::complex<float>> slopeZBuffer;

	std::vector<float> slopesX;
	std::vector<float> slopesZ;
	float maxImaginaryResidual = 0.0f;


};
