#pragma once
#include <array>
#include <cmath>
using namespace std;
template <size_t N>
class CoefficientSpectrum
{
public:
	std::array<float, N> c;
	CoefficientSpectrum(float v=0.f) {
		c.fill(v);
	}

	CoefficientSpectrum<N> operator+(const CoefficientSpectrum<N>& s) const {
		CoefficientSpectrum<N> result;
		for (size_t i = 0; i < N; ++i) {
			result.c[i] = c[i] + s.c[i];
		}
		return result;
	}

	CoefficientSpectrum<N> operator-(const CoefficientSpectrum<N>& s) const {
		CoefficientSpectrum<N> result;
		for (size_t i = 0; i < N; ++i) {
			result.c[i] = c[i] - s.c[i];
		}
		return result;
	}

	CoefficientSpectrum<N> operator*(const CoefficientSpectrum<N>& s) const {
		CoefficientSpectrum<N> result;
		for (size_t i = 0; i < N; ++i) {
			result.c[i] = c[i] * s.c[i];
		}
		return result;
	}
	CoefficientSpectrum <N> operator*(float f) const {
		CoefficientSpectrum<N> result;
		for (size_t i = 0; i < N; ++i) {
			result.c[i] = c[i] * f;
		}
		return result;
	}

	bool IsBlack() const {
		for (size_t i = 0; i < N; ++i) {
			if (c[i] >1e-5f) {
				return false;
			}
		}
		return true;
	}

	friend CoefficientSpectrum <N> Sqrt(CoefficientSpectrum<N> &s) const {
		CoefficientSpectrum<N> result;
		for (size_t i = 0; i < N; ++i) {
			result.c[i] = std::sqrt(s.c[i]);
		}
		return result;
	}
	CoefficientSpectrum<N> Clamp(float low = 0.f, float high = INFINITY) const {
		CoefficientSpectrum<N> result;
		for (size_t i = 0; i < N; ++i) {
			result.c[i] = std::clamp(c[i], low, high);
		}
		return result;
	}

};
