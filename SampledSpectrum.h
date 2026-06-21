#pragma once
#include "CoefficientSpectrum.h"
static const int nSpectralSamples = 60;
static const int sampledLambdaStart = 400;
static const int sampledLambdaEnd = 700;
inline float Interpolate(float x, float x0, float x1, float y0, float y1) {
    if (x <= x0) return y0;
    if (x >= x1) return y1;
    float t = (x - x0) / (x1 - x0);
    return (1.0f - t) * y0 + t * y1;
}

// 根据波长 lambda (单位 nm)，分析计算出对应的 CIE X, Y, Z 匹配值
void CIE_XYZ(float lambda, float& x, float& y, float& z) {
    // 拟合 X 曲线
    x = 1.056f * std::exp(-0.5f * std::pow((lambda - 599.8f) / 37.9f, 2))
        + 0.362f * std::exp(-0.5f * std::pow((lambda - 442.0f) / 16.0f, 2))
        - 0.065f * std::exp(-0.5f * std::pow((lambda - 501.1f) / 20.4f, 2));

    // 拟合 Y 曲线（这也是人眼亮度曲线）
    y = 0.821f * std::exp(-0.5f * std::pow((lambda - 568.8f) / 46.9f, 2))
        + 0.286f * std::exp(-0.5f * std::pow((lambda - 530.9f) / 16.3f, 2));

    // 拟合 Z 曲线
    z = 1.217f * std::exp(-0.5f * std::pow((lambda - 437.0f) / 11.8f, 2))
        + 0.681f * std::exp(-0.5f * std::pow((lambda - 459.0f) / 26.0f, 2));
}
class SampledSpectrum :public CoefficientSpectrum<nSpectralSamples>
{
public:
	SampledSpectrum(float v = 0.f) :CoefficientSpectrum<nSpectralSamples>(v) {}
    SampledSpectrum(const CoefficientSpectrum<nSpectralSamples>& v)
        : CoefficientSpectrum<nSpectralSamples>(v) {}
    float AverageSpectrumSamples(const float* lambda, const float* vals, int n,
        float lambdaStart ,float lambdaEnd) const {
		if (lambdaEnd <= lambda[0]) return vals[0];
		if (lambdaStart >= lambda[n - 1]) return vals[n - 1];
        float sum = 0.0f;
		if (lambdaStart < lambda[0]) {
			sum += vals[0] * (lambda[0] - lambdaStart);
		}
		if (lambdaEnd > lambda[n - 1]) {
			sum += vals[n - 1] * (lambdaEnd - lambda[n - 1]);
		}
        int i = 0;
		while (i < n && lambdaStart >= lambda[i+1]) i++;

        for (;i < n - 1;++i)
        {
            if (lambda[i] >= lambdaEnd)break;
			float segStart = std::max(lambdaStart, lambda[i]);
			float segEnd = std::min(lambdaEnd, lambda[i + 1]);
			if (segStart >= segEnd) continue;
			float vstart = Interpolate(segStart, lambda[i], lambda[i + 1], vals[i], vals[i + 1]);
            float vEnd = Interpolate(segEnd, lambda[i], lambda[i + 1], vals[i], vals[i + 1]);
            sum += 0.5f * (vstart + vEnd) * (segEnd - segStart);
              
        
        }
        return sum / (lambdaEnd - lambdaStart);
	}
  
   void TOXYZ(float xyz[3])const
   {
	   float xSum = 0.0f, ySum = 0.0f, zSum = 0.0f;
	   float yWeightSum = 0.0f;
       float delta = (float)(sampledLambdaEnd - sampledLambdaStart) / (nSpectralSamples - 1);
       for (int i = 0; i < nSpectralSamples; ++i) {
           float lambda = sampledLambdaStart + i * delta;
           float rx, ry, rz;
           CIE_XYZ(lambda, rx, ry, rz);// 获取人眼敏感度
           xSum += c[i] * rx;
           ySum += c[i] * ry;
           zSum += c[i] * rz;
           yWeightSum += ry;
       }
	   xyz[0] = xSum / yWeightSum;
	   xyz[1] = ySum / yWeightSum;
	   xyz[2] = zSum / yWeightSum;
   }

   void ToRGB(float rgb[3]) const {
       float xyz[3];
       TOXYZ(xyz);
       // XYZ to RGB conversion matrix
       rgb[0] = 3.2406f * xyz[0] - 1.5372f * xyz[1] - 0.4986f * xyz[2];
       rgb[1] = -0.9689f * xyz[0] + 1.8758f * xyz[1] + 0.0415f * xyz[2];
       rgb[2] = 0.0557f * xyz[0] - 0.2040f * xyz[1] + 1.0570f * xyz[2];

   }

   static SampledSpectrum FromSampled(const float* lambda, const float* vals, int n) {
	   SampledSpectrum s;
	   float delta = (float)(sampledLambdaEnd - sampledLambdaStart) / (nSpectralSamples - 1);
	   for (int i = 0; i < nSpectralSamples; ++i) {
		   float l0 = sampledLambdaStart + i * delta;
		   float l1 = sampledLambdaStart + (i + 1) * delta;
		   s.c[i] = s.AverageSpectrumSamples(lambda, vals, n, l0, l1);
	   }
	   return s;
   }
};

