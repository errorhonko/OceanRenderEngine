#pragma once
#include "CoefficientSpectrum.h"
class RGBSpectrum :CoefficientSpectrum<3>
{
	RGBSpectrum(float v = 0.0f) : CoefficientSpectrum<3>(v) {}
	RGBSpectrum(const CoefficientSpectrum<3>& v) : CoefficientSpectrum<3>(v) {}
    RGBSpectrum(float r, float g, float b) {
        c[0] = r; // 红
        c[1] = g; // 绿
        c[2] = b; // 蓝
    }
    static RGBSpectrum FromRGB(const float rgb[3]) {
        return RGBSpectrum(rgb[0], rgb[1], rgb[2]);
    }
    void ToRGB(float rgb[3]) const {
        rgb[0] = c[0];
        rgb[1] = c[1];
        rgb[2] = c[2];
    }

    float y() const {
        return 0.212671f * c[0] + 0.715159f * c[1] + 0.072169f * c[2];
        
    }

    // D. RGB 转换为 CIE XYZ 颜色空间
    // 物理矩阵公式：
    // X = 0.412453 * R + 0.357580 * G + 0.180423 * B
    // Y = 0.212671 * R + 0.715159 * G + 0.072169 * B
    // Z = 0.019334 * R + 0.119193 * G + 0.950227 * B
    void ToXYZ(float xyz[3]) const {
        
		xyz[0] = 0.412453f * c[0] + 0.357580f * c[1] + 0.180423f * c[2]; // X
		xyz[1] = 0.212671f * c[0] + 0.715159f * c[1] + 0.072169f * c[2]; // Y
		xyz[2] = 0.019334f * c[0] + 0.119193f * c[1] + 0.950227f * c[2]; // Z
        
    }

    // E. CIE XYZ 转换为 RGB (静态工厂函数)
    // 物理矩阵公式（逆矩阵）：
    // R =  3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z
    // G = -0.9692660 * X + 1.8760108 * Y + 0.0415560 * Z
    // B =  0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z
    static RGBSpectrum FromXYZ(const float xyz[3]) {
		float r = 3.2404542f * xyz[0] - 1.5371385f * xyz[1] - 0.4985314f * xyz[2];
		float g = -0.9692660f * xyz[0] + 1.8760108f * xyz[1] + 0.0415560f * xyz[2];
		float b = 0.0556434f * xyz[0] - 0.2040259f * xyz[1] + 1.0572252f * xyz[2];
		RGBSpectrum result(r, g, b);

        return result;
    }

};

