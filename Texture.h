#pragma once
#include	"Vector3f.h"
class Texture
{
public:
	virtual ~Texture() = default;
	virtual Vector3f value(float u, float v) const = 0;

};
class SolidColor :public Texture
{
private:
	Vector3f color_value;
public:
	SolidColor(Vector3f c) : color_value(c) {};
	Vector3f value(float u, float v) const override
	{
		return color_value;
	}

};
class FloatTexture
{
public:
	
	virtual	~FloatTexture() = default;
	virtual float value(float u, float v)const = 0;
private:

};

class ConstantFloatTexture : public FloatTexture
{
public:
	ConstantFloatTexture(float c) : value_(c) {};
	float value(float u, float v) const override
	{
		return value_;
	}
private:
	float value_;
};