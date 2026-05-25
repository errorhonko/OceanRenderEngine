#pragma once
#include "Ray.h"
class Camera
{
private:
	Vector3f eye;
	Vector3f lower_left;
	Vector3f horizontal;
	Vector3f vertical;
public:
	Camera(int width, int height)
	{
		eye = Vector3f(0.0f, 0.0f, 0.0f);
		lower_left = Vector3f(-2.0f, -1.5f, -1.0f);
		horizontal = Vector3f(4.0f, 0.0f, 0.0f);
		vertical = Vector3f(0.0f, 3.0f, 0.0f);
	}
	Ray get_ray(float i, float j, int width, int height) const
	{
		float u = i / static_cast<float>(width);
		float v = j / static_cast<float>(height);
		Vector3f target_pos = lower_left + horizontal * u + vertical * v;
		Vector3f ray_dir = target_pos - eye;
		return Ray(eye, ray_dir);

	}
};

