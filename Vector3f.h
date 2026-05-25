#pragma once
#include <cmath>
class Vector3f
{
public:
	float x, y, z;
	Vector3f ():x(0.0f),y(0.0f), z(0.0f) {
	}
	Vector3f(float
		x, float y, float z) :x(x), y(y), z(z) {
	}
	Vector3f operator + (const Vector3f& v) const {
		return Vector3f(x + v.x, y + v.y, z + v.z);
	}
	Vector3f operator - (const Vector3f& v)const {
		return Vector3f(x - v.x, y - v.y, z - v.z);
	}
	Vector3f operator * (float s) const {
		return Vector3f(x * s, y * s, z * s);
	}
    

    // 点乘 (Dot Product)：用于计算夹角余弦、投影、光照强度
    float dot(const Vector3f& v) const {
        return x*v.x+y*v.y+z*v.z;
    }

    // 叉乘 (Cross Product)：用于计算法向量、判断点是否在三角形内部
    Vector3f cross(const Vector3f& v) const {
        return Vector3f(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x);
    }

    // 模长与归一化 (Normalize)：将向量长度化为 1（变成纯粹的方向向量）
    float norm() const { return std::sqrt(x * x + y * y + z * z); }

    Vector3f normalize() const {
        float n = norm();
        if (n > 0.0f) {
            float inv = 1.0f / n;
            return Vector3f(x * inv, y * inv, z * inv);
        }
        return *this;
    }



};

