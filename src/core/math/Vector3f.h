#pragma once
#include <cmath>
#include <random>

inline float random_float() {
	static std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
	static std::mt19937 generator;
	return distribution(generator);
}

inline float random_float(float min, float max) {
	return min + (max - min) * random_float();
}


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

	Vector3f operator / (float s) const {
		float	inv = 1.0f / s;
		return Vector3f(x * inv, y * inv, z * inv);
	}
    Vector3f operator-() const {
        return Vector3f(-x, -y, -z);
    }

    // Allow scalar multiplication on the left (float * Vector3f)
    friend Vector3f operator*(float s, const Vector3f& v) {
        return v * s;
    }  

    // 点乘 (Dot Product)：用于计算夹角余弦、投影、光照强度
    float dot(const Vector3f& v) const {
        return x*v.x+y*v.y+z*v.z;
    }

    // 叉乘 (Cross Product)：用于计算法向量、判断点是否在三角形内部
    Vector3f cross(const Vector3f& v) const {
        return Vector3f(y*v.z - v.y*z, z*v.x - x*v.z, x*v.y - y*v.x);
    }

   
    static Vector3f reflect(const Vector3f& v, const Vector3f& n) {
        return v - 2.0f * v.dot(n) * n;
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
	
	bool near_zero() const {
		const float s = 1e-8f;
		return (std::fabs(x) < s) && (std::fabs(y) < s) && (std::fabs(z) < s);
	}

	static Vector3f random() {
		return Vector3f(random_float(), random_float(), random_float());
	}

	static Vector3f random(float min, float max) {
		return Vector3f(random_float(min, max), random_float(min, max), random_float(min, max));
	}

	static Vector3f random_in_unit_sphere() {
		while (true) {
			Vector3f p = Vector3f::random(-1.0f, 1.0f);
			if (p.dot(p) >= 1.0f) continue;
			return p;
		}
	}

	static Vector3f random_unit_vector() {
		return random_in_unit_sphere().normalize();
	}

};

