#pragma once

class Vector4f {
public:
    float x, y, z, w;

    Vector4f() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {} // 默认 w=1
    Vector4f(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
};
