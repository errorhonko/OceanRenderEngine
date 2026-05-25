#pragma once
#include "Vector4f.h"
#include "Vector3f.h"
#include <iostream>
#include <numbers>
const float pi = std::numbers::pi_v<float>;
class Matrix4f
{
public:
	float m[4][4];
	Matrix4f() {
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				m[i][j] = (i == j) ? 1.0f : 0.0f; // 初始化为单位矩阵
			}
		}
	}
    // 矩阵 ✖️ 向量 (用于对 3D 顶点进行坐标变换)
    Vector4f operator * (const Vector4f& v) const {
        Vector4f res;
		res.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w;
		res.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w;
		res.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w;
		res.w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w;
        return res;
    }

    // 矩阵 ✖️ 矩阵 (用于把多个变换组合成一个，比如 P * V * M)
    Matrix4f operator * (const Matrix4f& right) const {
        Matrix4f res;
        // res.m[i][j] = 行矩阵第 i 行与列矩阵第 j 列的点乘
        for (int i = 0;i < 4;i++)
        {
			for (int j = 0;j < 4;j++)
			{
				res.m[i][j] = m[i][0] * right.m[0][j] + m[i][1] * right.m[1][j] + m[i][2] * right.m[2][j] + m[i][3] * right.m[3][j];
			}
        }
            
        return res;
    }
    //创建缩放矩阵
    static  Matrix4f Scale(float sx, float sy, float sz)
    {
        Matrix4f ScaleMatrix;
        ScaleMatrix.m[0][0] = sx;
		ScaleMatrix.m[1][1] = sy;
		ScaleMatrix.m[2][2] = sz;
		return  ScaleMatrix;
           
     }
    //创建平移矩阵
    static Matrix4f Translate(float tx, float ty, float tz)
    {
        Matrix4f TranslateMatrix;  
        TranslateMatrix.m[0][3] = tx;
        TranslateMatrix.m[1][3] = ty;
        TranslateMatrix.m[2][3] = tz;
        return TranslateMatrix;
    }
  

    //创建旋转矩阵
    static Matrix4f RotateX(float angleDegrees)
    {
        float theta = angleDegrees * pi / 180.0f;
		float cosTheta = std::cos(theta);
		float sinTheta = std::sin(theta);
        Matrix4f RotateMatrix;
		RotateMatrix.m[1][1] = cosTheta;
        RotateMatrix.m[1][2] = -sinTheta;
		RotateMatrix.m[2][1] = sinTheta;
        RotateMatrix.m[2][2] = cosTheta;
        return RotateMatrix;
    }
    static Matrix4f RotateY(float angleDegrees)
    {
        float theta = angleDegrees * pi / 180.0f;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);
        Matrix4f RotateMatrix;
        RotateMatrix.m[0][0] = cosTheta;
        RotateMatrix.m[0][2] = sinTheta;
        RotateMatrix.m[2][0] = -sinTheta;
        RotateMatrix.m[2][2] = cosTheta;
        return RotateMatrix;
    }
    static Matrix4f RotateZ(float angleDegrees)
    {
        float theta = angleDegrees * pi / 180.0f;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);
        Matrix4f RotateMatrix;
        RotateMatrix.m[0][0] = cosTheta;
        RotateMatrix.m[0][1] = -sinTheta;
        RotateMatrix.m[1][0] = sinTheta;
        RotateMatrix.m[1][1] = cosTheta;
        return RotateMatrix;
    }
    static Matrix4f LookAt(Vector3f eye, Vector3f target, Vector3f up= Vector3f(0,1,0))
    {
		Vector3f w = (eye - target).normalize();   
		Vector3f u = up.cross(w).normalize();
		Vector3f v = w.cross(u);
        Matrix4f Tview = Matrix4f::Translate(-eye.x, -eye.y, -eye.z);;
		
        Matrix4f Rview;
        Rview.m[0][0] = u.x; Rview.m[0][1] = u.y; Rview.m[0][2] = u.z;
		Rview.m[1][0] = v.x; Rview.m[1][1] = v.y; Rview.m[1][2] = v.z;
		Rview.m[2][0] = w.x; Rview.m[2][1] = w.y; Rview.m[2][2] = w.z;
		return Rview * Tview; // 先平移再旋转

    }
    static Matrix4f  Perspective(float fovY, float aspect, float zNear, float zFar)
    {
        Matrix4f mat;
        //左 $l$、右 $r$、下 $b$、上 $t$、远 zFar、近 zNear 
        float t = std::tan(fovY * pi / 180.0f / 2.0f) * std::abs(-zNear);
        float b = -t;
		float r = t * aspect;
        float l = -r;
        mat.m[0][0] = zNear / r;
        mat.m[1][1] = zNear / t;
        mat.m[2][2] = -(zFar + zNear) / (zFar - zNear);
        mat.m[2][3] = -(2.0f * zFar * zNear) / (zFar - zNear);
        mat.m[3][2] = -1.0f; 
        mat.m[3][3] = 0.0f;
		return mat; // 先透视投影再正交归一化 


    }
	static Matrix4f Orthographic(float left, float right, float bottom, float top, float zNear, float zFar)
	{
		Matrix4f mat;
		mat.m[0][0] = 2.0f / (right - left);
		mat.m[1][1] = 2.0f / (top - bottom);
		mat.m[2][2] = -2.0f / (zFar - zNear);
		mat.m[0][3] = -(right + left) / (right - left);
		mat.m[1][3] = -(top + bottom) / (top - bottom);
		mat.m[2][3] = -(zFar + zNear) / (zFar - zNear);
		return mat; // 先正交归一化再平移到中心
	}
};
