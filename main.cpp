#include "rasterizer.h"
#include "Matrix4f.h"
#include   "load_obj.h"
#include "camera.h"
#include "HitRocord.h"
#include "Hittable.h"
#include "Texture.h"
#include "ImageTexture.h"
#include "Lambertian.h"
#include "Sphere.h"
#include "Triangle.h"
#include "HittableList.h"
int width = 800;
int height = 600;
float c0 = 200.0f;
Vertex getVertex(Matrix4f MVP, Vector3f v0)
{
    Vector4f v0_clip = MVP * Vector4f(v0.x, v0.y, v0.z, 1.0f);
    float v0_real_w = v0_clip.w;
    // 4. 执行硬件透视除法，降维到 NDC 空间
    float v0_ndc_x = v0_clip.x / v0_clip.w;
    float v0_ndc_y = v0_clip.y / v0_clip.w;
    float v0_ndc_z = v0_clip.z / v0_clip.w; // 这就是用于 Z-Buffer 线性插值的 z_ndc

    Vertex a;
    a.x = (v0_ndc_x + 1.0f) * 0.5f * width;
    a.y = (1.0f - (v0_ndc_y + 1.0f) * 0.5f) * height; // 做了 Y 轴反转
    a.z_ndc = v0_ndc_z;// 传递标准 NDC 深度，用于深度测试
    a.inv_w = 1.0f / v0_real_w;
    a.color = c0;
    return a;
}
// 核心解调函数：追踪一条射线，返回其能量（颜色值）
float cast_ray(const Ray& r, const Hittable& world) {
    HitRecord rec;
    // 门禁：飞行距离限制在 [0.001f, 无穷大]，0.001f 是著名的“光追暗影痤疮避坑数值”
    if (world.hit(r, 0.001f, std::numeric_limits<float>::infinity(), rec)) {
        // 【物理主场预留】通过了测试！该射线撞到了表面！
        // 接下来我们要拿 rec.normal 去调用你的海面/物体 BRDF 公式来反演回波能量
        // 目前做基础验证，我们直接把法向量的某一个轴向映射成单色灰度（比如法线可视化）
        return std::max(0.0f, rec.normal.z) * 255.0f;
    }
    // 没撞到任何东西：返回黑夜背景色 0
    return 0.0f;
}
int main()
{
    
    Rasterizer r(width, height);
    //
    Matrix4f model = Matrix4f::Scale(1.0f, 1.0f, 1.0f); // 保持原样
    Matrix4f view = Matrix4f::LookAt(Vector3f(0.0f, 1.0f, 4.0f), Vector3f(0.0f, 1.0f, 0.0f), Vector3f(0.0f, 1.0f, 0.0f)); // 相机在原点看 -Z
    Matrix4f proj = Matrix4f::Perspective(45.0f, (float)width / height, 0.1f, 100.0f); // 45度透视
    Matrix4f MVP = proj * view * model; // 严格从右往左连乘

    load_obj bunny("./models/stanford-bunny.obj");
	for (const auto& face : bunny.temp_faces) {
		Vector3f v0 = bunny.temp_vertices[face.v0];
		Vector3f v1 = bunny.temp_vertices[face.v1];
		Vector3f v2 = bunny.temp_vertices[face.v2];
		Vertex a = getVertex(MVP, v0);
		Vertex b = getVertex(MVP, v1);
		Vertex c = getVertex(MVP, v2);
		r.DrawTriangle(a, b, c);
	}
    r.WriteToImage("bunny_render.ppm");
    



    return 0;
}