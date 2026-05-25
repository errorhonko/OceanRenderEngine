#include "rasterizer.h"
#include "Matrix4f.h"
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
    a.depth = v0_ndc_z;// 传递标准 NDC 深度，用于深度测试
    a.color = c0;
    return a;
}
int main()
{
    
    Rasterizer r(width, height);
   
    Vector3f v0(-1.0f, -1.0f, -5.0f);
    Vector3f v1(1.0f, -1.0f, -5.0f);
    Vector3f v2(0.0f, 1.0f, -5.0f);
    //
    Matrix4f model = Matrix4f::Scale(1.0f, 1.0f, 1.0f); // 保持原样
    Matrix4f view = Matrix4f::LookAt(Vector3f(0, 0, 0.1), Vector3f(0, 0, -0.8), Vector3f(0, 0.7, 0)); // 相机在原点看 -Z
    Matrix4f proj = Matrix4f::Perspective(60.0f, (float)width / height, 0.1f, 100.0f); // 45度透视
    Matrix4f MVP = proj * view * model; // 严格从右往左连乘

   
	Vertex a = getVertex(MVP, v0);
	Vertex b = getVertex(MVP, v1);
	Vertex c = getVertex(MVP, v2);
    r.clear();
    r.DrawTriangle(a, b, c); // 渲染进内存
    r.WriteToImage("output.ppm"); // 导出到本地
    r.clear();

    // =========================================================
    // 2. 铺设三角形 1：背景底板（暗灰色：100，平铺在深度 5.0f）
    // =========================================================
    Vertex t1_a{ 200.0f, 150.0f, 5.0f, 100 };
    Vertex t1_b{ 600.0f, 150.0f, 5.0f, 100 };
    Vertex t1_c{ 400.0f, 450.0f, 5.0f, 100 };

    r.DrawTriangle(t1_a, t1_b, t1_c);

    // =========================================================
    // 3. 铺设三角形 2：穿刺斜片（亮白色：255，从深度 2.0f 斜插到 8.0f）
    // =========================================================
    Vertex t2_a{ 400.0f, 100.0f, 2.0f, 255 }; // 👈 顶部很近 (2.0f)
    Vertex t2_b{ 450.0f, 500.0f, 8.0f, 255 }; // 👈 底部很远 (8.0f)
    Vertex t2_c{ 350.0f, 300.0f, 5.0f, 255 }; // 👈 侧边刚好处于中央交界

    r.DrawTriangle(t2_a, t2_b, t2_c);

    // =========================================================
    // 4. 快门闭环：导出显影图像
    // =========================================================
    std::string filename = "zbuffer_pierce_test.ppm";
    r.WriteToImage(filename);

    std::cout << "渲染大坝显影成功！请在工程目录下查看: " << filename << std::endl;


    return 0;
}