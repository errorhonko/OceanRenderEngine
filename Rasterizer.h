#pragma once
#include <vector>
#include <fstream>
#include <string>
#include "Vector3f.h"
struct Vertex
{
	float x, y;
	float depth;
	float z_ndc;// 用于 Z-Buffer 深度测试的归一化标准深度 [-1, 1]
	float inv_w;// 用于颜色/属性透视矫正插值的 View 空间真实深度倒数 (1/W)
	unsigned char color;
};
class Rasterizer
{
private:
	int width, height;
	std::vector<unsigned char> framebuffer; // 存储像素数据的帧缓冲区
	std::vector<float> depthBuffer; // 深度缓冲区，用于深度测试
public:
	Rasterizer(int w, int h) : width(w), height(h) {
		framebuffer.resize(width * height * 3, 0); // 初始化为黑色
		depthBuffer.resize(width * height, std::numeric_limits<float>::infinity()); // 初始化为无穷大
	}
	void clear() {
		std::fill(framebuffer.begin(), framebuffer.end(), 0); // 清空帧缓冲区
		std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::infinity()); // 重置深度缓冲区
	}
	void DrawTriangle(Vertex a, Vertex b, Vertex c)
	{
		Vector3f pointa=Vector3f(a.x, a.y, a.depth);
		Vector3f pointb = Vector3f(b.x, b.y, b.depth);
		Vector3f pointc = Vector3f(c.x, c.y, c.depth);

		Vector3f edge1 = pointb - pointa;
		Vector3f edge2 = pointc - pointb;
		Vector3f edge3 = pointa - pointc;

		int minX = std::min({ a.x, b.x, c.x });
		int maxX = std::max({ a.x, b.x, c.x });
		int minY = std::min({ a.y, b.y, c.y });
		int maxY = std::max({ a.y, b.y, c.y });

		minX = std::max(0, minX);
		maxX = std::min((width - 1), maxX);
		minY = std::max(0, minY);
		maxY = std::min((height - 1), maxY);
		float  S_total = edge1.cross(Vector3f() - edge3).z;
		float inv_S_total = 1.0f / S_total;

		float inv_Za = 1 /a.depth;
		float inv_Zb = 1 / b.depth;
		float inv_Zc = 1 / c.depth;
		
		
		for (int i = minX;i <= maxX;i++)
		{
			for (int j = minY;j <= maxY;j++)
			{
				float centerX = i + 0.5f;
				float centerY = j + 0.5f;
				Vector3f line1 = Vector3f(centerX, centerY, 0) - pointa;
				Vector3f line2 = Vector3f(centerX, centerY, 0) - pointb;
				Vector3f line3 = Vector3f(centerX, centerY, 0) - pointc;

				float Sa = edge1.cross(line1).z;
				float Sb = edge2.cross(line2).z;
				float Sc = edge3.cross(line3).z;
				
				bool inside = (Sa >= 0 && Sb >= 0 && Sc >= 0) || (Sa <= 0 && Sb <= 0 && Sc <= 0);
				if (inside)
				{
					float alpha = Sb * inv_S_total; 
					float beta = Sc * inv_S_total;
					float gama = Sa * inv_S_total;
					float z_ndc_current = alpha * a.z_ndc + beta * b.z_ndc + gama * c.z_ndc;
					
					int idx = j * width + i;
					if (z_ndc_current < depthBuffer[idx])
					{
						depthBuffer[idx] = z_ndc_current;
						float interpolated_inv_w = alpha * a.inv_w + beta * b.inv_w + gama * c.inv_w;
						float w_current = 1.0f / interpolated_inv_w; // 还原出当前像素最纯正的 View Space Z

						float colorDivW = alpha * (a.color * a.inv_w) + beta * (b.color * b.inv_w) + gama * (c.color * c.inv_w);
						float color = colorDivW * w_current;
						
						framebuffer[idx * 3 + 0] = static_cast<unsigned char>(color); // R
						framebuffer[idx * 3 + 1] = static_cast<unsigned char>(color); // G
						framebuffer[idx * 3 + 2] = static_cast<unsigned char>(color); // B
					}
				}
				
			}
		}
	}
	void WriteToImage(const std::string& filename)
	{
		std::ofstream out(filename, std::ios::binary);
		// 1. 写入 PPM 纯文本文件头
		out << "P6\n" << width << " " << height << "\n255\n";

		// 2. 一铲子把一维帧缓冲里的 RGB 原始字节全部倒进去
		out.write(reinterpret_cast<char*>(framebuffer.data()), framebuffer.size());

		out.close();
	}

};

