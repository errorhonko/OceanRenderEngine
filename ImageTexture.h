#pragma once
#include	"Texture.h"
#include "stb_image.h"
#include <iostream>
class ImageTexture : public Texture
{
	private:
		unsigned char* data;
		int width, height, bytes_per_scanline;
public :
	ImageTexture(const char* filename)
	{int components_per_pixel = 3;
		data = stbi_load(filename, &width, &height, &components_per_pixel, components_per_pixel);
		if (!data) {
			std::cerr << "Failed to load texture image: " << filename << std::endl;
			width = height = 0;
		}
		bytes_per_scanline = components_per_pixel * width;
	};
	~ImageTexture();
	Vector3f value(float u, float v) const override
	{
		int i = static_cast<int>(u * width);
		int j = static_cast<int>((1.0f - v) * height - 0.001f);
	
		if (i < 0) i = 0;if (i >= width)i = width - 1;	
		if (j < 0)j = 0;if (j >= height)j = height - 1;
		
		int pixel_index = j * width + i ;
		float r = data[pixel_index * 3 + 0] / 255.0f;
		float g = data[pixel_index * 3 + 1] / 255.0f;
		float b = data[pixel_index * 3 + 2] / 255.0f;
		return Vector3f(r, g, b);
	}
};

