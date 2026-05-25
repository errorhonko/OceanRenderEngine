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
		int i=stas
	}
};

