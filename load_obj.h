#pragma once
#include <vector>
#include "Vector3f.h"
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

class load_obj
{	
	
	public:
	std::vector<Vector3f> temp_vertices;//顶点数组
	std::vector<Vector3f> temp_normals;//法线数组
	std::vector<Vector3f> temp_uvs;//纹理数组
	struct TriangleIndices {
		int v0, v1, v2;//顶点索引
		int vt0, vt1, vt2;//纹理坐标索引
		int vn0, vn1, vn2;//法线索引
	};
	std::vector<TriangleIndices> temp_faces;
private :
	// 辅助解析函数：解析形如 "v/vt/vn"、"v/vt"、"v//vn" 或 "v" 的字符串  
	void parseFaceToken(const std::string& token, int& vertexIndex, int& uvIndex, int& normalIndex) {
		vertexIndex = uvIndex = normalIndex = -1; // 初始化为 -1，表示未定义
		size_t firstSlash = token.find('/');
		size_t secondSlash = token.find('/', firstSlash + 1);
		if (firstSlash == std::string::npos) {
			// 只有顶点索引
			vertexIndex = std::stoi(token);
		}
		else if (secondSlash == std::string::npos) {
			// 顶点索引和纹理坐标索引
			vertexIndex = std::stoi(token.substr(0, firstSlash));
			uvIndex = std::stoi(token.substr(firstSlash + 1));
		}
		else {
			// 顶点索引、纹理坐标索引和法线索引
			vertexIndex = std::stoi(token.substr(0, firstSlash));
			if (secondSlash > firstSlash + 1) {
				uvIndex = std::stoi(token.substr(firstSlash + 1, secondSlash - firstSlash - 1));
			}
			if (secondSlash < token.size() - 1) {
				normalIndex = std::stoi(token.substr(secondSlash + 1));
			}
		}
	}
public:

	load_obj(const std::string& filepath)
	{	
		
		std::ifstream file(filepath);
		if (!file.is_open())
		{
			std::cerr << "Failed to open OBJ file: " << filepath << std::endl;
			return;
		}


		std::string line;
		while (std::getline(file, line)) 
		{
			if (line.empty()) continue;
			std::stringstream ss(line);
			std::string type;
			ss >> type;
			if (type == "v") {
				float x, y, z;
				ss >> x >> y >> z;
				temp_vertices.push_back(Vector3f(x, y, z));
			}
			else if (type == "vt") {
				float u, v;
				ss >> u >> v;
				temp_uvs.push_back(Vector3f(u, v, 0.0f));
			}
			else if (type == "vn") {
				float nx, ny, nz;
				ss >> nx >> ny >> nz;
				temp_normals.push_back(Vector3f(nx, ny, nz));
			}
			else if (type == "f") {
				
				std::string s0, s1, s2;
				ss >> s0 >> s1 >> s2;
				int v0, v1, v2;
				int vt0, vt1, vt2;
				int vn0, vn1, vn2;

				parseFaceToken(s0, v0, vt0, vn0);
				parseFaceToken(s1, v1, vt1, vn1);
				parseFaceToken(s2, v2, vt2, vn2);
				// OBJ 文件中的索引从 1 开始，转换为 0 基索引
				TriangleIndices face;
				face.v0 = v0 - 1;
				face.v1 = v1 - 1;
				face.v2 = v2 - 1;
				face.vt0 = (vt0 > 0) ? vt0 - 1 : -1; // 纹理坐标索引可能不存在
				face.vt1 = (vt1 > 0) ? vt1 - 1 : -1;
				face.vt2 = (vt2 > 0) ? vt2 - 1 : -1;
				face.vn0 = (vn0 > 0) ? vn0 - 1 : -1; // 法线索引可能不存在
				face.vn1 = (vn1 > 0) ? vn1 - 1 : -1;
				face.vn2 = (vn2 > 0) ? vn2 - 1 : -1;
				temp_faces.push_back(face);
				
			
			}
		}
		if(temp_normals.empty())
		{
			temp_normals.resize(temp_vertices.size(), Vector3f(0.0f, 0.0f, 0.0f));
			for (const auto& face : temp_faces)
			{
				Vector3f v0 = temp_vertices[face.v0];
				Vector3f v1 = temp_vertices[face.v1];
				Vector3f v2 = temp_vertices[face.v2];
				Vector3f face_normal = (v1 - v0).cross(v2 - v0).normalize();
				temp_normals[face.v0] = face_normal + temp_normals[face.v0];
				temp_normals[face.v1] = face_normal + temp_normals[face.v1];
				temp_normals[face.v2] = face_normal + temp_normals[face.v2];


			}
			for (auto& normal : temp_normals)
			{
				normal = normal.normalize();
			}
		}



	}
};

