#pragma once
#include <vector>
#include "Vector3f.h"
#include <fstream>
#include <sstream>
#include <string>
class load_obj
{	
	
	public:
	std::vector<Vector3f> temp_vertices;
	struct TriangleIndices {
		int v0, v1, v2;
	};
	std::vector<TriangleIndices> temp_faces;
	load_obj(const std::string& filepath)
	{	
		
		std::ifstream file(filepath);
		if (!file.is_open()) return;


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
			else if (type == "f") {
				int v0, v1, v2;
				char trash;
				std::string s0, s1, s2;
				ss >> s0 >> s1 >> s2;
				v0 = std::stoi(s0.substr(0, s0.find('/')));
				v1 = std::stoi(s1.substr(0, s1.find('/')));
				v2 = std::stoi(s2.substr(0, s2.find('/')));
				// OBJ 文件中的索引从 1 开始，转换为 0 基索引
				temp_faces.push_back({ v0 - 1, v1 - 1, v2 - 1 });
			}
		}

	}
};

