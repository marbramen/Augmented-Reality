#pragma once
#include <string>
#include <GL\glew.h>

class ModelData
{
private:
	GLuint m_vboVertex;
	GLuint m_vboTexture;
	GLuint m_vboNormal;
public:
	GLuint m_vao;
	GLuint textureID;
	int numTriangles;

	ModelData();
	~ModelData();

	void loadData(std::string pathObj, std::string pathTexture);
	void removeData();
};
