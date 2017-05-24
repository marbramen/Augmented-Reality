#include "ModelData.h"
#include "Loader3D.h"

using namespace std;

ModelData::ModelData()
{
	m_vao = 0;
	m_vboVertex = 0;
	m_vboTexture = 0;
	m_vboNormal = 0;
	textureID = 0;
	numTriangles = 0;
}


ModelData::~ModelData()
{
}

void ModelData::loadData(std::string pathObj, std::string pathTexture)
{
	std::vector < glm::vec3 > out_vertices;
	std::vector < glm::vec2 > out_uvs;
	std::vector < glm::vec3 > out_normals;
	string tmpStr;

	bool res = loadObjFile(pathObj.c_str(), tmpStr, out_vertices, out_uvs, out_normals);
	if (res)
		printf("- Datos cargados: %d, %d, %d\n", out_vertices.size(), out_uvs.size(), out_normals.size());

	numTriangles = out_vertices.size();
	textureID = loadTexture(pathTexture.c_str());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	glGenBuffers(1, &m_vboVertex);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
	glBufferData(GL_ARRAY_BUFFER, out_vertices.size() * sizeof(vec3), &out_vertices.front(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glGenBuffers(1, &m_vboTexture);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboTexture);
	glBufferData(GL_ARRAY_BUFFER, out_uvs.size() * sizeof(vec2), &out_uvs.front(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, NULL);

	glGenBuffers(1, &m_vboNormal);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
	glBufferData(GL_ARRAY_BUFFER, out_normals.size() * sizeof(vec3), &out_normals.front(), GL_STATIC_DRAW);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	/*
	glVertexAttribPointer(
	0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
	3,                  // size
	GL_FLOAT,           // type
	GL_FALSE,           // normalized?
	0,                  // stride
	(void*)0            // array buffer offset
	);
	*/

	out_vertices.clear();
	out_uvs.clear();
	out_normals.clear();
}

void ModelData::removeData()
{
	if (m_vboVertex) {
		glDeleteBuffers(1, &m_vboVertex);
	}
	if (m_vboTexture) {
		glDeleteBuffers(1, &m_vboTexture);
	}
	if (m_vboNormal) {
		glDeleteBuffers(1, &m_vboNormal);
	}
	if (m_vao) {
		glDeleteVertexArrays(1, &m_vao);
	}
}