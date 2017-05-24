#pragma once
#ifndef CHARACTER_H
#define CHARACTER_H

#include <glm\glm.hpp>
#include "globalConstants.h"
#include "ModelData.h"

class Character {
private:
	bool isInited;
	GLuint m_vao,
		m_textureID;
	int numTriangles;
	glm::vec3 gPosition,
		gOrientation,
		gScale;
public:
	Character();
	void init(ModelData *data);
	void draw(ShaderInfo shaderInfo);
	void cleanup();

	void move(glm::vec3 despl);
	void setPosition(glm::vec3 newPos);	
	void setOrientation(glm::vec3 newOr);
	glm::vec3  getOrientation();
	void setScale(glm::vec3 newScale);
};

#endif