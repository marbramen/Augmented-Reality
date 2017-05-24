#include "Character.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>
#include <iostream>

using namespace glm;

Character::Character()
{
	isInited = false;
	m_vao = 0;
	m_textureID = 0;
	gPosition = glm::vec3(0, 0, 0);
	gOrientation = glm::vec3(0, 0, 0);
	gScale = glm::vec3(1.0f, 1.0f, 1.0f);
}

void Character::init(ModelData *data)
{
	if (data == NULL) {
		return;
	}
	m_vao = data->m_vao;
	m_textureID = data->textureID;
	numTriangles = data->numTriangles;
	isInited = true;
}

void Character::draw(ShaderInfo shaderInfo)
{
	if (!isInited) {
		printf("Please load the resource of the character before draw()\n");
	}

	// Usar el shader del objeto
	glUseProgram(shaderInfo.shaderID);
	glUniform1i(shaderInfo.uniformInfo[3], 0);

	// Modifying matrices
	glm::mat4 RotationMatrix = eulerAngleYXZ(gOrientation.y, gOrientation.x, gOrientation.z);
	glm::mat4 TranslationMatrix = translate(mat4(), gPosition);
	glm::mat4 ScalingMatrix = scale(mat4(), gScale);
	glm::mat4 ModelMatrix = Model * TranslationMatrix * RotationMatrix * ScalingMatrix;

	//std::cout << "valor del RotationMatrix " << std::endl;
	//for (int i = 0; i < 4; i++) {
	//	std::cout << RotationMatrix[i].x << " " << RotationMatrix[i].y << " " << RotationMatrix[i].z << " " << RotationMatrix[i].w << std::endl;
	//}

		
	// Send our transformation to the currently bound shader, in the "MVP" uniform
	glUniformMatrix4fv(shaderInfo.uniformInfo[0], 1, GL_FALSE, &ModelMatrix[0][0]);	//M
	glUniformMatrix4fv(shaderInfo.uniformInfo[1], 1, GL_FALSE, &View[0][0]); //V
	glUniformMatrix4fv(shaderInfo.uniformInfo[2], 1, GL_FALSE, &Projection[0][0]); //P

																				   // Drawing the character, VAO stores VBOs, only need this value to draw the characters
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_textureID);

	glBindVertexArray(m_vao);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);

	glDrawArrays(GL_TRIANGLES, 0, numTriangles);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
}

void Character::cleanup()
{
	if (!isInited) {
		return;
	}

	isInited = false;
	m_vao = 0;
	numTriangles = 0;
}

void Character::move(glm::vec3 despl)
{
	gPosition += despl;
}

void Character::setPosition(glm::vec3 newPos)
{
	gPosition = newPos;
}

void Character::setOrientation(glm::vec3 newOr)
{
	gOrientation = newOr;
}

void Character::setScale(glm::vec3 newScale)
{
	gScale = newScale;
}

glm::vec3 Character::getOrientation() {
	return gOrientation;
}
