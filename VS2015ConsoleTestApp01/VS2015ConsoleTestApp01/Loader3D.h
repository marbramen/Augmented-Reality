#pragma once
#ifndef LOADER3D_H_
#define LOADER3D_H_

#include <vector>
#include <stdio.h>
#include <string>

#include <glm/glm.hpp>
#include <GL\glew.h>
#include "FreeImage.h"

using namespace std;
using namespace glm;

inline bool loadObjFile(const char *path, string &mtlFilename, vector<vec3> &out_vertices, vector<vec2> &out_uvs, vector<vec3> &out_normals) {
	printf("Loading OBJ file: %s...\n", path);

	std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
	std::vector<glm::vec3> temp_vertices;
	std::vector<glm::vec2> temp_uvs;
	std::vector<glm::vec3> temp_normals;

	FILE * file = fopen(path, "r");
	if (file == NULL) {
		printf("Impossible to open the file !\n");
		getchar();
		return false;
	}

	while (1) {
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

				   // It is a vertex coordinates
		if (strcmp(lineHeader, "v") == 0) {
			glm::vec3 vertex;
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
			temp_vertices.push_back(vertex);
		}
		// It is a vertex texture coordinates
		else if (strcmp(lineHeader, "vt") == 0) {
			glm::vec2 uv;
			fscanf(file, "%f %f\n", &uv.x, &uv.y);
			temp_uvs.push_back(uv);
		}
		// Vertex normal
		else if (strcmp(lineHeader, "vn") == 0) {
			glm::vec3 normal;
			fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
			temp_normals.push_back(normal);
		}
		// Triangles info
		else if (strcmp(lineHeader, "f") == 0) {
			std::string vertex1, vertex2, vertex3;
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
			if (matches != 9) {
				printf("File can't be read by our simple parser :-( Try exporting with other options\n");
				return false;
			}
			vertexIndices.push_back(vertexIndex[0]);
			vertexIndices.push_back(vertexIndex[1]);
			vertexIndices.push_back(vertexIndex[2]);
			uvIndices.push_back(uvIndex[0]);
			uvIndices.push_back(uvIndex[1]);
			uvIndices.push_back(uvIndex[2]);
			normalIndices.push_back(normalIndex[0]);
			normalIndices.push_back(normalIndex[1]);
			normalIndices.push_back(normalIndex[2]);
		}
		else if (strcmp(lineHeader, "mtllib") == 0) {
			char res[200];
			fscanf(file, "%s", &res);
			mtlFilename = string(res);
		}
		else {
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}
	}

	// For each vertex of each triangle
	for (unsigned int i = 0; i<vertexIndices.size(); i++) {
		// Get the indices of its attributes
		unsigned int vertexIndex = vertexIndices[i];
		unsigned int uvIndex = uvIndices[i];
		unsigned int normalIndex = normalIndices[i];

		// Get the attributes thanks to the index
		glm::vec3 vertex = temp_vertices[vertexIndex - 1];
		glm::vec2 uv = temp_uvs[uvIndex - 1];
		glm::vec3 normal = temp_normals[normalIndex - 1];

		// Put the attributes in buffers
		out_vertices.push_back(vertex);
		out_uvs.push_back(uv);
		out_normals.push_back(normal);
	}
	return true;
}

inline bool loadObjMtlFile(const char *path, string &textureFilename) {
	printf("Loading OBJ MTL file: %s...\n", path);

	// We obtain the textureName
	FILE * file = fopen(path, "r");
	if (file == NULL) {
		printf("Impossible to open the file !\n");
		getchar();
		return false;
	}

	while (1) {
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		if (strcmp(lineHeader, "map_Kd") == 0) {
			char res[200];
			fscanf(file, "%s", &res);
			textureFilename = string(res);
		}
		else {
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}
	}
	return true;
}

inline GLuint loadTexture(const char *pathTexture) {
	printf("Loading texture: %s...\n", pathTexture);

	GLuint textureID;
	FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(pathTexture);
	FIBITMAP *bitmap = FreeImage_Load(fif, pathTexture, 0);
	printf("- Bitmap loaded, size: %d x %d\n", FreeImage_GetWidth(bitmap), FreeImage_GetWidth(bitmap));

	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, FreeImage_GetWidth(bitmap), FreeImage_GetHeight(bitmap), 0, GL_BGRA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(bitmap)));
	FreeImage_Unload(bitmap);

	return textureID;
}

inline bool loadTextureCube(const char* pathTexture, GLenum sideCube) {
	//glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(pathTexture);
	FIBITMAP *bitmap = FreeImage_Load(fif, pathTexture, 0);
	printf("- Bitmap loaded, size: %d x %d\n", FreeImage_GetWidth(bitmap), FreeImage_GetWidth(bitmap));

	// copy image data into 'target' side of cube map
	glTexImage2D(sideCube, 0, GL_RGBA, FreeImage_GetWidth(bitmap), FreeImage_GetHeight(bitmap), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(bitmap)));
	FreeImage_Unload(bitmap);

	return true;
}

inline FIBITMAP *loadImage(const char *pathTexture)
{
	FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(pathTexture);
	FIBITMAP *bitmap = FreeImage_Load(fif, pathTexture, 0);
	printf("- Bitmap loaded, size: %d x %d\n", FreeImage_GetWidth(bitmap), FreeImage_GetWidth(bitmap));
	return bitmap;
}

inline void setupTexture(GLuint& texture) {
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

inline void deleteTexture(GLuint& texture) {
	glDeleteTextures(1, &texture);
}

inline void setupCubeMap(GLuint& texture) {
	glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_CUBE_MAP);
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_CUBE_MAP, texture);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

inline void setupCubeMap(GLuint& texture, FIBITMAP *xpos, FIBITMAP *xneg, FIBITMAP *ypos, FIBITMAP *yneg, FIBITMAP *zpos, FIBITMAP *zneg)
{
	setupCubeMap(texture);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA, FreeImage_GetWidth(xpos), FreeImage_GetHeight(xpos), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(xpos)));
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA, FreeImage_GetWidth(xneg), FreeImage_GetHeight(xneg), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(xneg)));
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA, FreeImage_GetWidth(ypos), FreeImage_GetHeight(ypos), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(ypos)));
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA, FreeImage_GetWidth(yneg), FreeImage_GetHeight(yneg), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(yneg)));
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA, FreeImage_GetWidth(zpos), FreeImage_GetHeight(zpos), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(zpos)));
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA, FreeImage_GetWidth(zneg), FreeImage_GetHeight(zneg), 0, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(FreeImage_ConvertTo32Bits(zneg)));
}

inline void deleteCubeMap(GLuint& texture) {
	glDeleteTextures(1, &texture);
}

inline void deleteImage(FIBITMAP *bitmap)
{
	FreeImage_Unload(bitmap);
}

#endif	//LOADER3D_H_