#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexUV;
layout(location = 2) in vec3 vertexNormal;

// Output data ; will be interpolated for each fragment.
out vec2 UV;

// Values that stay constant for the whole mesh.
uniform mat4 M, V, P;

void main()
{
	gl_Position =  P * V * M * vec4(vertexPosition, 1);	
	
	// UV of the vertex. No special space for this one.
	UV = vertexUV;
}