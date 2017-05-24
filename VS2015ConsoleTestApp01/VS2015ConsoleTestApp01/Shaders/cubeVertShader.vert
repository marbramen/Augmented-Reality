#version 400

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexColor;

// Output data ; will be interpolated for each fragment.
out vec3 Color;
// Values that stay constant for the whole mesh.
uniform mat4 M, V, P;

void main(){

	// Output position of the vertex, in clip space : MVP * position
	gl_Position = P * V * M * vec4(vertexPosition,1);

	// The color of each vertex will be interpolated
	// to produce the color of each fragment
	Color = vertexColor;
}