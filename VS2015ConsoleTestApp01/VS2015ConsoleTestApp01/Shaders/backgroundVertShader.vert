#version 400

// Input vertex data, different for all executions of this shader.
layout(location=0) in vec3 in_Position;
layout(location=1) in vec2 in_Texture;

// Output data ; will be interpolated for each fragment.
out vec2 ex_Texture;

void main(){	
	// Output position of the vertex, in clip space : MVP * position
	gl_Position =  vec4(in_Position, 1);

	// The color of each vertex will be interpolated
	// to produce the color of each fragment
	ex_Texture = in_Texture;
}
