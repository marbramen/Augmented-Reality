#version 400

// Interpolated values from the vertex shaders
in vec3 Color;
// Ouput data
out vec4 FragColor;

void main() {
	FragColor = vec4(Color, 1);
}