#version 330 core

// Interpolated values from the vertex shaders
in vec2 ex_Texture;

// Ouput data
out vec3 out_Color;

// Values that stay constant for the whole mesh.
uniform sampler2D myTexture;

void main() {
	out_Color = texture( myTexture, ex_Texture ).rgb;
}
