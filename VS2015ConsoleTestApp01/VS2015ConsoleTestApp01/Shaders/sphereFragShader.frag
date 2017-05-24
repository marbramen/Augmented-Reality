#version 330 core

in vec2 UV;
in vec3 v_refraction;
in vec3 v_reflection;
in float v_fresnel;

// Ouput data
out vec4 color;

// Values that stay constant for the whole mesh.
uniform sampler2D myTexture;

void main()
{
	color = vec4(texture( myTexture, UV ).rgb, 1);
}