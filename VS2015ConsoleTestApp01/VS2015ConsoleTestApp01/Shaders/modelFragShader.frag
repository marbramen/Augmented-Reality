#version 330 core

in vec2 UV;
in vec3 pos_eye;
in vec3 n_eye;

// Ouput data
out vec3 color;
//out vec4 color;

// Values that stay constant for the whole mesh.
uniform sampler2D myTexture;
uniform mat4 V;

// Phong Variables
const vec3 lightPos = vec3(100.0,100.0,-800);
const vec3 ambientColor = vec3(0.3, 0.0, 0.0);
const vec3 diffuseColor = vec3(0.5, 0.0, 0.0);
const vec3 specColor = vec3(1.0, 1.0, 1.0);	

void main(){
	vec3 incident_eye = normalize(pos_eye);
	vec3 normal = normalize(n_eye);
	vec3 reflected = reflect(incident_eye, normal);
	reflected = vec3(inverse(V) * vec4(reflected, 0.0));

	// Output color = color of the texture at the specified UV
	color = texture( myTexture, UV ).rgb;
	//color = texture( myTexture, reflected ).rgb;

	// Phong ilumination
	
	vec3 normalPhong = normalize(n_eye);
    vec3 lightDir = normalize(lightPos - pos_eye);
    vec3 reflectDir = reflect(-lightDir, normalPhong);
    vec3 viewDir = normalize(-pos_eye);
	
    float lambertian = max(dot(lightDir,normalPhong), 0.0);
    float specular = 0.0;
	
    if(lambertian > 0.0) {
       float specAngle = max(dot(reflectDir, viewDir), 0.0);
       specular = pow(specAngle, 4.0);
    }
	color = color + ambientColor + lambertian*diffuseColor + specular*specColor;
}