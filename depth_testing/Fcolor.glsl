#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

float near = 0.1; 
float far  = 10.0; 

uniform sampler2D texture1;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near * far) / (far + near - z * (far - near));	
}

void main()
{    
	 FragColor = vec4(vec3(gl_FragCoord.z), 1.0);
}