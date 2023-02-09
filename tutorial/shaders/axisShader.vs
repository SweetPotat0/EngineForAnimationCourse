#version 330

in vec3 position;
in vec3 normal;
in vec4 Ka;
in vec4 Kd;
in vec4 Ks;
in vec2 texcoord;
in vec3 root;

out vec2 texCoord0;
out vec3 normal0;
out vec3 color0;
out vec3 position0;

uniform mat4 Proj;
uniform mat4 View;
uniform mat4 Model;

void main()
{
	texCoord0 = texcoord;
    if(position.x > root.x){
        color0 = vec3(1,0,0);
    } else if(position.y > root.y) {
        color0 = vec3(0,1,0);
    } else if(position.z > root.z) {
        color0 = vec3(0,0,1);
    } else {
        color0 = vec3(0,0,0);
    }
	normal0 = (Model  * vec4(normal, 0.0)).xyz;
	position0 = vec3(Proj * View * Model * vec4(position, 1.0));
	gl_Position = Proj * View * Model * vec4(position, 1.0); // you must have gl_Position
}
