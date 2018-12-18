#version 330 core

in vec3 v_normal;

out vec4 frag_color;

uniform vec4 u_color;

void main() {
    float shade = dot(v_normal, vec3(0.0, 0.0, 1.0));

    frag_color = vec4(u_color.rgb * shade, u_color.a);
}