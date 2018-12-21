#version 330 core

in vec3 v_normal;

out vec4 frag_color;

uniform vec4 u_color;
uniform int u_lighting;

void main() {
    float shade = 1.0;
    if (u_lighting == 1) {
        shade = dot(v_normal, normalize(vec3(1.0, 4.0, 2.0)));
        shade = shade * 0.5 + 0.5;
    }

    frag_color = vec4(u_color.rgb * shade * shade, u_color.a);
}