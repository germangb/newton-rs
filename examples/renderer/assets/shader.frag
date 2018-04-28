#version 130

in vec2 v_uv;
in vec3 v_normal;

out vec4 frag_color;

uniform vec3 u_color;
uniform vec3 u_light;

void main() {
    float light_cont = dot(normalize(u_light), -v_normal) * 0.5 + 0.5;

    frag_color = vec4(u_color * light_cont * light_cont, 1.0);
}
