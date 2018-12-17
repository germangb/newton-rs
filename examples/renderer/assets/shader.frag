#version 130

in vec3 v_normal;
in vec3 v_normal_view;

out vec4 frag_color;

uniform vec3 u_color;
uniform vec3 u_light;

void main() {
    float light = dot(normalize(u_light), -v_normal) * 0.5 + 0.5;
          light = mix(0.5, 1.0, light * light);

    frag_color = vec4(1.0, 0.0, 1.0, 1.0);
}

