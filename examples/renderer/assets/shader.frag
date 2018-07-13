#version 130

in vec2 v_uv;
in vec3 v_normal;
in vec3 v_normal_view;

out vec4 frag_color;

uniform vec3 u_color;
uniform vec3 u_light;
uniform sampler2D u_texture;

void main() {
    float light = dot(normalize(u_light), -v_normal) * 0.5 + 0.5;
          light = mix(0.5, 1.0, light * light);

    vec3 texture_color = texture2D(u_texture, v_uv).rgb;

    frag_color = vec4(u_color * texture_color * light, 1.0);
}

