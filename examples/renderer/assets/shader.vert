#version 130

in vec3 a_position;
in vec3 a_normal;
in vec2 a_uv;

out vec2 v_uv;
out vec3 v_normal;
out vec3 v_normal_view;

uniform mat4 u_projection;
uniform mat4 u_view;
uniform mat4 u_world;

void main() {
    mat4 world_view = u_view * u_world;
    gl_Position = u_projection * floor(world_view * vec4(a_position * 0.5, 1.0) * 32) / 32;

    v_uv = a_uv;
    v_normal = normalize((u_world * vec4(a_normal, 0.0)).xyz);
    v_normal_view = normalize((world_view * vec4(a_normal, 0.0)).xyz);
}

