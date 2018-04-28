#version 130

in vec3 a_position;
in vec3 a_normal;
in vec2 a_uv;

out vec2 v_uv;
out vec3 v_normal;

uniform mat4 u_view_projection;
uniform mat4 u_world;

void main() {
    gl_Position = u_view_projection * u_world * vec4(a_position * 0.5, 1.0);
    //gl_Position = vec4(a_position, 1.0);

    v_uv = a_uv;
    v_normal = a_normal;
}
