#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

out vec3 v_normal;

uniform mat4 u_projection;
uniform mat4 u_view;
uniform mat4 u_world;

void main() {
    mat4 model_view = u_view * u_world;
    gl_Position = u_projection * model_view * vec4(a_position, 1.0);

    v_normal = normalize( (model_view * vec4(a_normal, 0.0)).xyz );
}
