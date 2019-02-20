use std::{mem, str};
use std::error::Error;
use std::fs::File;

use cgmath::{Deg, Matrix3, Matrix4, Point3, Vector3, Vector4};
use cgmath::prelude::*;
use gl::types::*;
use imgui_ext::ImGuiExt;
use serde::{Deserialize, Serialize};

use crate::math::Vector;

#[derive(ImGuiExt, Serialize, Deserialize, Clone)]
pub struct Camera {
    #[imgui(checkbox(label = "Mouse control"))]
    pub controller: bool,
    #[imgui(drag(speed = 0.1))]
    pub eye: [f32; 3],
    #[imgui(drag(speed = 0.1))]
    pub center: [f32; 3],
    #[imgui(slider(min = 1.0, max = 179.0))]
    pub fov: f32,
    #[imgui(slider(min = 0.01, max = 2000.0))]
    pub near: f32,
    #[imgui(
        slider(min = 0.01, max = 2000.0),
        button(label = "Reset camera", catch = "reset")
    )]
    pub far: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            eye: [6.0, 8.0, 16.0],
            center: [0.0, 0.0, 0.0],
            controller: true,
            fov: 55.0,
            near: 0.01,
            far: 1000.0,
        }
    }
}

#[rustfmt::skip]
pub fn compute_view_proj(camera: &Camera, viewport: [u32; 4]) -> Matrix4<f32> {
    let aspect = viewport[2] as f32 / viewport[3] as f32;
    let proj = cgmath::perspective(Deg(camera.fov), aspect, camera.near, camera.far);
    let view = Matrix4::look_at(Point3::from(camera.eye),
                                Point3::from(camera.center),
                                Vector3::new(0.0, 1.0, 0.0));
    proj * view
}

#[rustfmt::skip]
pub fn compute_ray(camera: &Camera, mut mx: i32, mut my: i32, [x, y, w, h]: [u32; 4]) -> (Vector, Vector) {
    mx -= x as i32;
    my -= y as i32;
    let view_proj = compute_view_proj(camera, [x, y, w, h]);
    let view_proj_inv = view_proj.invert().unwrap();
    let mut start = Vector4::new(mx as f32 / w as f32 * 2.0 - 1.0, my as f32 / h as f32 * 2.0 - 1.0, 0.0, 1.0);
    let mut end = start;
    end.z = 1.0;

    //println!("{:?}", start);
    let mut s = view_proj_inv * start;
    let mut e = view_proj_inv * end;
    s /= s.w;
    e /= e.w;

    (Vector::new3(s.x, s.y, s.z), Vector::new3(e.x, e.y, e.z))
}

impl Camera {
    pub fn r(&self) -> [f32; 3] {
        let [cx, cy, cz] = self.center;
        let [ex, ey, ez] = self.eye;
        [ex - cx, ey - cy, ez - cz]
    }

    pub fn dist(&self) -> f32 {
        let [x, y, z] = self.r();
        (x * x + y * y + z * z).sqrt()
    }

    fn basis(&self) -> Matrix3<f32> {
        let mut radius = -Vector3::from(self.r()).normalize();
        let up = Vector3::new(0.0, 1.0, 0.0);
        let up = up - radius * cgmath::dot(up, radius);
        let right = up.cross(radius);
        // TODO redundant normalization
        Matrix3::from_cols(right.normalize(), up.normalize(), radius.normalize())
    }

    pub fn orbit(&mut self, xrel: f32, yrel: f32) {
        let Matrix3 {
            x: right,
            y: up,
            z: look,
        } = self.basis();

        let k = 0.005;
        let tan = (up * yrel * k + right * xrel * k - look).normalize_to(self.dist());

        self.eye = self.center;
        self.eye[0] += tan.x;
        self.eye[1] += tan.y;
        self.eye[2] += tan.z;
    }

    pub fn pan(&mut self, xrel: f32, yrel: f32) {
        let Matrix3 {
            x: right,
            y: up,
            z: look,
        } = self.basis();

        let k = 0.001 * self.dist();
        self.center[0] += right.x * xrel * k;
        self.center[1] += right.y * xrel * k;
        self.center[2] += right.z * xrel * k;

        self.center[0] += up.x * yrel * k;
        self.center[1] += up.y * yrel * k;
        self.center[2] += up.z * yrel * k;

        self.eye[0] += right.x * xrel * k;
        self.eye[1] += right.y * xrel * k;
        self.eye[2] += right.z * xrel * k;

        self.eye[0] += up.x * yrel * k;
        self.eye[1] += up.y * yrel * k;
        self.eye[2] += up.z * yrel * k;
    }

    pub fn scroll(&mut self, n: f32) {
        let [x, y, z] = self.r();
        let dist = (x * x + y * y + z * z).sqrt();
        self.eye[0] -= n * x / dist;
        self.eye[1] -= n * y / dist;
        self.eye[2] -= n * z / dist;
    }
}

#[derive(ImGuiExt, Serialize, Deserialize, Clone)]
pub struct RenderParams {
    #[imgui(new_line, checkbox(label = "Solid"))]
    pub solid: bool,
    #[imgui(checkbox(label = "Wireframe"))]
    pub wire: bool,
    #[imgui(slider(min = 1.0, max = 4.0))]
    pub wire_size: f32,
    #[imgui(checkbox)]
    pub axis: bool,
    #[imgui(checkbox)]
    pub individual_axis: bool,

    #[imgui(checkbox(label = "AABB"))]
    pub aabb: bool,
    #[imgui(checkbox(label = "Names"))]
    pub names: bool,
    #[imgui(checkbox(label = "Origins"))]
    pub origins: bool,
    #[imgui(checkbox(label = "Lighting"))]
    pub lighting: bool,
    #[imgui(checkbox(label = "Shadows"))]
    pub shadows: bool,

    #[imgui(
        button(label = "Reset##RenderParams", catch = "reset"),
        new_line,
        nested
    )]
    pub camera: Camera,
    // colors
    #[imgui(new_line, color(edit))]
    pub background: [f32; 4],
    #[imgui(color(edit))]
    pub wireframe: [f32; 4],
    #[imgui(color(edit))]
    pub active: [f32; 4],
    #[imgui(color(edit))]
    pub sleeping: [f32; 4],
    #[imgui(color(edit))]
    pub selected: [f32; 4],
    #[imgui(color(edit))]
    pub aabb_color: [f32; 4],
}

impl RenderParams {
    fn load() -> Result<Self, Box<dyn Error>> {
        let mut file = File::open("testbed.json")?;
        Ok(serde_json::from_reader(file)?)
    }

    fn save(&self) -> Result<(), Box<dyn Error>> {
        let mut file = File::create("testbed.json")?;
        Ok(serde_json::to_writer(file, self)?)
    }
}

impl Default for RenderParams {
    fn default() -> Self {
        Self {
            camera: Default::default(),
            background: [0.24, 0.24, 0.3, 1.0],
            wireframe: [0.0, 0.0, 0.0, 1.0],
            active: [1.0, 0.5, 1.0, 1.0],
            sleeping: [0.7, 0.7, 0.7, 1.0],
            selected: [1.0, 1.0, 0.5, 1.0],
            aabb_color: [1.0, 1.0, 1.0, 1.0],

            solid: true,
            wire: true,
            wire_size: 2.0,
            axis: true,
            individual_axis: false,

            aabb: true,
            names: true,
            origins: true,

            lighting: false,
            shadows: false,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Vert {
    pub pos: [f32; 3],
    pub color: [u8; 3],
}

pub const fn vert(pos: [f32; 3], color: [u8; 3]) -> Vert {
    Vert { pos, color }
}

/// Immediate renderer. Its main functions are:
///
/// - Collect raw geometry (vertex data & index data)
/// - Set a view & projection transformation
#[derive(ImGuiExt)]
pub struct TestbedRenderer {
    vbo: GLuint,
    ebo: GLuint,
    vao: GLuint,

    vbo_data: Vec<Vert>,
    ebo_data: Vec<u16>,

    program: GLuint,
    // uniform
    u_view_proj: GLint,
    u_tint: GLint,
    u_offset: GLint,

    #[imgui(nested)]
    params: RenderParams,
    #[imgui(checkbox(label = "Persistent"))]
    persist: bool,

    // last frame stats
    // TODO implement ImGuiExt on optionals
    #[imgui(separator, nested)]
    stats: FrameStats,
    //stats: Option<FrameStats>,
}

macro_rules! gl {
    ($fun:ident( $( $arg:expr ),* )) => {{
        gl::GetError();
        let res = gl::$fun ( $($arg),* );
        assert_eq!(gl::NO_ERROR, gl::GetError());
        res
    }}
}

impl Drop for TestbedRenderer {
    fn drop(&mut self) {
        if self.persist {
            match self.params.save() {
                Err(e) => eprintln!("Error saving testbed.json: {}", e),
                Ok(_) => {}
            }
        }
        unsafe {
            gl!(DeleteBuffers(1, &self.vbo));
            gl!(DeleteBuffers(1, &self.ebo));
            gl!(DeleteVertexArrays(1, &self.vao));
            gl!(DeleteProgram(self.program));
        }
    }
}

impl TestbedRenderer {
    pub fn params(&self) -> &RenderParams {
        &self.params
    }

    pub fn params_mut(&mut self) -> &mut RenderParams {
        &mut self.params
    }

    pub fn new<F: FnMut(&str) -> *const ()>(mut loader: F) -> Self {
        gl::load_with(|s| loader(s) as _);

        let (mut ebo, mut vbo, mut vao) = (0, 0, 0);
        let mut vbo_data = Vec::with_capacity(0xFFFF);
        let mut ebo_data = Vec::with_capacity(0xFFFF);
        #[rustfmt::skip]
            let _ = unsafe {
            gl!(GenBuffers(1, &mut vbo));
            gl!(GenBuffers(1, &mut ebo));
            gl!(GenVertexArrays(1, &mut vao));

            gl!(BindVertexArray(vao));
            gl!(BindBuffer(gl::ARRAY_BUFFER, vbo));
            gl!(BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo));

            let vbo_size = 0xFFFF * mem::size_of::<Vert>();
            let ebo_size = 0xFFFF * mem::size_of::<u16>();
            gl!(BufferData(gl::ARRAY_BUFFER, vbo_size as _, vbo_data.as_ptr() as _, gl::STREAM_DRAW));
            gl!(BufferData(gl::ELEMENT_ARRAY_BUFFER, ebo_size as _, ebo_data.as_ptr() as _, gl::STREAM_DRAW));

            gl!(EnableVertexAttribArray(0));
            gl!(EnableVertexAttribArray(1));
            gl!(VertexAttribPointer(0, 3, gl::FLOAT, gl::FALSE, mem::size_of::<Vert>() as _, 0 as _));
            gl!(VertexAttribPointer(1, 3, gl::UNSIGNED_BYTE, gl::FALSE, mem::size_of::<Vert>() as _, (mem::size_of::<[f32; 3]>()) as _));

            gl!(BindVertexArray(0));
            gl!(DisableVertexAttribArray(0));
            gl!(DisableVertexAttribArray(1));
            gl!(BindBuffer(gl::ARRAY_BUFFER, 0));
            gl!(BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0));
        };

        let (program, u_view_proj, u_tint, u_offset) = unsafe { Self::init_program() };

        let params = match RenderParams::load() {
            Ok(params) => params,
            Err(e) => {
                eprintln!("Error reading testbed.json: {}", e);
                Default::default()
            }
        };

        Self {
            vbo,
            ebo,
            vao,
            vbo_data,
            ebo_data,
            program,
            u_view_proj,
            u_tint,
            u_offset,

            params,
            persist: true,

            stats: Default::default(),
        }
    }

    unsafe fn init_program() -> (GLuint, GLint, GLint, GLint) {
        let vert = Self::shader(
            gl::VERTEX_SHADER,
            r#"#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color; // 0-255 ranges
out vec4 v_color;
uniform mat4 u_view_proj;
void main() {
    gl_Position = u_view_proj * vec4(a_position, 1.0);
    v_color = a_color / 255.0;
}
        "#,
        );
        let frag = Self::shader(
            gl::FRAGMENT_SHADER,
            r#"#version 330 core
in vec4 v_color;
out vec4 frag_color;
uniform vec4 u_tint;
uniform vec4 u_offset;
void main() {
    vec4 final_color = v_color * u_tint + u_offset;
    frag_color = final_color;
}
"#,
        );

        let program = gl!(CreateProgram());

        gl!(AttachShader(program, vert));
        gl!(AttachShader(program, frag));
        gl!(LinkProgram(program));

        gl!(DeleteShader(vert));
        gl!(DeleteShader(frag));

        let view_proj = gl!(GetUniformLocation(program, "u_view_proj\0".as_ptr() as _));
        let tint = gl!(GetUniformLocation(program, "u_tint\0".as_ptr() as _));
        let offset = gl!(GetUniformLocation(program, "u_offset\0".as_ptr() as _));
        assert_ne!(-1, view_proj);
        assert_ne!(-1, tint);
        assert_ne!(-1, offset);
        (program, view_proj, tint, offset)
    }

    unsafe fn shader(shader_ty: GLenum, source: &str) -> GLuint {
        let shader = gl!(CreateShader(shader_ty));
        gl!(ShaderSource(
            shader,
            1,
            [source.as_ptr() as _].as_ptr(),
            [source.len() as _].as_ptr()
        ));
        gl!(CompileShader(shader));

        let mut log = vec![0_u8; 1024];
        let mut len = 0;
        gl!(GetShaderInfoLog(shader, 1024, &mut len, log.as_ptr() as _));
        if len > 0 {
            let len = len as usize;
            let msg = str::from_utf8_unchecked(&log[..len]);
            panic!("{}", msg);
        }
        shader
    }

    pub fn frame(&mut self, [x, y, w, h]: [u32; 4]) -> Frame {
        let params = self.params();
        let view_proj = compute_view_proj(&params.camera, [x, y, w, h]);

        unsafe {
            let [r, g, b, a] = params.background;
            gl!(Enable(gl::DEPTH_TEST));
            gl!(Viewport(x as _, y as _, w as _, h as _));
            gl!(ClearColor(r, g, b, a));
            gl!(Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT));
            gl!(UseProgram(self.program));
            gl!(LineWidth(params.wire_size));

            gl!(UniformMatrix4fv(
                self.u_view_proj,
                1,
                gl::FALSE,
                view_proj.as_ptr()
            ));

            gl!(BindVertexArray(self.vao));
            gl!(BindBuffer(gl::ARRAY_BUFFER, self.vbo));
            gl!(BindBuffer(gl::ELEMENT_ARRAY_BUFFER, self.ebo));
        }

        Frame {
            renderer: self,
            primitive: None,
            stats: FrameStats {
                drawcalls: 0,
                verts: 0,
                indices: 0,
            },
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
enum Primitive {
    Triangles,
    Lines,
}

#[derive(ImGuiExt, Clone, Default)]
pub struct FrameStats {
    #[imgui]
    drawcalls: usize,
    #[imgui]
    verts: usize,
    #[imgui]
    indices: usize,
}

/// New rendering frame
pub struct Frame<'a> {
    renderer: &'a mut TestbedRenderer,
    stats: FrameStats,
    primitive: Option<Primitive>,
}

impl<'a> Frame<'a> {
    pub fn render(self) {}

    pub fn line(&mut self, a: Vert, b: Vert) {
        match (self.primitive, self.renderer.ebo_data.len()) {
            // First render call
            (None, 0) => self.primitive = Some(Primitive::Lines),

            // Switch from rendering Triangles to rendering Lines.
            // If we reach this state, there must be buffered geometry, otherwise
            // the state is invalid and a panic is raised
            (Some(Primitive::Triangles), n) if n > 0 => {
                self.flush();
                self.primitive = Some(Primitive::Lines);
            }

            // Keep filling the lines buffer
            (Some(Primitive::Lines), n) if n > 0 => {}

            // If this pattern is matched, there is a bug in
            // the implementation that needs to be fixed.
            _ => panic!(),
        }

        if self.renderer.ebo_data.len() + 2 >= 0xFFFF {
            self.flush();
        }

        let idx = self.renderer.ebo_data.len() as u16;
        self.renderer.ebo_data.extend_from_slice(&[idx, idx + 1]);
        self.renderer.vbo_data.extend_from_slice(&[a, b]);
    }

    pub fn triangle(&mut self, a: Vert, b: Vert, c: Vert) {
        match (self.primitive, self.renderer.ebo_data.len()) {
            // First render call
            (None, 0) => self.primitive = Some(Primitive::Triangles),

            // Switch from rendering Lines to rendering Triangles.
            // If we reach this state, there must be buffered geometry, otherwise
            // the state is invalid and a panic is raised
            (Some(Primitive::Lines), n) if n > 0 => {
                self.flush();
                self.primitive = Some(Primitive::Triangles);
            }

            // Keep filling the triangles buffer
            (Some(Primitive::Triangles), n) if n > 0 => {}

            _ => panic!(),
        }

        if self.renderer.ebo_data.len() + 3 >= 0xFFFF {
            self.flush();
        }

        let idx = self.renderer.ebo_data.len() as u16;
        self.renderer
            .ebo_data
            .extend_from_slice(&[idx, idx + 1, idx + 2]);
        self.renderer.vbo_data.extend_from_slice(&[a, b, c]);
    }

    /// Flush buffered geometry
    ///
    /// Uploads buffered vertex and index buffers to the GPU.
    /// Renders solid and wire-frame geometry.
    ///
    /// # Panics
    /// This method panics if either the vertex or index buffer is empty
    fn flush(&mut self) {
        let verts = self.renderer.vbo_data.len();
        let index = self.renderer.ebo_data.len();
        assert_ne!(0, verts);
        assert_ne!(0, index);
        self.stats.verts += verts;
        self.stats.indices += index;

        unsafe {
            #[rustfmt::skip]
            gl!(BufferSubData(gl::ARRAY_BUFFER,
                              0,
                              (verts * mem::size_of::<Vert>()) as _,
                              self.renderer.vbo_data.as_ptr() as *const _));
            #[rustfmt::skip]
            gl!(BufferSubData(gl::ELEMENT_ARRAY_BUFFER,
                              0,
                              (index * mem::size_of::<u16>()) as _,
                              self.renderer.ebo_data.as_ptr() as *const _));

            let params = self.renderer.params();
            let primitive = match self.primitive {
                Some(Primitive::Triangles) => gl::TRIANGLES,
                Some(Primitive::Lines) => gl::LINES,

                // if this pattern is matched, there is a bug
                None => panic!("bug"),
            };

            gl!(Uniform4f(self.renderer.u_offset, 0.0, 0.0, 0.0, 0.0));
            gl!(Uniform4f(self.renderer.u_tint, 1.0, 1.0, 1.0, 1.0));

            if primitive == gl::TRIANGLES {
                gl!(Enable(gl::POLYGON_OFFSET_FILL));
                gl!(PolygonOffset(1.0, 1.0));
            }

            gl!(DrawElements(
                primitive,
                index as _,
                gl::UNSIGNED_SHORT,
                0 as _
            ));

            if primitive == gl::TRIANGLES {
                gl!(Disable(gl::POLYGON_OFFSET_FILL));
            }

            self.stats.drawcalls += 1;
            self.stats.verts += verts;
            self.stats.indices += index;
        }

        self.renderer.ebo_data.clear();
        self.renderer.vbo_data.clear();
    }

    fn is_empty(&self) -> bool {
        self.renderer.vbo_data.is_empty() && self.renderer.ebo_data.is_empty()
    }
}

impl<'a> Drop for Frame<'a> {
    fn drop(&mut self) {
        if !self.is_empty() {
            self.flush()
        }
        self.renderer.stats = self.stats.clone();
        unsafe {
            gl!(LineWidth(1.0));
            gl!(BindVertexArray(0));
            gl!(UseProgram(0));
            gl!(BindBuffer(gl::ARRAY_BUFFER, 0));
            gl!(BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0));
        }
    }
}
