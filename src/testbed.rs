use std::io::{Read, Write};
use std::fs::File;
use std::os::raw::{c_void, c_int};
use std::slice;
use std::mem;
use std::error::Error;
use std::ops::Deref;
use std::time::Duration;
use std::path::Path;

use super::ffi;
use super::world::Newton;

use sdl2::event::{Event, WindowEvent};
use sdl2::video::{GLContext, Window};
use sdl2::{EventPump, Sdl, VideoSubsystem};

//use gl;
use gl::types::*;

//use imgui;
use imgui_ext::prelude::*;

use imgui_opengl_renderer::Renderer as ImguiRenderer;
use imgui_sdl2::ImguiSdl2;

use cgmath::prelude::*;
use cgmath::{Deg, Matrix4, Point3, Vector3};

use serde::{Serialize, Deserialize};

/// Simulation UI
#[doc(hidden)]
#[derive(ImGuiExt, Default)]
pub struct Stats {
    #[imgui(
        checkbox,
        button(label = "Invalidate", catch = "invalidate"),
        button(label = "Reset", catch = "reset"),
    )]
    running: bool,
    #[imgui(display(display = "{:?}", 0))]
    elapsed: (Duration,),
    #[imgui(display)]
    bodies: usize,
    #[imgui(display)]
    contacts: usize,
}

#[derive(ImGuiExt)]
pub struct Testbed {
    newton: Newton,

    sdl: Sdl,
    sdl_video: VideoSubsystem,
    sdl_window: Window,
    sdl_gl: GLContext,
    sdl_events: EventPump,

    // FIXME(imgui-ext) this type (Controls) shouldn't need to be public...
    #[imgui(nested)]
    controls: Stats,
    #[imgui(separator, nested)]
    renderer: Renderer,
}

impl Testbed {
    pub fn new(newton: Newton) -> Result<Self, Box<dyn Error>> {
        let sdl = sdl2::init()?;
        let sdl_events = sdl.event_pump()?;
        let sdl_video = sdl.video()?;
        let sdl_window = sdl_video
            .window("Newton testbed", 800, 600)
            .opengl()
            .position_centered()
            .resizable()
            .allow_highdpi()
            .hidden()
            .build()?;

        let sdl_gl = sdl_window.gl_create_context()?;
        sdl_window.gl_make_current(&sdl_gl)?;

        let renderer = Renderer::new(|s| sdl_video.gl_get_proc_address(s));
        let controls = Default::default();

        Ok(Self {
            newton,

            sdl,
            sdl_events,
            sdl_video,
            sdl_window,
            sdl_gl,

            controls,
            renderer,
        })
    }

    pub fn run(mut self) -> Result<(), Box<dyn Error>> {
        self.sdl_window.show();

        let mut imgui = imgui::ImGui::init();
        let mut imgui_sdl = ImguiSdl2::new(&mut imgui);
        let mut imgui_renderer =
            ImguiRenderer::new(&mut imgui, |s| self.sdl_video.gl_get_proc_address(s) as _);

        let mut time = 0.0f32;

        'mainLoop: loop {
            for event in self.sdl_events.poll_iter() {
                imgui_sdl.handle_event(&mut imgui, &event);
                if !imgui_sdl.ignore_event(&event) {
                    match event {
                        Event::Window {
                            win_event: WindowEvent::Close,
                            ..
                        } => break 'mainLoop,
                        _ => {}
                    }
                }
            }

            if self.controls.running {
                let step = Duration::new(0, 1_000_000_000 / 60);
                self.newton.update(step);
            }

            let mut frame = self.renderer.frame(&self.sdl_window);

            unsafe {
                let mut body = ffi::NewtonWorldGetFirstBody(self.newton.as_ptr());
                while !body.is_null() {
                    let mut matrix: [f32; 16] = Default::default();
                    let collision = ffi::NewtonBodyGetCollision(body);
                    ffi::NewtonBodyGetMatrix(body, matrix.as_mut_ptr());
                    ffi::NewtonCollisionForEachPolygonDo(collision, matrix.as_ptr(), Some(polygons), mem::transmute(&mut frame));
                    #[rustfmt::skip]
                    unsafe extern "C" fn polygons (user_data: *const c_void, vertex_count: c_int, face_array: *const f32, face_id: c_int) {
                        let frame: &mut Frame = mem::transmute(user_data);
                        let mut pos = slice::from_raw_parts(face_array, vertex_count as usize * 3).chunks(3);
                        let a = pos.next().map(|s| [s[0], s[1], s[2]]).unwrap();
                        let b = pos.next().map(|s| [s[0], s[1], s[2]]).unwrap();
                        let c = pos.next().map(|s| [s[0], s[1], s[2]]).unwrap();
                        let d = pos.next().map(|s| [s[0], s[1], s[2]]);
                        let color = [255, 255, 255];
                        frame.triangle(Vert {pos: a, color }, Vert { pos: b, color }, Vert { pos: c, color });
                        if let Some(d) = d { frame.triangle(Vert {pos: a, color }, Vert { pos: c, color }, Vert { pos: d, color }) }
                    }
                    body = ffi::NewtonWorldGetNextBody(self.newton.as_ptr(), body);
                }
            }

            frame.render();

            let ui = imgui_sdl.frame(&self.sdl_window, &mut imgui, &self.sdl_events.mouse_state());
            if ui.imgui_ext(&mut self).controls().invalidate() {
                self.newton.invalidate();
            }
            imgui_renderer.render(ui);

            self.sdl_window.gl_swap_window();
        }

        Ok(())
    }
}

#[derive(ImGuiExt, Serialize, Deserialize)]
pub struct Camera {
    #[imgui(drag(speed = 0.1))]
    position: [f32; 3],
    #[imgui(slider(min = 1.0, max = 179.0))]
    fov: f32,
    #[imgui(slider(min = 0.01, max = 2000.0))]
    near: f32,
    #[imgui(slider(min = 0.01, max = 2000.0))]
    far: f32,
}

impl Camera {
    fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn Error>> {
        let mut file = File::open(path)?;
        Ok(serde_json::from_reader(file)?)
    }

    fn save_to_file<P: AsRef<Path>>(&self, path: P) -> Result<(), Box<dyn Error>> {
        let mut file = File::create(path)?;
        Ok(serde_json::to_writer(file, self)?)
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct Vert {
    pos: [f32; 3],
    color: [u8; 3],
}

/// Immediate renderer. Its main functions are:
///
/// - Collect raw geometry (vertex data & index data)
/// - Set a view & projection transformation
#[derive(ImGuiExt)]
#[doc(hidden)]
pub struct Renderer {
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

    #[imgui(color(edit))]
    background: [f32; 4],
    #[imgui(color(edit))]
    wireframe: [f32; 4],
    // sim states
    #[imgui(color(edit))]
    active: [f32; 4],
    #[imgui(color(edit))]
    inactive: [f32; 4],

    #[imgui(checkbox(label = "Wireframe"))]
    wire: bool,
    #[imgui(checkbox(label = "Lighting"))]
    lighting: bool,
    #[imgui(checkbox(label = "Shadows"))]
    shadows: bool,

    #[imgui(nested)]
    camera: Camera,
    #[imgui(checkbox(label = "Save camera state"))]
    camera_save: bool,

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

impl Drop for Renderer {
    fn drop(&mut self) {
        if self.camera_save {
            match self.camera.save_to_file("camera.json") {
                Err(e) => eprintln!("Error saving camera.json: {}", e),
                Ok(_) => {},
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

impl Renderer {
    fn new<F: FnMut(&str) -> *const ()>(mut loader: F) -> Self {
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

        let (program, u_view_proj, u_tint, u_offset) = unsafe {
            Self::init_program()
        };

        let camera = match Camera::from_file("camera.json") {
            Ok(camera) => camera,
            Err(e) => {
                eprintln!("Error reading camera.json: {}", e);
                Camera {
                    fov: 55.0,
                    near: 0.01,
                    far: 1000.0,
                    position: [2.0, 1.0, 6.0],
                }
            },
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
            background: [0.2, 0.2, 0.2, 1.0],
            wireframe: [0.0, 0.0, 0.0, 1.0],
            active: [1.0, 0.5, 1.0, 1.0],
            inactive: [1.0, 1.0, 1.0, 1.0],
            wire: true,
            lighting: true,
            shadows: true,
            camera,
            camera_save: true,
            stats: FrameStats {
                drawcalls: 0,
                verts: 0,
                indices: 0,
            },
        }
    }

    unsafe fn init_program() -> (GLuint, GLint, GLint, GLint) {
        let vert = Self::shader(gl::VERTEX_SHADER, r#"#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color; // 0-255 ranges
out vec4 v_color;
uniform mat4 u_view_proj;
void main() {
    gl_Position = u_view_proj * vec4(a_position, 1.0);
    v_color = a_color / 255.0;
}
        "#);
        let frag = Self::shader(gl::FRAGMENT_SHADER, r#"#version 330 core
in vec4 v_color;
out vec4 frag_color;
uniform vec4 u_tint;
uniform vec4 u_offset;
void main() {
    frag_color = v_color * u_tint + u_offset;
}
"#);

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
        gl!(ShaderSource(shader, 1, [source.as_ptr() as _].as_ptr(), [source.len() as _].as_ptr()));
        gl!(CompileShader(shader));

        let mut log = vec![0_u8; 1024];
        let mut len = 0;
        gl!(GetShaderInfoLog(shader, 1024, &mut len, log.as_ptr() as _));
        if len > 0 {
            let len = len as usize;
            let msg = ::std::str::from_utf8_unchecked(&log[..len]);
            panic!("{}", msg);
        }
        shader
    }

    fn frame(&mut self, window: &Window) -> Frame {
        // compute projection matrix
        let (w, h) = window.size();
        #[rustfmt::skip]
        let proj = cgmath::perspective(Deg(self.camera.fov),
                                       w as f32 / h as f32,
                                       self.camera.near,
                                       self.camera.far);
        #[rustfmt::skip]
        let view = Matrix4::look_at(Point3::from(self.camera.position),
                                    Point3::new(0.0, 0.0, 0.0),
                                    Vector3::new(0.0, 1.0, 0.0));

        unsafe {
            let [r, g, b, a] = self.background;
            gl!(Enable(gl::DEPTH_TEST));
            gl!(Viewport(0, 0, w as _, h as _));
            gl!(ClearColor(r, g, b, a));
            gl!(Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT));
            gl!(UseProgram(self.program));

            let view_proj = proj * view;
            gl!(UniformMatrix4fv(self.u_view_proj, 1, gl::FALSE, view_proj.as_ptr()));

            gl!(BindVertexArray(self.vao));
            gl!(BindBuffer(gl::ARRAY_BUFFER, self.vbo));
            gl!(BindBuffer(gl::ELEMENT_ARRAY_BUFFER, self.ebo));
        }

        Frame {
            renderer: self,
            stats: FrameStats {
                drawcalls: 0,
                verts: 0,
                indices: 0,
            },
        }
    }
}

#[doc(hidden)]
#[derive(ImGuiExt, Clone)]
pub struct FrameStats {
    #[imgui]
    drawcalls: usize,
    #[imgui]
    verts: usize,
    #[imgui]
    indices: usize,
}

/// New rendering frame
#[doc(hidden)]
struct Frame<'a> {
    renderer: &'a mut Renderer,
    stats: FrameStats,
}

impl<'a> Frame<'a> {
    fn render(self) {}

    fn triangle(&mut self, a: Vert, b: Vert, c: Vert) {
        if self.renderer.ebo_data.len() + 3 >= 0xFFFF {
            self.flush();
        }

        let idx = self.renderer.ebo_data.len() as u16;
        self.renderer.ebo_data.extend_from_slice(&[idx, idx+1, idx+2]);
        self.renderer.vbo_data.extend_from_slice(&[a, b, c]);
    }

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

            if self.renderer.wire {
                let [r, g, b, _] = self.renderer.wireframe;
                gl!(LineWidth(3.0));
                gl!(PolygonMode(gl::FRONT_AND_BACK, gl::LINE));
                gl!(Uniform4f(self.renderer.u_tint, 0.0, 0.0, 0.0, 1.0));
                gl!(Uniform4f(self.renderer.u_offset, r, g, b, 0.0));
                gl!(DrawElements(gl::TRIANGLES, index as _, gl::UNSIGNED_SHORT, 0 as _));
                gl!(LineWidth(1.0));
                gl!(PolygonMode(gl::FRONT_AND_BACK, gl::FILL));
                self.stats.drawcalls += 1;
            }

            gl!(Uniform4f(self.renderer.u_offset, 0.0, 0.0, 0.0, 0.0));
            gl!(Uniform4f(self.renderer.u_tint, 1.0, 1.0, 1.0, 1.0));
            gl!(DrawElements(gl::TRIANGLES, index as _, gl::UNSIGNED_SHORT, 0 as _));
            self.stats.drawcalls += 1;
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
        if !self.is_empty() { self.flush() }
        self.renderer.stats = self.stats.clone();
        unsafe {
            gl!(BindVertexArray(0));
            gl!(UseProgram(0));
        }
    }
}

