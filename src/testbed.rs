use std::error::Error;
use std::fs::File;
use std::io::{Read, Write};
use std::ops::Deref;
use std::os::raw::{c_int, c_void};
use std::path::Path;
use std::time::{Instant, Duration};
use std::{mem, slice, str};

use crate::Body;
use crate::body::SleepState;
use crate::ffi;
use crate::math::Vector;
use crate::world::Newton;

use sdl2::mouse::MouseButton;
use sdl2::event::{Event, WindowEvent};
use sdl2::video::{GLContext, Window};
use sdl2::{EventPump, Sdl, VideoSubsystem};

//use gl;
use gl::types::*;

//use imgui;
use imgui::{ImGuiCond, im_str};
use imgui_ext::prelude::*;

use imgui_opengl_renderer::Renderer as ImguiRenderer;
use imgui_sdl2::ImguiSdl2;

use cgmath::prelude::*;
use cgmath::{Vector4, Matrix3, Deg, Matrix4, Point3, Vector3};

use crate::testbed::Primitive::Triangles;
use serde::{Deserialize, Serialize};

pub trait Demo {
    fn reset(newton: &Newton) -> Self;
}

/// Simulation UI
#[doc(hidden)]
#[derive(ImGuiExt, Default)]
pub struct Stats {
    #[imgui(
        checkbox,
        button(label = "Invalidate", catch = "invalidate"),
        button(label = "Reset##Stats", catch = "reset")
    )]
    running: bool,
    #[imgui(display(display = "{:.1?}", 0))]
    elapsed: (Duration,),
    #[imgui(slider(min = 0.01, max = 8.0), separator)]
    time_scale: f32,
    #[imgui(display)]
    bodies: usize,
    #[imgui(display)]
    constraints: usize,
    #[imgui(display)]
    threads: usize,
}

impl Stats {
    fn new() -> Self {
        Self {
            time_scale: 1.0,
            ..Default::default()
        }
    }
}

#[derive(ImGuiExt)]
pub struct SelectedBody {
    ptr: *const ffi::NewtonBody,
    #[imgui(display(display = "{:?}", 0))]
    name: (Option<&'static str>, ),
    #[imgui(drag(speed = 0.1))]
    position: [f32; 3],
}

impl Default for SelectedBody {
    fn default() -> Self {
        Self {
            ptr: 0 as _,
            name: (None, ),
            position: Default::default(),
        }
    }
}

#[derive(ImGuiExt)]
pub struct Testbed<T> {
    demo: T,
    newton: Newton,

    sdl: Sdl,
    sdl_video: VideoSubsystem,
    sdl_window: Window,
    sdl_gl: GLContext,
    sdl_events: EventPump,

    selected: Option<SelectedBody>,

    // FIXME(imgui-ext) this type (Controls) shouldn't need to be public...
    #[imgui(nested)]
    controls: Stats,
    #[imgui(separator, nested)]
    renderer: Renderer,
}

impl<T: Demo> Testbed<T> {
    pub fn run() -> Result<(), Box<dyn Error>> {
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
        let controls = Stats::new();

        let newton = Newton::create();
        let demo = T::reset(&newton);

        Self {
            selected: None,
            demo,
            newton,
            sdl,
            sdl_events,
            sdl_video,
            sdl_window,
            sdl_gl,
            controls,
            renderer,
        }
        .main_loop()?;
        Ok(())
    }

    fn main_loop(mut self) -> Result<(), Box<dyn Error>> {
        self.sdl_window.show();

        let mut imgui = imgui::ImGui::init();
        let mut imgui_sdl = ImguiSdl2::new(&mut imgui);
        let mut imgui_renderer =
            ImguiRenderer::new(&mut imgui, |s| self.sdl_video.gl_get_proc_address(s) as _);

        let mut time = 0.0f32;
        let mut delta_time = Instant::now();

        'mainLoop: loop {
            let left = self.sdl_events.mouse_state().left();
            let right = self.sdl_events.mouse_state().right();
            for event in self.sdl_events.poll_iter() {
                imgui_sdl.handle_event(&mut imgui, &event);
                if !imgui_sdl.ignore_event(&event) {
                    match (self.renderer.params.camera.controller, event) {
                        (
                            _,
                            Event::Window {
                                win_event: WindowEvent::Close,
                                ..
                            },
                        ) => break 'mainLoop,
                        (_, Event::MouseButtonUp { mouse_btn: MouseButton::Left, x, y, .. }) => {
                            // clip space
                            let (w, h) = self.sdl_window.size();
                            let camera = &self.renderer.params.camera;
                            let (start, end) = compute_ray(&camera, x, y, [0, 0, w, h]);

                            let mut min = 2.0;
                            let mut selected = None;
                            self.newton.ray_cast(&start, &end, |body, _, _, _, hit| {
                                if hit < min {
                                    selected = Some(body.as_ptr());
                                    min = hit;
                                }
                                hit
                            }, None);
                            self.selected = selected.map(|s| SelectedBody { ptr: s, ..Default::default() });
                        }
                        (true, Event::MouseMotion { xrel, yrel, .. }) => {
                            if right {
                                self.renderer.params.camera.pan(xrel as f32, yrel as f32)
                            }
                            if left {
                                self.renderer.params.camera.orbit(xrel as f32, yrel as f32)
                            }
                        }
                        (true, Event::MouseWheel { y, .. }) => {
                            self.renderer.params.camera.scroll(y as f32)
                        }
                        _ => {}
                    }
                }
            }

            self.controls.bodies = self.newton.bodies().count();
            self.controls.threads = self.newton.threads();
            self.controls.constraints = self.newton.constraints();

            let now = Instant::now();
            let mut elapsed = now - delta_time;
            if self.controls.running {
                let seconds = self.controls.time_scale / 60.0;
                let step = Duration::new(
                    seconds.floor() as u64,
                    (1_000_000_000.0 * seconds.fract()) as u32,
                );

                while elapsed > step {
                    self.newton.update(step);
                    self.controls.elapsed.0 += step;
                    elapsed -= step;
                    delta_time = Instant::now() - elapsed;
                }
            } else {
                delta_time += elapsed;
            }

            let params = self.renderer.params().clone();
            let mut frame = self.renderer.frame(&self.sdl_window);

            for body in self.newton.bodies().iter() {
                let [r, g, b, _] = match body.sleep_state() {
                    SleepState::Active => params.active,
                    SleepState::Sleeping => params.sleeping,
                };

                let color = match self.selected {
                    Some(SelectedBody { ptr, .. }) if ptr == body.as_ptr() => {
                        let [r, g, b, _] = params.selected;
                        [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8]
                    },
                    _ => [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8],
                };
                let collision = body.collision();
                let matrix = body.matrix();

                #[rustfmt::skip]
                collision.polygons(&matrix, |face, _face_id| {
                    let mut pos = face.chunks(3);
                    let a = pos.next().map(|s| [s[0], s[1], s[2]]);
                    let b = pos.next().map(|s| [s[0], s[1], s[2]]);
                    let c = pos.next().map(|s| [s[0], s[1], s[2]]);
                    let d = pos.next().map(|s| [s[0], s[1], s[2]]);
                    let e = pos.next().map(|s| [s[0], s[1], s[2]]);

                    match (a, b, c, d, e) {
                        // Triangle face
                        (Some(a), Some(b), Some(c), None, None) => {
                            frame.triangle(Vert {pos: a, color }, Vert { pos: b, color }, Vert { pos: c, color });
                        },

                        // Quad face.
                        // Assumes the four points are coplanar.
                        (Some(a), Some(b), Some(c), Some(d), None) => {
                            frame.triangle(Vert {pos: a, color }, Vert { pos: b, color }, Vert { pos: c, color });
                            frame.triangle(Vert {pos: a, color }, Vert { pos: c, color }, Vert { pos: d, color });
                        },

                        // Unexpected number of vertices. Face is neither a triangle nor a quad.
                        // If this panic is ever raised, check the docs or the original implementation.
                        _ => panic!("Unexpected number of faces")
                    }
                });
            }

            if params.aabb {
                for body in self.newton.bodies().iter() {
                    let Vector { x, y, z, w: _ } = body.matrix().c3;
                    let (min, max) = body.aabb();
                    let [r, g, b, _] = params.aabb_color;
                    let color = [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8];
                    frame.line(Vert { pos: [min.x, min.y, min.z], color }, Vert { pos: [max.x, min.y, min.z], color });
                    frame.line(Vert { pos: [min.x, min.y, min.z], color }, Vert { pos: [min.x, max.y, min.z], color });
                    frame.line(Vert { pos: [min.x, min.y, min.z], color }, Vert { pos: [min.x, min.y, max.z], color });
                    frame.line(Vert { pos: [max.x, max.y, max.z], color }, Vert { pos: [min.x, max.y, max.z], color });
                    frame.line(Vert { pos: [max.x, max.y, max.z], color }, Vert { pos: [max.x, min.y, max.z], color });
                    frame.line(Vert { pos: [max.x, max.y, max.z], color }, Vert { pos: [max.x, max.y, min.z], color });
                    frame.line(Vert { pos: [min.x, max.y, min.z], color }, Vert { pos: [max.x, max.y, min.z], color });
                    frame.line(Vert { pos: [min.x, max.y, min.z], color }, Vert { pos: [min.x, max.y, max.z], color });
                    frame.line(Vert { pos: [max.x, min.y, max.z], color }, Vert { pos: [min.x, min.y, max.z], color });
                    frame.line(Vert { pos: [max.x, min.y, max.z], color }, Vert { pos: [max.x, min.y, min.z], color });
                    frame.line(Vert { pos: [max.x, min.y, min.z], color }, Vert { pos: [max.x, max.y, min.z], color });
                    frame.line(Vert { pos: [min.x, min.y, max.z], color }, Vert { pos: [min.x, max.y, max.z], color });
                }
            }

            if params.axis {
                frame.line(Vert{ pos: [0.0,0.0,0.0], color: [255,0,0] }, Vert{ pos: [1024.0,0.0,0.0], color: [255,0,0] });
                frame.line(Vert{ pos: [0.0,0.0,0.0], color: [0,255,0] }, Vert{ pos: [0.0,1024.0,0.0], color: [0,255,0] });
                frame.line(Vert{ pos: [0.0,0.0,0.0], color: [0,0,255] }, Vert{ pos: [0.0,0.0,1024.0], color: [0,0,255] });
            }

            frame.render();

            let ui = imgui_sdl.frame(&self.sdl_window, &mut imgui, &self.sdl_events.mouse_state());

            let (w, h) = self.sdl_window.size();
            let width = 256.0;
            let margin = 8.0;

            ui.window(im_str!("Controls"))
                .title_bar(false)
                .movable(false)
                .position((w as f32 - width - margin, margin), ImGuiCond::Always)
                .size((width, h as f32 - margin * 2.0), ImGuiCond::Always)
                .resizable(false)
                .build(|| {
                    let events = ui.imgui_ext(&mut self);
                    if events.controls().invalidate() {
                        self.newton.invalidate();
                    }
                    if events.controls().reset() {
                        self.controls.elapsed = (Duration::default(),);
                        let newton = Newton::create();
                        let demo = T::reset(&newton);

                        self.newton = newton;
                        self.demo = demo;
                    }
                    if events.renderer().params().reset() {
                        self.renderer.params = RenderParams::default();
                    }
                    if events.renderer().params().camera().reset() {
                        let camera = &mut self.renderer.params.camera;
                        let cont = camera.controller;
                        *camera = Default::default();
                        camera.controller = cont;
                    }
            });
            if let Some(mut sel) = self.selected.take() {
                let ptr = sel.ptr;
                let body = unsafe { Body::from_raw(&self.newton, ptr as _) };
                let mut matrix = body.matrix();
                sel.position = [matrix.c3.x, matrix.c3.y, matrix.c3.z];
                sel.name = (body.name(), );
                ui.window(im_str!("Body"))
                    .title_bar(false)
                    .movable(false)
                    .resizable(false)
                    //.position((w as f32 - width*2.0 - margin*2.0, margin), ImGuiCond::Always)
                    .position((margin, margin), ImGuiCond::Always)
                    //.size((width, 200.0), ImGuiCond::Always)
                    .always_auto_resize(true)
                    .build(|| {
                        let events = ui.imgui_ext(&mut sel);
                        if events.position() {
                            matrix.c3 = Vector::from(sel.position);
                            body.set_matrix(&matrix);
                        }
                    });

                self.selected = Some(sel);
            }
            imgui_renderer.render(ui);

            self.sdl_window.gl_swap_window();
        }

        Ok(())
    }
}

#[derive(ImGuiExt, Serialize, Deserialize, Clone)]
pub struct Camera {
    #[imgui(checkbox(label = "Mouse control"))]
    controller: bool,
    #[imgui(drag(speed = 0.1))]
    eye: [f32; 3],
    #[imgui(drag(speed = 0.1))]
    center: [f32; 3],
    #[imgui(slider(min = 1.0, max = 179.0))]
    fov: f32,
    #[imgui(slider(min = 0.01, max = 2000.0))]
    near: f32,
    #[imgui(
        slider(min = 0.01, max = 2000.0),
        button(label = "Reset camera", catch = "reset"),
    )]
    far: f32,
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
fn compute_view_proj(camera: &Camera, viewport: [u32; 4]) -> Matrix4<f32> {
    let aspect = viewport[2] as f32 / viewport[3] as f32;
    let proj = cgmath::perspective(Deg(camera.fov), aspect, camera.near, camera.far);
    let view = Matrix4::look_at(Point3::from(camera.eye),
                                Point3::from(camera.center),
                                Vector3::new(0.0, 1.0, 0.0));
    proj * view
}

#[rustfmt::skip]
fn compute_ray(camera: &Camera, mx: i32, my: i32, viewport: [u32; 4]) -> (Vector, Vector) {
    let [_, _, w, h] = viewport;
    let view_proj = compute_view_proj(camera, viewport);
    let view_proj_inv = view_proj.invert().unwrap();
    let mut start = Vector4::new(mx as f32 / w as f32 * 2.0 - 1.0, my as f32 / h as f32 * 2.0 - 1.0, 0.0, 1.0);
    let mut end = start;
    start.y *= -1.0;
    end.y *= -1.0;
    end.z = 1.0;

    //println!("{:?}", start);
    let mut s = view_proj_inv * start;
    let mut e = view_proj_inv * end;
    s /= s.w;
    e /= e.w;

    (Vector::new3(s.x, s.y, s.z), Vector::new3(e.x, e.y, e.z))
}

impl Camera {
    fn r(&self) -> [f32; 3] {
        let [cx, cy, cz] = self.center;
        let [ex, ey, ez] = self.eye;
        [ex - cx, ey - cy, ez - cz]
    }

    fn dist(&self) -> f32 {
        let [x, y, z] = self.r();
        (x*x+y*y+z*z).sqrt()
    }

    fn basis(&self) -> Matrix3<f32> {
        let mut radius = -Vector3::from(self.r()).normalize();
        let up = Vector3::new(0.0, 1.0, 0.0);
        let up = up - radius * cgmath::dot(up, radius);
        let right = up.cross(radius);
        // TODO redundant normalization
        Matrix3::from_cols(right.normalize(), up.normalize(), radius.normalize())
    }

    fn orbit(&mut self, xrel: f32, yrel: f32) {
        let mut radius = Vector3::from(self.r()).normalize();
        let up = Vector3::new(0.0, 1.0, 0.0);
        let up = up - radius * cgmath::dot(up, radius);
        let right = up.cross(radius);

        let k = 0.005;
        let radius = (radius + up*yrel*k - right*xrel*k).normalize() * self.dist();

        self.eye = self.center;
        self.eye[0] += radius.x;
        self.eye[1] += radius.y;
        self.eye[2] += radius.z;
    }

    fn pan(&mut self, xrel: f32, yrel: f32) {
        let up = Vector3::new(0.0, 1.0, 0.0);
        let look = Vector3::from(self.center) - Vector3::from(self.eye);
        let look = look.normalize();
        let right = up.cross(look);

        let [x, y, z] = self.r();
        let dist = (x*x + y*y + z*z).sqrt();

        let k = 0.001 * dist;
        self.center[1] += up.y * yrel * k;
        self.eye[1] += up.y * yrel * k;

        self.center[0] += right.x * xrel * k;
        self.center[1] += right.y * xrel * k;
        self.center[2] += right.z * xrel * k;

        self.eye[0] += right.x * xrel * k;
        self.eye[1] += right.y * xrel * k;
        self.eye[2] += right.z * xrel * k;
    }

    fn scroll(&mut self, n: f32) {
        let [x, y, z] = self.r();
        let dist = (x*x + y*y + z*z).sqrt();
        self.eye[0] -= n * x / dist;
        self.eye[1] -= n * y / dist;
        self.eye[2] -= n * z / dist;
    }
}

#[derive(ImGuiExt, Serialize, Deserialize, Clone)]
#[doc(hidden)]
pub struct RenderParams {
    #[imgui(
        button(label = "Reset##RenderParams", catch = "reset"),
        new_line,
        nested
    )]
    camera: Camera,
    // colors
    #[imgui(new_line, color(edit))]
    background: [f32; 4],
    #[imgui(color(edit))]
    wireframe: [f32; 4],
    #[imgui(color(edit))]
    active: [f32; 4],
    #[imgui(color(edit))]
    sleeping: [f32; 4],
    #[imgui(color(edit))]
    selected: [f32; 4],
    #[imgui(color(edit))]
    aabb_color: [f32; 4],

    #[imgui(new_line, checkbox(label = "Solid"))]
    solid: bool,

    #[imgui(checkbox(label = "Wireframe"))]
    wire: bool,
    #[imgui(slider(min = 1.0, max = 4.0))]
    wire_size: f32,
    #[imgui(checkbox)]
    axis: bool,
    #[imgui(checkbox)]
    individual_axis: bool,

    #[imgui(checkbox(label = "AABB"))]
    aabb: bool,
    #[imgui(checkbox(label = "Names"))]
    names: bool,
    #[imgui(checkbox(label = "Lighting"))]
    lighting: bool,
    #[imgui(checkbox(label = "Shadows"))]
    shadows: bool,
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

            lighting: false,
            shadows: false,
        }
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

impl Drop for Renderer {
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

    fn params(&self) -> &RenderParams {
        &self.params
    }

    fn frame(&mut self, window: &Window) -> Frame {
        // compute projection matrix
        let (w, h) = window.size();

        let params = self.params();
        let view_proj = compute_view_proj(&params.camera, [0, 0, w, h]);

        /*
        #[rustfmt::skip]
        let proj = cgmath::perspective(Deg(fov), w as f32 / h as f32, near, far);
        #[rustfmt::skip]
        let view = Matrix4::look_at(Point3::from(eye),
                                    Point3::from(center),
                                    Vector3::new(0.0, 1.0, 0.0));
                                    */

        unsafe {
            let [r, g, b, a] = params.background;
            gl!(Enable(gl::DEPTH_TEST));
            gl!(Viewport(0, 0, w as _, h as _));
            gl!(ClearColor(r, g, b, a));
            gl!(Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT));
            gl!(UseProgram(self.program));

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

#[doc(hidden)]
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
#[doc(hidden)]
struct Frame<'a> {
    renderer: &'a mut Renderer,
    stats: FrameStats,
    primitive: Option<Primitive>,
}

impl<'a> Frame<'a> {
    fn render(self) {}

    fn line(&mut self, a: Vert, b: Vert) {
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

    fn triangle(&mut self, a: Vert, b: Vert, c: Vert) {
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
            if params.wire && primitive != gl::LINES {
                let [r, g, b, _] = params.wireframe;
                gl!(LineWidth(params.wire_size));
                gl!(PolygonMode(gl::FRONT_AND_BACK, gl::LINE));
                gl!(Uniform4f(self.renderer.u_tint, 0.0, 0.0, 0.0, 1.0));
                gl!(Uniform4f(self.renderer.u_offset, r, g, b, 0.0));
                gl!(DrawElements(
                    primitive,
                    index as _,
                    gl::UNSIGNED_SHORT,
                    0 as _
                ));
                gl!(LineWidth(1.0));
                gl!(PolygonMode(gl::FRONT_AND_BACK, gl::FILL));
                self.stats.drawcalls += 1;
                self.stats.verts += verts;
                self.stats.indices += index;
            }

            if params.solid {
                gl!(Uniform4f(self.renderer.u_offset, 0.0, 0.0, 0.0, 0.0));
                gl!(Uniform4f(self.renderer.u_tint, 1.0, 1.0, 1.0, 1.0));
                gl!(DrawElements(
                    primitive,
                    index as _,
                    gl::UNSIGNED_SHORT,
                    0 as _
                ));
                self.stats.drawcalls += 1;
                self.stats.verts += verts;
                self.stats.indices += index;
            }
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
            gl!(BindVertexArray(0));
            gl!(UseProgram(0));
            gl!(BindBuffer(gl::ARRAY_BUFFER, 0));
            gl!(BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0));
        }
    }
}
