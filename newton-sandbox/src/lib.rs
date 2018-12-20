pub mod math;
mod renderer;

use std::marker::PhantomData;
use std::rc::Rc;
use std::time::{Duration, Instant};

use newton::body::SleepState;

use cgmath::prelude::*;
use cgmath::{perspective, Deg, Matrix4, Point3, Vector3};

use self::renderer::{Mode, Primitive, RenderStats, Renderer};

use imgui::im_str;
use imgui::{ImGui, ImGuiCond, ImString, Ui};

pub use self::renderer::Color;

#[derive(Debug)]
pub enum SandboxData {}
impl newton::NewtonConfig for SandboxData {
    const GRAVITY: math::Vector3<f32> = math::Vector3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    };
    type Vector3 = math::Vector3<f32>;
    type Vector4 = math::Vector4<f32>;
    type Matrix4 = math::Matrix4<f32>;
    type Quaternion = math::Quaternion<f32>;
    type Collision = ();
}

pub trait EventHandler {
    fn event(&mut self, event: &Event) {}
}

impl EventHandler for () {}

// re-export newton types

pub type NewtonWorld = newton::world::NewtonWorld<SandboxData>;
pub type NewtonBody = newton::body::NewtonBody<SandboxData>;
pub type NewtonCuboid = newton::collision::NewtonCuboid<SandboxData>;

#[macro_export]
macro_rules! rgba {
    ($r:expr, $g:expr, $b:expr, $a:expr) => {
        $crate::Color {
            r: $r,
            g: $g,
            b: $b,
            a: $a,
        }
    };
    ($r:expr, $g:expr, $b:expr) => {
        rgba!($r, $g, $b, 1.0)
    };
    ($r:expr) => {
        rgba!($r, $r, $r)
    };
}

pub use sdl2::event::Event;
pub use sdl2::event::WindowEvent;
pub use sdl2::keyboard::{Keycode, Mod, Scancode};
pub use sdl2::mouse::MouseButton;

pub struct Sandbox {
    handler: Option<Box<EventHandler>>,

    // window
    width: usize,
    height: usize,

    // render
    background: Color,
    awake_color: Color,
    sleep_color: Color,
    lighting: bool,

    keyboard: bool,

    simulate: bool,
    elapsed: Duration,
    time_scale: f32,

    wireframe: bool,
    aabb: bool,
    constraints: bool,
    bodies: bool,
    stats: bool,
}

impl Sandbox {
    pub fn new() -> Sandbox {
        Sandbox {
            handler: None,
            background: rgba!(1.0, 1.0, 1.0),

            simulate: false,
            elapsed: Duration::new(0, 0),
            time_scale: 1.0,

            awake_color: rgba!(1.0, 0.0, 1.0),
            sleep_color: rgba!(1.0, 1.0, 1.0),
            lighting: true,

            width: 800,
            height: 600,

            wireframe: false,
            keyboard: false,
            aabb: true,
            constraints: false,
            bodies: true,

            stats: true,
        }
    }

    pub fn size(mut self, width: usize, height: usize) -> Self {
        self.width = width;
        self.height = height;
        self
    }

    /*
    pub fn event_handler(&mut self, handler: E) {
        self.handler = Some(handler);
    }
    */

    pub fn background_color(&mut self, background: Color) {
        self.background = background;
    }

    pub fn set_wireframe(&mut self, wireframe: bool) {
        self.wireframe = wireframe;
    }

    pub fn set_aabb(&mut self, aabb: bool) {
        self.aabb = aabb;
    }

    fn render(&self, renderer: &Renderer, bodies: &[NewtonBody]) -> RenderStats {
        renderer.reset();

        let mut stats = RenderStats {
            tris: 0,
            drawcalls: 0,
        };

        if self.bodies {
            renderer.set_lighting(self.lighting);

            for body in bodies.iter() {
                let mut transform = Matrix4::identity();
                unsafe {
                    // XXX pointers
                    std::ptr::copy(
                        &body.matrix() as *const _ as *const f32,
                        &mut transform[0][0] as *mut f32,
                        16,
                    );
                }
                let color = match body.sleep_state() {
                    SleepState::Sleeping => self.sleep_color,
                    SleepState::Awake => self.awake_color,
                };
                let mode = if self.wireframe {
                    Mode::Wireframe
                } else {
                    Mode::Fill
                };
                renderer.render(Primitive::Box, mode, color, transform, Some(&mut stats));
            }
        }

        if self.aabb {
            // aabb
            // XXX pointers
            for body in bodies.iter() {
                unsafe {
                    let mut aabb: (Vector3<f32>, Vector3<f32>) = std::mem::zeroed();
                    let mut position: Vector3<f32> = std::mem::zeroed();

                    std::ptr::copy(
                        &body.position() as *const _ as *const f32,
                        &mut position as *mut _ as *mut f32,
                        3,
                    );
                    std::ptr::copy(
                        &body.aabb() as *const _ as *const f32,
                        &mut aabb as *mut _ as *mut f32,
                        6,
                    );

                    let scale = aabb.1 - aabb.0;

                    let mut transform = Matrix4::from_translation(position)
                        * Matrix4::from_nonuniform_scale(scale.x, scale.y, scale.z);

                    renderer.render(
                        Primitive::Box,
                        Mode::Wireframe,
                        rgba!(0.0),
                        transform,
                        Some(&mut stats),
                    );
                }
            }
        }

        stats
    }

    pub fn run<B>(mut self, world: NewtonWorld, bodies: B)
    where
        B: Fn() -> Vec<NewtonBody>,
    {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("Window", self.width as _, self.height as _)
            .opengl()
            .position_centered()
            .build()
            .unwrap();

        let attr = video_subsystem.gl_attr();
        attr.set_context_profile(sdl2::video::GLProfile::Core);
        attr.set_context_version(3, 3);

        let gl_context = window.gl_create_context().unwrap();
        window.gl_make_current(&gl_context).unwrap();

        gl::load_with(|s| video_subsystem.gl_get_proc_address(s) as _);

        let renderer = get_renderer();

        let aspect = (self.width as f32) / (self.height as f32);
        let proj = perspective(Deg(55.0_f32), aspect, 0.01, 1000.0);
        let view = Matrix4::look_at(
            Point3::new(12.0, 6.0, 16.0f32),
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        );

        renderer.set_projection(proj);
        renderer.set_view(view);

        let mut imgui = ImGui::init();
        let mut imgui_sdl2 = imgui_sdl2::ImguiSdl2::new(&mut imgui);

        let mut imgui_sdl2 = imgui_sdl2::ImguiSdl2::new(&mut imgui);
        let imgui_renderer = imgui_opengl_renderer::Renderer::new(&mut imgui, |s| {
            video_subsystem.gl_get_proc_address(s) as _
        });

        let mut event_pump = sdl_context.event_pump().unwrap();
        'main: loop {
            for event in event_pump.poll_iter() {
                imgui_sdl2.handle_event(&mut imgui, &event);

                match &event {
                    &Event::Quit { .. } => break 'main,
                    &Event::KeyDown { .. }
                    | Event::KeyUp { .. }
                    | &Event::Window { .. }
                    | Event::MouseButtonDown { .. }
                    | &Event::MouseButtonUp { .. }
                    | Event::MouseMotion { .. }
                    | &Event::MouseWheel { .. } => {
                        if let Some(ref mut h) = self.handler {
                            h.event(&event);
                        }
                    }
                    _ => {}
                }
            }

            if self.simulate {
                let step = (1_000_000_000.0f32 / 60.0) * self.time_scale;
                let step = Duration::new(0, step as u32);
                world.update(step);

                self.elapsed += step;
            }

            unsafe {
                gl::Enable(gl::DEPTH_TEST);
                gl::Disable(gl::STENCIL_TEST);
                gl::Disable(gl::SCISSOR_TEST);
            }

            let stats = self.render(&renderer, &bodies());

            unsafe {
                gl::Disable(gl::DEPTH_TEST);
                gl::Enable(gl::STENCIL_TEST);
                gl::Enable(gl::SCISSOR_TEST);
            }

            let ui = imgui_sdl2.frame(&window, &mut imgui, &event_pump);
            self.do_imgui(&ui, &world, &stats);
            imgui_renderer.render(ui);

            window.gl_swap_window();
        }
    }

    fn do_imgui(&mut self, ui: &imgui::Ui, world: &NewtonWorld, stats: &RenderStats) {
        ui.main_menu_bar(|| {
            ui.menu(im_str!("Simulation")).build(|| {
                ui.menu_item(im_str!("Simulate"))
                    .selected(&mut self.simulate)
                    .build();
            });

            ui.menu(im_str!("Render")).build(|| {
                ui.menu_item(im_str!("Bodies"))
                    .selected(&mut self.bodies)
                    .build();
                ui.menu_item(im_str!("AABB"))
                    .selected(&mut self.aabb)
                    .build();
                ui.menu_item(im_str!("Constraints"))
                    .selected(&mut self.constraints)
                    .build();
                ui.separator();
                ui.menu_item(im_str!("Wireframe"))
                    .selected(&mut self.wireframe)
                    .build();
                ui.menu_item(im_str!("Lighting"))
                    .selected(&mut self.lighting)
                    .build();
                ui.menu_item(im_str!("Stats"))
                    .selected(&mut self.stats)
                    .build();
            });
        });

        if self.stats {
            ui.window(im_str!("Stats"))
                .size((300.0, 100.0), ImGuiCond::FirstUseEver)
                .build(|| {
                    ui.label_text(
                        im_str!("Elapsed"),
                        &ImString::new(format!("{:?}", self.elapsed)),
                    );
                    ui.slider_float(im_str!("Time scale"), &mut self.time_scale, 0.1, 2.0)
                        .build();
                    ui.separator();
                    ui.label_text(
                        im_str!("Bodies"),
                        &ImString::new(format!("{}", world.body_count())),
                    );
                    ui.label_text(im_str!("Constraints"), im_str!("0"));
                    ui.separator();
                    ui.label_text(
                        im_str!("Triangles"),
                        &ImString::new(format!("{}", stats.tris)),
                    );
                    ui.label_text(
                        im_str!("Draw calls"),
                        &ImString::new(format!("{}", stats.drawcalls)),
                    );
                    ui.label_text(
                        im_str!("Frames/second"),
                        &ImString::new(format!("{}", ui.framerate())),
                    );
                })
        }
    }
}

fn get_renderer() -> Renderer {
    Renderer::new()
}
