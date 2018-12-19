mod renderer;
pub mod math;

use std::marker::PhantomData;
use std::time::Duration;
use std::rc::Rc;

use newton::body::SleepState;

use cgmath::{Matrix4, perspective, Deg, Point3, Vector3};
use cgmath::prelude::*;

use self::renderer::{Renderer, Primitive, Mode};

use imgui::{ImGui, Ui, ImGuiCond};
use imgui::im_str;

pub use self::renderer::Color;

#[derive(Debug)]
pub enum SandboxData {}
impl newton::NewtonData for SandboxData {
    type Vector3 = math::Vector3<f32>;
    type Vector4 = math::Vector4<f32>;
    type Matrix4 = math::Matrix4<f32>;
    type Quaternion = math::Quaternion<f32>;
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
    ($r:expr, $g:expr, $b:expr, $a:expr) => { $crate::Color { r: $r, g: $g, b: $b, a: $a} };
    ($r:expr, $g:expr, $b:expr) => { rgba!($r, $g, $b, 1.0) };
    ($r:expr) => { rgba!($r, $r, $r) };
}

pub use sdl2::event::Event;
pub use sdl2::keyboard::{Keycode, Scancode, Mod};
pub use sdl2::event::WindowEvent;
pub use sdl2::mouse::MouseButton;

pub struct Sandbox<E> {
    handler: Option<E>,
    background: Color,
    wireframe: bool,
    keyboard: bool,
    aabb: bool,
    awake_color: Color,
    sleep_color: Color,
}

impl<E: EventHandler> Sandbox<E> {
    pub fn new() -> Sandbox<E> {
        Sandbox {
            handler: None,
            background: rgba!(1.0, 1.0, 1.0),
            wireframe: false,
            keyboard: false,
            aabb: true,
            awake_color: rgba!(1.0, 0.0, 1.0),
            sleep_color: rgba!(1.0, 1.0, 1.0),
        }
    }

    pub fn event_handler(&mut self, handler: E) {
        self.handler = Some(handler);
    }

    pub fn background_color(&mut self, background: Color) {
        self.background = background;
    }

    pub fn set_wireframe(&mut self, wireframe: bool) {
        self.wireframe = wireframe;
    }

    pub fn set_aabb(&mut self, aabb: bool) {
        self.aabb = aabb;
    }

    fn render(&self, renderer: &Renderer, bodies: &[NewtonBody]) {
        renderer.reset();

        for body in bodies.iter() {
            let mut transform = Matrix4::identity();
            unsafe {
                // XXX pointers
                std::ptr::copy(&body.matrix() as *const _ as *const f32, &mut transform[0][0] as *mut f32, 16);
            }
            let color = match body.sleep_state() {
                SleepState::Sleeping => self.sleep_color,
                SleepState::Awake => self.awake_color,
            };
            renderer.render(Primitive::Box, Mode::Fill, color, transform);
        }

        if self.aabb {
            // aabb
            // XXX pointers
            for body in bodies.iter() {
                unsafe {
                    let mut aabb: (Vector3<f32>, Vector3<f32>) = std::mem::zeroed();
                    let mut position: Vector3<f32> = std::mem::zeroed();

                    std::ptr::copy(&body.position() as *const _ as *const f32, &mut position as *mut _ as *mut f32, 3);
                    std::ptr::copy(&body.aabb() as *const _ as *const f32, &mut aabb as *mut _ as *mut f32, 6);

                    let scale = aabb.1 - aabb.0;

                    let mut transform = Matrix4::from_translation(position) *
                        Matrix4::from_nonuniform_scale(scale.x, scale.y, scale.z);

                    renderer.render(Primitive::Box, Mode::Wireframe, rgba!(0.0), transform);
                }
            }
        }
    }

    pub fn run<B>(mut self, world: NewtonWorld, bodies: B)
    where
        B: Fn() -> Vec<NewtonBody>,
    {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("Window", 640, 480)
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

        let proj = perspective(Deg(55.0_f32), 4.0/3.0, 0.01, 1000.0);
        let view = Matrix4::look_at(Point3::new(12.0, 6.0, 16.0f32) / 2.0, Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));

        renderer.set_projection(proj);
        renderer.set_view(view);

        let mut imgui = ImGui::init();
        let mut imgui_sdl2 = imgui_sdl2::ImguiSdl2::new(&mut imgui);

        let mut imgui_sdl2 = imgui_sdl2::ImguiSdl2::new(&mut imgui);
        let imgui_renderer = imgui_opengl_renderer::Renderer::new(&mut imgui, |s| video_subsystem.gl_get_proc_address(s) as _);

        let mut event_pump = sdl_context.event_pump().unwrap();
        'main: loop {
            for event in event_pump.poll_iter() {
                imgui_sdl2.handle_event(&mut imgui, &event);

                match &event {
                    &Event::Quit { .. } => break 'main,
                    &Event::KeyDown { .. } | Event::KeyUp { .. } |
                    &Event::Window { .. } | Event::MouseButtonDown { .. } |
                    &Event::MouseButtonUp { .. } | Event::MouseMotion { .. } |
                    &Event::MouseWheel { .. } => {
                        if let Some(ref mut h) = self.handler {
                            h.event(&event);
                        }
                    },
                    _ => {},
                }
            }

            world.update(Duration::new(0, 1_000_000_000 / 60));

            unsafe {
                gl::Enable(gl::DEPTH_TEST);
                gl::Disable(gl::STENCIL_TEST);
                gl::Disable(gl::SCISSOR_TEST);
            }

            self.render(&renderer, &bodies());

            unsafe {
                gl::Disable(gl::DEPTH_TEST);
                gl::Enable(gl::STENCIL_TEST);
                gl::Enable(gl::SCISSOR_TEST);
            }

            let ui = imgui_sdl2.frame(&window, &mut imgui, &event_pump);
            self.do_imgui(&ui);
            imgui_renderer.render(ui);

            window.gl_swap_window();
        }
    }

    fn do_imgui(&self, ui: &imgui::Ui) {
        ui.window(im_str!("Hello world"))
            .size((300.0, 100.0), ImGuiCond::FirstUseEver)
            .build(|| {
                ui.text(im_str!("Hello world!"));
                ui.text(im_str!("こんにちは世界！"));
                ui.text(im_str!("This...is...imgui-rs!"));
                ui.separator();
                let mouse_pos = ui.imgui().mouse_pos();
                ui.text(im_str!("Mouse Position: ({:.1},{:.1})", mouse_pos.0, mouse_pos.1));
            })
    }
}

fn get_renderer() -> Renderer {
    Renderer::new()
}