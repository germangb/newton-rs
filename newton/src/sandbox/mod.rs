mod renderer;

use std::cell::RefCell;
use std::marker::PhantomData;
use std::mem;
use std::rc::Rc;
use std::time::{Duration, Instant};

use crate::collision::Collision;
use crate::NewtonApp;
use crate::SleepState;

/*
use crate::Body;
use crate::World;
*/

use cgmath::prelude::*;
use cgmath::Angle;
use cgmath::{perspective, Deg, Matrix4, Point3, Quaternion, Vector3};

use self::renderer::{Mode, Primitive, RenderStats, Renderer};

use imgui::im_str;
use imgui::{ImGui, ImGuiCond, ImStr, ImString, Ui};

pub use cgmath;
pub use imgui;
//pub use sdl2::event;

pub use self::renderer::Color;

#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct SandboxApp;
unsafe impl NewtonApp for SandboxApp {
    type Vector = Vector3<f32>;
    type Matrix = Matrix4<f32>;
    type Quaternion = Quaternion<f32>;
}

pub type World = crate::World<SandboxApp>;
pub type DynamicBody = crate::DynamicBody<SandboxApp>;

pub type BoxCollision = crate::BoxCollision<SandboxApp>;
pub type SphereCollision = crate::SphereCollision<SandboxApp>;
pub type CapsuleCollision = crate::CapsuleCollision<SandboxApp>;
pub type CylinderCollision = crate::CylinderCollision<SandboxApp>;
pub type ConeCollision = crate::ConeCollision<SandboxApp>;

pub type BallJoint = crate::BallJoint<SandboxApp>;
pub type SliderJoint = crate::SliderJoint<SandboxApp>;
pub type HingeJoint = crate::HingeJoint<SandboxApp>;
pub type UniversalJoint = crate::UniversalJoint<SandboxApp>;
pub type CorkscrewJoint = crate::CorkscrewJoint<SandboxApp>;
pub type UpVectorJoint = crate::UpVectorJoint<SandboxApp>;

pub trait Handler {
    fn pre_update(&mut self) {}
    fn post_update(&mut self) {}
    fn event(&mut self, event: &Event) {}
}

#[macro_export]
macro_rules! color {
    ($r:expr, $g:expr, $b:expr, $a:expr) => {
        $crate::sandbox::Color {
            r: $r,
            g: $g,
            b: $b,
            a: $a,
        }
    };
    ($r:expr, $g:expr, $b:expr) => {
        color!($r, $g, $b, 1.0)
    };
    ($r:expr) => {
        color!($r, $r, $r)
    };
}

pub use sdl2::event::Event;
pub use sdl2::event::WindowEvent;
pub use sdl2::keyboard::{Keycode, Scancode};
pub use sdl2::mouse::MouseButton;
use std::collections::HashMap;

pub struct Sandbox {
    //handler: Box<Handler>,
    world: crate::World<SandboxApp>,

    // camera movement
    mouse_down: bool,
    radius: f32,
    alpha: Deg<f32>,
    delta: Deg<f32>,

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

    solid: bool,
    wireframe: bool,

    aabb: bool,
    constraints: bool,
    bodies: bool,
    stats: bool,
}

impl Sandbox {
    pub fn new() -> Self {
        Sandbox {
            //handler: Box::new(handler),
            world: crate::World::new(crate::BroadPhaseAlgorithm::Default, SandboxApp),

            mouse_down: false,
            radius: 24.0,
            alpha: Deg(20.0),
            delta: Deg(30.0),

            background: color!(1.0, 1.0, 1.0),

            simulate: false,
            elapsed: Duration::new(0, 0),
            time_scale: 1.0,

            awake_color: color!(1.0, 0.5, 1.0),
            sleep_color: color!(1.0, 1.0, 1.0),
            lighting: true,

            width: 800,
            height: 600,

            solid: true,
            wireframe: true,
            keyboard: false,
            aabb: false,
            constraints: false,
            bodies: true,

            stats: true,
        }
    }

    pub fn world(&self) -> &crate::World<SandboxApp> {
        &self.world
    }

    pub fn window_size(&mut self, width: usize, height: usize) -> &mut Self {
        self.width = width;
        self.height = height;
        self
    }

    /*
    pub fn event_handler(&mut self, handler: E) {
        self.handler = Some(handler);
    }
    */

    pub fn background_color(&mut self, background: Color) -> &mut Self {
        self.background = background;
        self
    }

    pub fn render_wireframe(&mut self, wireframe: bool) -> &mut Self {
        self.wireframe = wireframe;
        self
    }

    pub fn render_solid(&mut self, solid: bool) -> &mut Self {
        self.solid = solid;
        self
    }

    pub fn render_aabb(&mut self, aabb: bool) -> &mut Self {
        self.aabb = aabb;
        self
    }

    pub fn simulate(&mut self, sim: bool) -> &mut Self {
        self.simulate = sim;
        self
    }

    fn render_aabb_(
        &self,
        renderer: &Renderer,
        bodies: &[crate::DynamicBody<SandboxApp>],
        stats: &mut RenderStats,
    ) {
        for body in bodies.iter() {
            let (min, max) = body.aabb();
            let position = body.position();

            let scale = max - min;
            let transform = Matrix4::from_translation(position)
                * Matrix4::from_nonuniform_scale(scale.x, scale.y, scale.z);

            renderer.render(
                Primitive::Box,
                Mode::Wireframe,
                color!(0.0),
                transform,
                Some(stats),
            );
        }
    }

    fn render_bodies(
        &self,
        renderer: &Renderer,
        bodies: &HashMap<*const (), crate::DynamicBody<SandboxApp>>,
        mode: Mode,
        awake_color: Color,
        sleeping_color: Color,
        stats: &mut RenderStats,
    ) {
        for (i, body) in bodies.values().enumerate() {
            let mut transform = body.matrix();
            let color = match body.sleep_state() {
                SleepState::Sleeping => sleeping_color,
                SleepState::Awake => awake_color,
            };

            match body.collision() {
                Collision::Box(b) => {
                    let params = b.params();
                    transform =
                        transform * Matrix4::from_nonuniform_scale(params.x, params.y, params.z);
                    renderer.render(Primitive::Box, mode, color, transform, Some(stats));
                }
                Collision::Sphere(s) => {
                    let params = s.params();
                    transform = transform * Matrix4::from_scale(params.radius);
                    renderer.render(Primitive::Sphere, mode, color, transform, Some(stats));
                }
                Collision::Cone(s) => {
                    let params = s.params();
                    transform = transform
                        * Matrix4::from_nonuniform_scale(
                            params.height,
                            params.radius,
                            params.radius,
                        );
                    renderer.render(Primitive::Cone, mode, color, transform, Some(stats));
                }
                Collision::Cylinder(s) => {
                    let params = s.params();
                    transform = transform
                        * Matrix4::from_nonuniform_scale(
                            params.height,
                            params.radius0,
                            params.radius0,
                        );
                    renderer.render(Primitive::Cylinder, mode, color, transform, Some(stats));
                }
                Collision::Capsule(s) => {
                    let params = s.params();
                    transform = transform
                        * Matrix4::from_nonuniform_scale(
                            params.height,
                            params.radius0,
                            params.radius0,
                        );
                    renderer.render(Primitive::Cylinder, mode, color, transform, Some(stats));
                    renderer.render(
                        Primitive::Sphere,
                        mode,
                        color,
                        transform
                            * Matrix4::from_translation(Vector3::new(
                                s.params().height / 2.0,
                                0.0,
                                0.0,
                            )),
                        Some(stats),
                    );
                    renderer.render(
                        Primitive::Sphere,
                        mode,
                        color,
                        transform
                            * Matrix4::from_translation(Vector3::new(
                                -s.params().height / 2.0,
                                0.0,
                                0.0,
                            )),
                        Some(stats),
                    );
                }
                _ => {
                    eprintln!("unimplemented shape");
                }
            }
        }
    }

    pub fn handle_event(&mut self, event: &Event) {
        match (self.mouse_down, event) {
            (_, Event::MouseWheel { y, .. }) => {
                self.radius -= (*y as f32) * self.radius * 0.2;
            }
            (_, Event::MouseButtonDown { .. }) => self.mouse_down = true,
            (_, Event::MouseButtonUp { .. }) => self.mouse_down = false,
            (true, Event::MouseMotion { xrel, yrel, .. }) => {
                self.alpha -= Deg((*xrel as f32) * 0.5);
                self.delta += Deg((*yrel as f32) * 0.5);
            }
            (
                _,
                Event::Window {
                    win_event: sdl2::event::WindowEvent::Resized(w, h),
                    ..
                },
            ) => {
                self.width = *w as _;
                self.height = *h as _;
            }
            (_, Event::KeyDown { .. }) => {
                //self.handler.event(&self, event);
            }
            _ => {}
        }
    }

    pub fn run<H: Handler>(mut self, bodies: Vec<crate::DynamicBody<SandboxApp>>, mut handler: H) {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("Window", self.width as _, self.height as _)
            .opengl()
            .resizable()
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
                        self.handle_event(&event);
                    }
                    _ => {}
                }

                handler.event(&event);
            }

            let bodies_map: HashMap<_, _> = bodies
                .iter()
                .map(|b| (b.as_raw() as *const (), b.clone()))
                .collect();

            if self.simulate {
                let step = (1_000_000_000.0f32 / 60.0) * self.time_scale;
                let step = Duration::new(0, step as u32);

                handler.pre_update();
                self.world.update(step);
                self.elapsed += step;
                handler.post_update();
            }

            let aspect = (self.width as f32) / (self.height as f32);
            let proj = perspective(Deg(55.0_f32), aspect, 0.01, 1000.0);
            renderer.set_projection(proj);

            let view = Matrix4::look_at(
                Point3::new(
                    self.radius * Deg::cos(self.delta) * Deg::sin(self.alpha),
                    self.radius * Deg::sin(self.delta),
                    self.radius * Deg::cos(self.delta) * Deg::cos(self.alpha),
                ),
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
            );
            renderer.set_view(view);

            unsafe {
                gl::Viewport(0, 0, self.width as _, self.height as _);
                gl::Enable(gl::DEPTH_TEST);
                gl::Disable(gl::STENCIL_TEST);
                gl::Disable(gl::SCISSOR_TEST);
            }

            renderer.reset(self.background);

            let mut stats = RenderStats {
                tris: 0,
                drawcalls: 0,
            };

            if self.solid {
                renderer.set_lighting(self.lighting);
                self.render_bodies(
                    &renderer,
                    &bodies_map,
                    Mode::Fill,
                    self.awake_color,
                    self.sleep_color,
                    &mut stats,
                );
            }
            if self.wireframe {
                renderer.set_lighting(false);
                renderer.set_linewidth(3.0);
                self.render_bodies(
                    &renderer,
                    &bodies_map,
                    Mode::Wireframe,
                    color!(0.0),
                    color!(0.0),
                    &mut stats,
                );
            }
            if self.aabb {
                renderer.set_lighting(false);
                renderer.set_linewidth(1.0);
                self.render_aabb_(&renderer, &bodies, &mut stats)
            }

            unsafe {
                gl::Disable(gl::DEPTH_TEST);
                gl::Enable(gl::STENCIL_TEST);
                gl::Enable(gl::SCISSOR_TEST);
            }

            let ui = imgui_sdl2.frame(&window, &mut imgui, &event_pump);
            self.set_up_imgui(&ui, &bodies, &stats);
            imgui_renderer.render(ui);

            window.gl_swap_window();
        }
    }

    fn set_up_imgui(
        &mut self,
        ui: &imgui::Ui,
        bodies: &Vec<crate::DynamicBody<SandboxApp>>,
        stats: &RenderStats,
    ) {
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
                ui.menu_item(im_str!("Solid"))
                    .selected(&mut self.solid)
                    .build();
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
                //.resizable(false)
                //.position((0.0, 0.0), ImGuiCond::Always)
                .always_auto_resize(true)
                //.size((300.0, 200.0), ImGuiCond::FirstUseEver)
                .build(|| {
                    ui.label_text(
                        im_str!("Elapsed"),
                        &ImString::new(format!("{:?}", self.elapsed)),
                    );
                    ui.slider_float(im_str!("Time scale"), &mut self.time_scale, 0.1, 2.0)
                        .build();
                    ui.checkbox(im_str!("Simulate"), &mut self.simulate);
                    ui.separator();
                    ui.label_text(
                        im_str!("Bodies"),
                        &ImString::new(format!("{}", self.world.body_count())),
                    );
                    ui.label_text(im_str!("Constraints"), im_str!("0"));
                    ui.separator();

                    let pointer_strings: Vec<_> = bodies
                        .iter()
                        .map(|p| ImString::from(format!("{:?}", p.as_raw())))
                        .collect();
                    let pointer_ref: Vec<_> =
                        pointer_strings.iter().map(|b| ImStr::new(b)).collect();

                    ui.list_box(im_str!("Bodies"), &mut 0, &pointer_ref[..], 8);
                    ui.list_box(im_str!("Joints"), &mut 0, &pointer_ref[..], 8);

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
