pub mod math;
mod renderer;

use std::marker::PhantomData;
use std::mem;
use std::rc::Rc;
use std::time::{Duration, Instant};

use newton::collision::NewtonCollision;
use newton::NewtonConfig;
use newton::SleepState;

use cgmath::prelude::*;
use cgmath::Angle;
use cgmath::{perspective, Deg, Matrix4, Point3, Vector3};

use self::renderer::{Mode, Primitive, RenderStats, Renderer};

use imgui::im_str;
use imgui::{ImGui, ImGuiCond, ImStr, ImString, Ui};

/// Reexport of imgui-rs
pub use imgui;

pub use self::renderer::Color;

#[derive(Debug, Clone)]
pub enum SandboxData {}
unsafe impl NewtonConfig for SandboxData {
    const GRAVITY: math::Vector3<f32> = math::Vector3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    };
    type Vector3 = math::Vector3<f32>;
    type Vector4 = math::Vector4<f32>;
    type Matrix4 = math::Matrix4<f32>;
    type Quaternion = math::Quaternion<f32>;
}

pub trait SandboxHandler {
    fn imgui(&mut self, imgui: &Ui) {}
    fn event(&mut self, event: &Event) {}
}

// re-export newton types

pub type NewtonWorld = newton::world::NewtonWorld<SandboxData>;
pub type NewtonBody = newton::body::NewtonBody<SandboxData>;
pub type BoxCollision = newton::collision::CollisionBox<SandboxData>;
pub type SphereCollision = newton::collision::CollisionSphere<SandboxData>;
pub type ConeCollision = newton::collision::CollisionCone<SandboxData>;

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
    handler: Option<Box<SandboxHandler>>,

    // camera movement
    mouse_down: bool,
    radius: f32,
    alpha: Deg<f32>,
    delta: Deg<f32>,
    follow: Option<usize>,

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

    // aabb
    render_render_aabb: bool,
    min: Vector3<f32>,
    max: Vector3<f32>,
}

impl Sandbox {
    pub fn new() -> Sandbox {
        Sandbox {
            handler: None,

            mouse_down: false,
            radius: 24.0,
            alpha: Deg(20.0),
            delta: Deg(30.0),

            background: rgba!(1.0, 1.0, 1.0),

            simulate: false,
            elapsed: Duration::new(0, 0),
            time_scale: 1.0,
            follow: None,

            awake_color: rgba!(1.0, 0.0, 1.0),
            sleep_color: rgba!(1.0, 1.0, 1.0),
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

            render_render_aabb: true,
            min: Vector3::new(-8.0, -8.0, -8.0),
            max: Vector3::new(8.0, 8.0, 8.0),
        }
    }

    pub fn window_size(mut self, width: usize, height: usize) -> Self {
        self.width = width;
        self.height = height;
        self
    }

    /*
    pub fn event_handler(&mut self, handler: E) {
        self.handler = Some(handler);
    }
    */

    pub fn background_color(mut self, background: Color) -> Self {
        self.background = background;
        self
    }

    pub fn render_wireframe(mut self, wireframe: bool) -> Self {
        self.wireframe = wireframe;
        self
    }

    pub fn render_solid(mut self, solid: bool) -> Self {
        self.solid = solid;
        self
    }

    pub fn render_aabb(mut self, aabb: bool) -> Self {
        self.aabb = aabb;
        self
    }

    pub fn simulate(mut self, sim: bool) -> Self {
        self.simulate = sim;
        self
    }

    pub fn aabb(mut self, min: Vector3<f32>, max: Vector3<f32>) -> Self {
        self.min = min;
        self.max = max;
        self
    }

    pub fn render_render_aabb(&self, renderer: &Renderer, stats: &mut RenderStats) {
        let scale = self.max - self.min;
        let position = (self.max + self.min) * 0.5;
        let transform = Matrix4::from_translation(position)
            * Matrix4::from_nonuniform_scale(scale.x, scale.y, scale.z);

        //renderer.set_linewidth(3.0);

        renderer.set_lighting(false);
        renderer.render(
            Primitive::Box,
            Mode::Fill,
            rgba!(1.0),
            transform,
            Some(stats),
        );
        renderer.set_lighting(self.lighting);
    }

    fn render_aabb_(&self, renderer: &Renderer, bodies: &[NewtonBody], stats: &mut RenderStats) {
        for body in bodies.iter() {
            let (min, max) = body.aabb();
            let position = body.position();

            let scale = max - min;
            let transform = Matrix4::from_translation(position)
                * Matrix4::from_nonuniform_scale(scale.x, scale.y, scale.z);

            renderer.render(
                Primitive::Box,
                Mode::Wireframe,
                rgba!(0.0),
                transform,
                Some(stats),
            );
        }
    }

    fn render_bodies(
        &self,
        renderer: &Renderer,
        bodies: &[NewtonBody],
        follow: Option<usize>,
        mode: Mode,
        awake_color: Color,
        sleeping_color: Color,
        stats: &mut RenderStats,
    ) {
        for (i, body) in bodies.iter().enumerate() {
            let mut transform = body.matrix();
            let color = match (body.sleep_state(), follow) {
                (_, Some(n)) if n == i => rgba!(1.0, 1.0, 0.4),
                (SleepState::Sleeping, _) => sleeping_color,
                (SleepState::Awake, _) => awake_color,
            };

            match body.collision() {
                NewtonCollision::Box(b) => {
                    let params = b.params();
                    transform =
                        transform * Matrix4::from_nonuniform_scale(params.x, params.y, params.z);
                    renderer.render(Primitive::Box, mode, color, transform, Some(stats));
                }
                NewtonCollision::Sphere(s) => {
                    let params = s.params();
                    transform = transform * Matrix4::from_scale(params.radius);
                    renderer.render(Primitive::Sphere, mode, color, transform, Some(stats));
                }
                NewtonCollision::Cone(s) => {
                    let params = s.params();
                    transform = transform
                        * Matrix4::from_nonuniform_scale(
                            params.height,
                            params.radius,
                            params.radius,
                        );
                    renderer.render(Primitive::Cone, mode, color, transform, Some(stats));
                }
                _ => unimplemented!(),
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
            _ => {}
        }
    }

    pub fn run<H: SandboxHandler>(mut self, world: NewtonWorld, mut handler: Option<H>) {
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

        renderer.set_projection(proj);

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
                        if let Some(ref mut h) = self.handler {
                            h.event(&event);
                        }
                    }
                    _ => {}
                }
            }

            let bodies: Vec<_> = world.bodies_in_aabb(self.min, self.max);

            if self.simulate {
                let step = (1_000_000_000.0f32 / 60.0) * self.time_scale;
                let step = Duration::new(0, step as u32);
                world.update(step);

                self.elapsed += step;
            }

            let center = self
                .follow
                .and_then(|i| bodies.get(i))
                .map(|b| b.position())
                .map(|p| Point3::new(p.x, p.y, p.z))
                .unwrap_or(Point3::new(0.0, 0.0, 0.0));

            let view = Matrix4::look_at(
                Point3::new(
                    center.x + self.radius * Deg::cos(self.delta) * Deg::sin(self.alpha),
                    center.y + self.radius * Deg::sin(self.delta),
                    center.z + self.radius * Deg::cos(self.delta) * Deg::cos(self.alpha),
                ),
                center,
                Vector3::new(0.0, 1.0, 0.0),
            );

            renderer.reset(self.background);

            unsafe {
                gl::Enable(gl::DEPTH_TEST);
                gl::Disable(gl::STENCIL_TEST);
                gl::Disable(gl::SCISSOR_TEST);
            }

            renderer.set_view(view);

            let mut stats = RenderStats {
                tris: 0,
                drawcalls: 0,
            };

            if self.render_render_aabb {
                unsafe { gl::DepthMask(gl::FALSE) };
                self.render_render_aabb(&renderer, &mut stats);
                unsafe { gl::DepthMask(gl::TRUE) };
            }

            if self.solid {
                renderer.set_lighting(self.lighting);
                self.render_bodies(
                    &renderer,
                    &bodies,
                    self.follow,
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
                    &bodies,
                    None,
                    Mode::Wireframe,
                    rgba!(0.0),
                    rgba!(0.0),
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
            self.set_up_imgui(&ui, &world, &bodies, &stats);

            if let Some(ref mut h) = handler {
                h.imgui(&ui)
            }

            imgui_renderer.render(ui);

            window.gl_swap_window();
        }
    }

    fn set_up_imgui(
        &mut self,
        ui: &imgui::Ui,
        world: &NewtonWorld,
        bodies: &[NewtonBody],
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
                .size((300.0, 100.0), ImGuiCond::FirstUseEver)
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
                        &ImString::new(format!("{}", world.body_count())),
                    );
                    ui.label_text(im_str!("Constraints"), im_str!("0"));
                    ui.separator();

                    let bodies: Vec<_> = bodies
                        .iter()
                        .map(|b| ImString::from(format!("{:?}", b)))
                        .collect();
                    let refs: Vec<_> = bodies.iter().map(|b| ImStr::new(b)).collect();

                    let mut selected = self.follow.as_mut().map(|f| *f).unwrap_or(0) as _;
                    if ui.list_box(im_str!("Bodies"), &mut selected, &refs[..], 16) {
                        if let Some(f) = self.follow.as_mut() {
                            *f = selected as usize;
                        }
                    }

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
