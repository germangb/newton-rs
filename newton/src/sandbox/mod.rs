use ffi;

mod renderer;

use std::cell::RefCell;
use std::marker::PhantomData;
use std::mem;
use std::rc::Rc;
use std::time::{Duration, Instant};

use cgmath::prelude::*;
use cgmath::Angle;
use cgmath::{perspective, vec3, Deg, Matrix4, Point3, Quaternion, Vector3, Vector4};

use self::renderer::{Mode, Primitive, RenderStats, Renderer};

use imgui::im_str;
use imgui::{ImGui, ImGuiCond, ImStr, ImString, Ui};

pub use self::renderer::Color;
pub use cgmath;
pub use imgui;

#[macro_export]
macro_rules! rgba {
    ($r:expr, $g:expr, $b:expr, $a:expr) => {
        $crate::sandbox::Color {
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
pub use sdl2::keyboard::{Keycode, Scancode};
pub use sdl2::mouse::MouseButton;
use std::collections::HashMap;

//use super::body::{Body, NewtonBody};
//use super::world::{BroadphaseAlgorithm, World};
//use super::{Application, Types};
//use super::body::SleepState;
//use super::collision::CollisionParams;

pub struct Sandbox {
    world: *const ffi::NewtonWorld,

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

    input: Input,
    handler: Option<Box<FnMut(Input)>>,
}

#[derive(Debug, Copy, Clone)]
pub struct Input {
    pub look: Vector3<f32>,
    pub w: bool,
    pub a: bool,
    pub s: bool,
    pub d: bool,
    pub space: bool,
    pub lshift: bool,
}

pub fn run(world: *const ffi::NewtonWorld) {
    Sandbox::default(world).run()
}

impl Sandbox {
    pub fn default(world: *const ffi::NewtonWorld) -> Self {
        let mut app = Sandbox::new(world);
        app.window_size(800, 600);
        app.render_solid(true);
        app.render_wireframe(true);
        app.render_aabb(false);
        app.simulate(false);
        app
    }

    pub fn new(world: *const ffi::NewtonWorld) -> Self {
        Sandbox {
            world,
            mouse_down: false,
            radius: 24.0,
            alpha: Deg(20.0),
            delta: Deg(30.0),

            background: rgba!(1.0, 1.0, 1.0),

            simulate: false,
            elapsed: Duration::new(0, 0),
            time_scale: 1.0,

            awake_color: rgba!(1.0, 0.5, 1.0),
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

            input: Input {
                space: false,
                lshift: false,
                look: Vector3::new(0.0, 0.0, 0.0),
                w: false,
                a: false,
                s: false,
                d: false,
            },
            handler: None,
        }
    }

    pub fn set_handler<C: FnMut(Input) + 'static>(&mut self, handle: C) {
        self.handler = Some(Box::new(handle))
    }

    /*
    pub fn world(&self) -> &World<Self> {
        &self.world
    }
    */

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
        bodies: &[*mut ffi::NewtonBody],
        stats: &mut RenderStats,
    ) {
        for body in bodies.iter() {
            let mut position = Vector3::new(0.0, 0.0, 0.0);
            let (mut min, mut max) = (position, position);

            unsafe {
                ffi::NewtonBodyGetPosition(*body, position.as_mut_ptr());
                ffi::NewtonBodyGetAABB(*body, min.as_mut_ptr(), max.as_mut_ptr());
            }

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
            (
                _,
                Event::KeyDown {
                    keycode: Some(key), ..
                },
            ) => match key {
                Keycode::W => self.input.w = true,
                Keycode::A => self.input.a = true,
                Keycode::S => self.input.s = true,
                Keycode::D => self.input.d = true,
                Keycode::Space => self.input.space = true,
                Keycode::LShift => self.input.lshift = true,
                _ => {}
            },
            (
                _,
                Event::KeyUp {
                    keycode: Some(key), ..
                },
            ) => match key {
                Keycode::W => self.input.w = false,
                Keycode::A => self.input.a = false,
                Keycode::S => self.input.s = false,
                Keycode::D => self.input.d = false,
                Keycode::Space => self.input.space = false,
                Keycode::LShift => self.input.lshift = false,
                _ => {}
            },
            _ => {}
        }
    }

    fn render_bodies(
        &self,
        renderer: &Renderer,
        bodies: &[*mut ffi::NewtonBody],
        mode: Mode,
        awake_color: Color,
        sleeping_color: Color,
        stats: &mut RenderStats,
    ) {
        let mut collision_info: ffi::NewtonCollisionInfoRecord = unsafe { mem::zeroed() };

        for body in bodies {
            let mut transform = Matrix4::identity();

            unsafe {
                ffi::NewtonBodyGetMatrix(*body, transform.as_mut_ptr());
            }

            let color = match unsafe { ffi::NewtonBodyGetSleepState(*body) } {
                // sleeping
                1 => sleeping_color,

                // awake
                0 => awake_color,
                _ => unreachable!(),
            };

            unsafe {
                let collision = ffi::NewtonBodyGetCollision(*body);
                ffi::NewtonCollisionGetInfo(collision, &mut collision_info);

                let mut offset = Matrix4::identity();
                ffi::NewtonCollisionGetMatrix(collision, offset.as_mut_ptr());
                transform = transform * offset;
            }

            match collision_info.m_collisionType as _ {
                ffi::SERIALIZE_ID_BOX => {
                    let p = unsafe { collision_info.__bindgen_anon_1.m_box };
                    transform = transform * Matrix4::from_nonuniform_scale(p.m_x, p.m_y, p.m_z);
                    renderer.render(Primitive::Box, mode, color, transform, Some(stats));
                }
                ffi::SERIALIZE_ID_SPHERE => {
                    let p = unsafe { collision_info.__bindgen_anon_1.m_sphere };
                    transform = transform * Matrix4::from_scale(p.m_radio);
                    renderer.render(Primitive::Sphere, mode, color, transform, Some(stats));
                }
                ffi::SERIALIZE_ID_CAPSULE => {
                    let p = unsafe { collision_info.__bindgen_anon_1.m_capsule };
                    let t = Vector3::new(p.m_height / 2.0, 0.0, 0.0);
                    renderer.render(
                        Primitive::Sphere,
                        mode,
                        color,
                        transform * Matrix4::from_translation(t) * Matrix4::from_scale(p.m_radio0),
                        Some(stats),
                    );
                    renderer.render(
                        Primitive::Sphere,
                        mode,
                        color,
                        transform * Matrix4::from_translation(-t) * Matrix4::from_scale(p.m_radio1),
                        Some(stats),
                    );
                    renderer.render(
                        Primitive::Cylinder,
                        mode,
                        color,
                        transform
                            * Matrix4::from_nonuniform_scale(p.m_height, p.m_radio0, p.m_radio1),
                        Some(stats),
                    );
                }
                ffi::SERIALIZE_ID_CYLINDER => {
                    let p = unsafe { collision_info.__bindgen_anon_1.m_cylinder };
                    let t = Vector3::new(p.m_height / 2.0, 0.0, 0.0);
                    renderer.render(
                        Primitive::Cylinder,
                        mode,
                        color,
                        transform
                            * Matrix4::from_nonuniform_scale(p.m_height, p.m_radio0, p.m_radio1),
                        Some(stats),
                    );
                }
                id @ _ => {
                    eprintln!("unimplemented shape. ID = {}", id);
                }
            }
        }
    }
    pub fn run(mut self) {
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

        let renderer = Renderer::new();

        let mut imgui = ImGui::init();
        let mut imgui_sdl2 = imgui_sdl2::ImguiSdl2::new(&mut imgui);

        let mut imgui_sdl2 = imgui_sdl2::ImguiSdl2::new(&mut imgui);
        let imgui_renderer = imgui_opengl_renderer::Renderer::new(&mut imgui, |s| {
            video_subsystem.gl_get_proc_address(s) as _
        });

        /*
        unsafe {
            ffi::NewtonSetThreadsCount(self.world, 1);
        }
        */

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
            }

            let aspect = (self.width as f32) / (self.height as f32);
            let projection = perspective(Deg(55.0_f32), aspect, 0.01, 1000.0);

            let view = Matrix4::look_at(
                Point3::new(
                    self.radius * Deg::cos(self.delta) * Deg::sin(self.alpha),
                    self.radius * Deg::sin(self.delta),
                    self.radius * Deg::cos(self.delta) * Deg::cos(self.alpha),
                ),
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
            );

            if let &mut Some(ref mut h) = &mut self.handler {
                let mut look = Vector3::new(0.0, 0.0, 1.0);
                self.input.look = view.invert().unwrap().transform_vector(look);
                h(self.input)
            }

            let mut bodies = Vec::new();
            unsafe {
                let mut b = ffi::NewtonWorldGetFirstBody(self.world);
                while !b.is_null() {
                    bodies.push(b);
                    b = ffi::NewtonWorldGetNextBody(self.world, b);
                }
            }

            if self.simulate {
                let step = (1_000_000_000.0f32 / 60.0) * self.time_scale;
                let step_dur = Duration::new(0, step as u32);

                unsafe {
                    ffi::NewtonUpdate(self.world, step / 1_000_000_000.0);
                    //ffi::NewtonUpdateAsync(self.world, step / 1_000_000_000.0);
                    //ffi::NewtonWaitForUpdateToFinish(self.world);
                }

                self.elapsed += step_dur;
            }

            renderer.set_projection(projection);
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
                    &bodies[..],
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
                    &bodies[..],
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
            self.set_up_imgui(&ui, &stats);
            imgui_renderer.render(ui);

            window.gl_swap_window();
        }
    }

    fn set_up_imgui(&mut self, ui: &imgui::Ui, stats: &RenderStats) {
        ui.main_menu_bar(|| {
            ui.menu(im_str!("Simulation")).build(|| {
                ui.checkbox(im_str!("Simulate"), &mut self.simulate);
                if ui.button(im_str!("Invalidate"), (100.0, 24.0)) {
                    unsafe {
                        ffi::NewtonInvalidateCache(self.world);
                    }
                }
                ui.separator();
                ui.radio_button(im_str!("1 thread"), &mut 0, 0);
                ui.radio_button(im_str!("2 thread"), &mut 0, 1);
                ui.radio_button(im_str!("3 thread"), &mut 0, 1);
                ui.radio_button(im_str!("4 thread"), &mut 0, 1);
            });

            ui.menu(im_str!("Render")).build(|| {
                ui.checkbox(im_str!("Bodies"), &mut self.bodies);
                ui.checkbox(im_str!("AABB"), &mut self.aabb);
                ui.checkbox(im_str!("Constraints"), &mut self.constraints);
                ui.separator();
                ui.checkbox(im_str!("Solid"), &mut self.solid);
                ui.checkbox(im_str!("Wireframe"), &mut self.wireframe);
                ui.checkbox(im_str!("Lighting"), &mut self.lighting);
                ui.separator();
                ui.checkbox(im_str!("Stats"), &mut self.stats);
            });
        });

        if self.stats {
            ui.window(im_str!("Stats")).build(|| {
                ui.label_text(
                    im_str!("Elapsed"),
                    &ImString::new(format!("{:?}", self.elapsed)),
                );
                ui.slider_float(im_str!("Time scale"), &mut self.time_scale, 0.1, 2.0)
                    .build();
                ui.checkbox(im_str!("Simulate"), &mut self.simulate);
                ui.separator();
                ui.label_text(
                    im_str!("Threads"),
                    &ImString::new(format!("{}", unsafe {
                        ffi::NewtonGetThreadsCount(self.world)
                    })),
                );
                ui.label_text(
                    im_str!("Bodies"),
                    &ImString::new(format!("{}", unsafe {
                        ffi::NewtonWorldGetBodyCount(self.world)
                    })),
                );
                ui.label_text(
                    im_str!("Constraints"),
                    &ImString::new(format!("{}", unsafe {
                        ffi::NewtonWorldGetConstraintCount(self.world)
                    })),
                );
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
                    &ImString::new(format!("{}", (ui.framerate() + 0.5) as u32)),
                );
            })
        }
    }
}
