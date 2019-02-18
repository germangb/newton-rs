use std::{mem, str};
use std::error::Error;
use std::time::{Duration, Instant};

use cgmath::{Deg, Matrix3, Matrix4, Point3, Vector3, Vector4};
use cgmath::prelude::*;

//use gl;
use gl::types::*;

//use imgui;
use imgui::{im_str, ImGuiCond, ImGuiInputTextFlags, StyleVar};
use imgui_ext::prelude::*;
use imgui_opengl_renderer::Renderer as ImguiRenderer;
use imgui_sdl2::ImguiSdl2;

use sdl2::{EventPump, Sdl, VideoSubsystem};
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::Keycode;
use sdl2::mouse::MouseButton;
use sdl2::video::{GLContext, Window};

use renderer::{compute_ray, compute_view_proj, Renderer, RenderParams, Vert, vert};

use crate::{Body, Collision};
use crate::body::SleepState;
use crate::ffi;
use crate::math::Vector;
use crate::world::Newton;

mod renderer;

pub trait Demo {
    fn reset(newton: &Newton) -> Self;
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum Sidebar {
    Open,
    Disabled,
}

impl Sidebar {
    pub fn next(&mut self) {
        match self {
            Sidebar::Open => *self = Sidebar::Disabled,
            Sidebar::Disabled => *self = Sidebar::Open,
        }
    }
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

#[derive(ImGuiExt, Clone)]
pub struct SelectedBody {
    ptr: *const ffi::NewtonBody,
    #[imgui(display(display = "{:?}", 0))]
    name: (Option<&'static str>, ),
    #[imgui(drag(speed = 0.1))]
    position: [f32; 3],
    #[imgui(input(flags = "disabled_text"))]
    velocity: [f32; 3],
}

unsafe impl Send for SelectedBody {}
unsafe impl Sync for SelectedBody {}

const fn disabled_text() -> ImGuiInputTextFlags {
    ImGuiInputTextFlags::ReadOnly
}

impl Default for SelectedBody {
    fn default() -> Self {
        Self {
            ptr: 0 as _,
            name: (None, ),
            position: Default::default(),
            velocity: Default::default(),
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

    // Menu opened flag
    sidebar: (Sidebar, usize),

    #[imgui(nested)]
    controls: Stats,
    #[imgui(separator, nested)]
    renderer: Renderer,
}

impl<T: Demo + Send + Sync> Testbed<T> {
    pub fn run() -> Result<(), Box<dyn Error>> {
        let sdl = sdl2::init()?;
        let sdl_events = sdl.event_pump()?;
        let sdl_video = sdl.video()?;
        let sdl_window = sdl_video
            .window("Newton testbed", 1024, 600)
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
            sidebar: (Sidebar::Open, 256),
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
        let mut drag = 0.0;

        'mainLoop: loop {
            // viewport
            let (w, h) = self.sdl_window.size();
            let mut viewport = [0, 0, w, h];
            if self.sidebar.0 == Sidebar::Open { viewport[2] -= self.sidebar.1 as u32; }

            // mouse button states
            // get values here because we need to borrow event pump
            // mutably in the following loop:
            let left = self.sdl_events.mouse_state().left();
            let right = self.sdl_events.mouse_state().right();
            let mut body_popup = false;

            for event in self.sdl_events.poll_iter() {
                imgui_sdl.handle_event(&mut imgui, &event);
                if !imgui_sdl.ignore_event(&event) {

                    use sdl2::mouse::MouseButton::*;
                    match (self.renderer.params().camera.controller, event) {
                        (_, Event::Window { win_event: WindowEvent::Close, .. }) => break 'mainLoop,
                        (_, Event::KeyDown { keycode: Some(Keycode::T), .. }) => self.sidebar.0.next(),
                        (_, Event::MouseButtonDown { .. }) => drag = 0.0,
                        (_, Event::MouseButtonUp { mouse_btn, x, y, .. })
                            if drag < 16.0 && (mouse_btn == Left || mouse_btn == Right)
                        => {

                            let (_, h) = self.sdl_window.size();
                            let camera = &self.renderer.params().camera;
                            let (start, end) = compute_ray(&camera, x, h as i32 - y, viewport);

                            let mut min = 42.0; // some value > 1.0
                            let mut selected = None;

                            self.newton.ray_cast(&start, &end, |body, _, _, _, hit| {
                                if hit < min {
                                    selected = Some(body.as_ptr());
                                    min = hit;
                                }
                                hit
                            }, None, 0);
                            self.selected = selected.map(|ptr| SelectedBody { ptr, ..Default::default() });
                            body_popup = mouse_btn == Right && self.selected.is_some();
                        }
                        (true, Event::MouseMotion { xrel, yrel, .. }) => {
                            drag += (xrel as f32).abs() + (yrel as f32).abs();
                            let camera = &mut self.renderer.params_mut().camera;
                            if right {
                                camera.pan(xrel as f32, yrel as f32)
                            }
                            if left {
                                camera.orbit(xrel as f32, yrel as f32)
                            }
                        }
                        (true, Event::MouseWheel { y, .. }) => {
                            let camera = &mut self.renderer.params_mut().camera;
                            camera.scroll(y as f32);
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
            let mut frame = self.renderer.frame(viewport);

            if params.solid {
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

                    collision.polygons(&matrix, |face, _face_id| {
                        let mut pos = face.chunks(3).map(|s| [s[0], s[1], s[2]]);

                        // Render as a triangle fan
                        let f = pos.next().expect("First vertex");
                        let mut a = pos.next();

                        while let (Some(u), Some(v)) = (a, pos.next()) {
                            frame.triangle(vert(f, color), vert(u, color), vert(v, color));
                            a = Some(v);
                        }
                    });
                }
            }

            if params.wire {
                for body in self.newton.bodies().iter() {
                    let collision = body.collision();
                    let matrix = body.matrix();
                    let color = match self.selected {
                        Some(SelectedBody { ptr, .. }) if ptr == body.as_ptr() => {
                            [255,255,255]
                        },
                        _ => {
                            let [r, g, b, _] = params.wireframe;
                            [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8]
                        },
                    };

                    collision.polygons(&matrix, |face, _face_id| {
                        let mut pos = face.chunks(3).map(|s| [s[0], s[1], s[2]]);

                        // Render as a triangle fan
                        let f = pos.next().expect("First vertex");
                        let mut a = pos.next();

                        while let (Some(u), Some(v)) = (a, pos.next()) {
                            frame.line(vert(u, color), vert(v, color));
                            a = Some(v);
                        }

                        frame.line(vert(f, color), vert(a.unwrap(), color));
                    });
                }
            }

            if params.aabb {
                for body in self.newton.bodies().iter() {
                    let Vector { x, y, z, w: _ } = body.matrix().c3;
                    let (min, max) = body.aabb();
                    let [r, g, b, _] = params.aabb_color;
                    let color = [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8];
                    let edge = [0.0,0.0];
                    frame.line(vert([min.x, min.y, min.z], color), vert([max.x, min.y, min.z], color));
                    frame.line(vert([min.x, min.y, min.z], color), vert([min.x, max.y, min.z], color));
                    frame.line(vert([min.x, min.y, min.z], color), vert([min.x, min.y, max.z], color));
                    frame.line(vert([max.x, max.y, max.z], color), vert([min.x, max.y, max.z], color));
                    frame.line(vert([max.x, max.y, max.z], color), vert([max.x, min.y, max.z], color));
                    frame.line(vert([max.x, max.y, max.z], color), vert([max.x, max.y, min.z], color));
                    frame.line(vert([min.x, max.y, min.z], color), vert([max.x, max.y, min.z], color));
                    frame.line(vert([min.x, max.y, min.z], color), vert([min.x, max.y, max.z], color));
                    frame.line(vert([max.x, min.y, max.z], color), vert([min.x, min.y, max.z], color));
                    frame.line(vert([max.x, min.y, max.z], color), vert([max.x, min.y, min.z], color));
                    frame.line(vert([max.x, min.y, min.z], color), vert([max.x, max.y, min.z], color));
                    frame.line(vert([min.x, min.y, max.z], color), vert([min.x, max.y, max.z], color));
                }
            }

            if params.axis {
                let edge = [0.0,0.0];
                frame.line(vert([0.0,0.0,0.0], [255,0,0]), vert([1024.0,0.0,0.0],[255,0,0]));
                frame.line(vert([0.0,0.0,0.0], [0,255,0]), vert([0.0,1024.0,0.0],[0,255,0]));
                frame.line(vert([0.0,0.0,0.0], [0,0,255]), vert([0.0,0.0,1024.0],[0,0,255]));
            }

            if params.individual_axis {
                for body in self.newton.bodies().iter() {
                    let matrix = body.matrix();
                    let p = [matrix.c3.x, matrix.c3.y, matrix.c3.z];
                    let x = [p[0] + matrix.c0.x, p[1] + matrix.c0.y, p[2] + matrix.c0.z];
                    let y = [p[0] + matrix.c1.x, p[1] + matrix.c1.y, p[2] + matrix.c1.z];
                    let z = [p[0] + matrix.c2.x, p[1] + matrix.c2.y, p[2] + matrix.c2.z];
                    frame.line(vert(p, [255,0,0]), vert(x, [255,0,0]));
                    frame.line(vert(p, [0,255,0]), vert(y, [0,255,0]));
                    frame.line(vert(p, [0,0,255]), vert(z, [0,0,255]));
                }
            }

            frame.render();

            let ui = imgui_sdl.frame(&self.sdl_window, &mut imgui, &self.sdl_events.mouse_state());

            if params.names || params.origins {
                let style = &[
                    StyleVar::WindowBorderSize(0.0),
                    StyleVar::Alpha(0.5),
                ];
                let colors = &[
                    (imgui::ImGuiCol::WindowBg, (0.0, 0.0, 0.0, 0.0)),
                    (imgui::ImGuiCol::Text, (1.0, 1.0, 1.0, 1.0)),
                ];

                let view_proj = compute_view_proj(&params.camera, viewport);

                for (i, body) in self.newton.bodies().iter().enumerate() {
                    let name = if let Some(name) = body.name() {
                        name
                    } else {
                        continue
                    };
                    let model = unsafe { mem::transmute::<_, Matrix4<f32>>(body.matrix()) };
                    let mut screen = view_proj * model * Vector4::new(0.0, 0.0, 0.0, 1.0);
                    screen /= screen.w;
                    screen.x = (screen.x * 0.5 + 0.5) * viewport[2] as f32 + viewport[0] as f32;
                    screen.y = (-screen.y * 0.5 + 0.5) * viewport[3] as f32 + viewport[1] as f32;

                    ui.with_style_and_color_vars(style, colors, || {
                        let id = imgui::ImString::from(format!("##body{}", i));
                        ui.window(&id)
                            .inputs(false)
                            .position((screen.x, screen.y), ImGuiCond::Always)
                            .always_auto_resize(true)
                            .title_bar(false)
                            .resizable(false)
                            .movable(false)
                            .build(|| {
                                let draw = ui.get_window_draw_list();
                                draw.with_clip_rect((screen.x - 128.0, screen.y - 128.0), (screen.x + 128.0, screen.y + 128.0), || {

                                    if params.origins {
                                        draw.add_circle((screen.x, screen.y), 4.0, (0.0, 0.0, 0.0, 1.0))
                                            .filled(true)
                                            .thickness(1.0)
                                            .build();

                                        draw.add_circle((screen.x, screen.y), 3.0, (0.95, 0.64, 0.0, 1.0))
                                            .filled(true)
                                            .thickness(1.0)
                                            .build();
                                    }

                                    if params.names {
                                        let imgui::ImVec2 { x, y } = ui.calc_text_size(im_str!("{}", name), true, 1000.0);
                                        draw.add_rect((screen.x + 2.0, screen.y + 2.0), (screen.x+x+6.0, screen.y+y+6.0), (0.0, 0.0, 0.0, 0.75))
                                            .rounding(2.0)
                                            .filled(true)
                                            .build();

                                        draw.add_text((screen.x + 4.0, screen.y + 4.0), (1.0, 1.0, 1.0, 1.0), name);
                                    }
                                });
                                //ui.text(name)
                            });
                    });
                }
            }

            let (w, h) = self.sdl_window.size();
            let width = self.sidebar.1 as f32;
            let margin = 8.0;

            if self.sidebar.0 != Sidebar::Disabled {
                let style = &[
                    StyleVar::WindowRounding(0.0),
                    StyleVar::WindowBorderSize(0.0),
                ];

                ui.with_style_vars(style, || {
                    ui.window(im_str!("Controls"))
                        .title_bar(false)
                        .movable(false)
                        .position((w as f32 - width, 0.0), ImGuiCond::Always)
                        .size((width, h as f32), ImGuiCond::Always)
                        .resizable(false)
                        .build(|| {
                            ui.text(im_str!("Press [T] to open/show this sidebar."));
                            ui.new_line();
                            ui.separator();
                            ui.new_line();
                            let events = ui.imgui_ext(&mut self);
                            if events.controls().invalidate() {
                                self.newton.invalidate();
                            }
                            if events.controls().reset() {
                                self.controls.elapsed = (Duration::default(), );
                                let newton = Newton::create();
                                let demo = T::reset(&newton);

                                self.selected = None;
                                self.newton = newton;
                                self.demo = demo;
                            }
                            if events.renderer().params().reset() {
                                *self.renderer.params_mut() = RenderParams::default();
                            }
                            if events.renderer().params().camera().reset() {
                                let camera = &mut self.renderer.params_mut().camera;
                                let cont = camera.controller;
                                *camera = Default::default();
                                camera.controller = cont;
                            }
                        });
                });
            }
            if let Some(mut sel) = self.selected.take() {
                let mut drop_body = false;
                let mut focus_cam = false;

                let ptr = sel.ptr;
                let body = unsafe { Body::from_raw(&self.newton, ptr as _) };
                let mut matrix = body.matrix();
                let vel = body.velocity();
                sel.position = [matrix.c3.x, matrix.c3.y, matrix.c3.z];
                sel.velocity = [vel.x, vel.y, vel.z];
                sel.name = (body.name(), );

                if body_popup {
                    ui.open_popup(im_str!("##body_popup"))
                }
                ui.popup(im_str!("##body_popup"), || {
                    if ui.menu_item(im_str!("Awake")).build() { body.active() }
                    if ui.menu_item(im_str!("Sleep")).build() { body.asleep() }
                    ui.separator();
                    drop_body = ui.menu_item(im_str!("Destroy")).build();
                    focus_cam = ui.menu_item(im_str!("Focus")).build();
                });

                if body_popup {
                    ui.open_popup(im_str!("##body_popup"))
                }

                ui.window(im_str!("Body Inspector"))
                    //.title_bar(false)
                    .movable(false)
                    .resizable(false)
                    //.position((w as f32 - width*2.0 - margin*2.0, margin), ImGuiCond::Always)
                    .position((margin, margin), ImGuiCond::Always)
                    //.size((width, 200.0), ImGuiCond::Always)
                    .always_auto_resize(true)
                    .build(|| {
                        if ui.imgui_ext(&mut sel).position() {
                            matrix.c3 = Vector::from(sel.position);
                            body.set_matrix(&matrix);
                        }
                    });

                if focus_cam {
                    let cam = &mut self.renderer.params_mut().camera;
                    cam.center = sel.position;
                }
                if drop_body {
                    let handle = body.into_handle();
                    let _ = self.newton.take_body(&handle);
                } else {
                    self.selected = Some(sel);
                }
            }

            imgui_renderer.render(ui);
            self.sdl_window.gl_swap_window();
        }

        Ok(())
    }
}
