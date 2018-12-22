#[macro_use]
extern crate newton_sandbox;

use newton_sandbox::Sandbox;

use newton_sandbox::imgui::{im_str, ImGuiCond, Ui};
use newton_sandbox::math::*;
use newton_sandbox::{
    CollisionBox, CollisionCone, CollisionCylinder, CollisionSphere, NewtonBody, NewtonWorld,
    SandboxHandler,
};

use newton::callback::Gravity;

struct MyHandler {
    world: NewtonWorld,
    bodies: Vec<NewtonBody>,
}

impl SandboxHandler for MyHandler {
    fn imgui(&mut self, ui: &Ui) {
        ui.window(im_str!("Hello world"))
            .size((300.0, 200.0), ImGuiCond::FirstUseEver)
            .build(|| {
                if ui.button(im_str!("Spawn Box"), (128.0, 24.0)) {
                    let cube = CollisionBox::new(&self.world, 1.0, 1.0, 1.0, 0, None);
                    let body = NewtonBody::new(
                        &self.world,
                        &cube,
                        Matrix4::from_translation(Vector3::new(0.5, 4.0, 0.0)),
                    );
                    body.set_update::<Gravity>();
                    body.set_mass(1.0);

                    self.bodies.push(body);
                }
                if ui.button(im_str!("Spawn Sphere"), (128.0, 24.0)) {
                    let sphere = CollisionSphere::new(&self.world, 0.8, 0, None);
                    let body = NewtonBody::new(
                        &self.world,
                        &sphere,
                        Matrix4::from_translation(Vector3::new(0.5, 4.0, 0.0)),
                    );
                    body.set_update::<Gravity>();
                    body.set_mass(1.0);

                    self.bodies.push(body);
                }
                if ui.button(im_str!("Spawn Cone"), (128.0, 24.0)) {
                    let cone = CollisionCone::new(&self.world, 1.0, 1.0, 0, None);
                    let body = NewtonBody::new(
                        &self.world,
                        &cone,
                        Matrix4::from_translation(Vector3::new(0.5, 4.0, 0.0)),
                    );
                    body.set_update::<Gravity>();
                    body.set_mass(1.0);

                    self.bodies.push(body);
                }
                if ui.button(im_str!("Spawn Cylinder"), (128.0, 24.0)) {
                    let cone = CollisionCylinder::new(&self.world, 1.0, 1.0, 1.0, 0, None);
                    let body = NewtonBody::new(
                        &self.world,
                        &cone,
                        Matrix4::from_translation(Vector3::new(0.5, 4.0, 0.0)),
                    );
                    body.set_update::<Gravity>();
                    body.set_mass(1.0);

                    self.bodies.push(body);
                }
            });
    }
}

fn main() {
    let world = NewtonWorld::new();

    let shapes = [CollisionBox::new(&world, 1.0, 1.0, 1.0, 0, None)];

    //let polys = shapes[0].polygons(Matrix4::identity());
    //println!("{:#?}", polys);

    let mut bodies: Vec<_> = [(0.623, 0.245), (-0.123, -0.145), (0.023, -0.245)]
        .iter()
        .cycle()
        .take(4)
        .enumerate()
        .map(|(i, &(x, z))| Vector3::new(x, 4.0 + (i as f32) * 1.2, z))
        .map(Matrix4::from_translation)
        .enumerate()
        .map(|(i, m)| {
            /*
            let body = NewtonBody::new()
            body.set_mass(1.0);
            body.set_update::<Gravity>();
            body
            */
        })
        .collect();

    let mut bodies = Vec::new();

    let floor = CollisionBox::new(&world, 16.0, 0.2, 16.0, 0, None);
    bodies.push(NewtonBody::new(
        &world,
        &floor,
        Matrix4::from_translation(Vector3::new(0.0, 0.0, 0.0)),
    ));

    Sandbox::new()
        .window_size(1280, 720)
        .background_color(rgba!(0.9))
        // default rendering params
        .render_solid(true)
        .render_wireframe(true)
        .render_aabb(false)
        // only render the bodies inside this AABB
        .aabb(
            Vector3::new(-24.0, -24.0, -24.0),
            Vector3::new(24.0, 24.0, 24.0),
        )
        // Don't start simulation right away
        .simulate(false)
        .run(world.clone(), Some(MyHandler { world, bodies }));
}
