#[macro_use]
extern crate newton_sandbox;

use newton_sandbox::Sandbox;

use newton_sandbox::imgui::{im_str, ImGuiCond, Ui};
use newton_sandbox::math::*;
use newton_sandbox::{NewtonBody, NewtonCollision, NewtonWorld, SandboxHandler};

use newton::callback::Gravity;

struct MyHandler {
    collisions: [NewtonCollision; 4],
    bodies: Vec<NewtonBody>,
}

impl MyHandler {
    fn spawn_body(&mut self, shape: NewtonCollision) {
        let body = shape.body(Matrix4::from_translation(Vector3::new(0.0, 16.0, 0.0)));
        body.set_update::<Gravity>();
        body.set_mass(1.0);
        self.bodies.push(body);
    }
}

impl SandboxHandler for MyHandler {
    fn imgui(&mut self, ui: &Ui) {
        ui.window(im_str!("Hello world"))
            .size((300.0, 200.0), ImGuiCond::FirstUseEver)
            .build(|| {
                if ui.button(im_str!("Spawn Box"), (128.0, 24.0)) {
                    self.spawn_body(self.collisions[0].clone())
                }
                if ui.button(im_str!("Spawn Cylinder"), (128.0, 24.0)) {
                    self.spawn_body(self.collisions[3].clone())
                }
                if ui.button(im_str!("Spawn Sphere"), (128.0, 24.0)) {
                    self.spawn_body(self.collisions[2].clone())
                }
                if ui.button(im_str!("Spawn Cone"), (128.0, 24.0)) {
                    self.spawn_body(self.collisions[1].clone())
                }
            });
    }
}

fn create_container(world: &NewtonWorld, bodies: &mut Vec<NewtonBody>) {
    let floor = NewtonCollision::cuboid(&world, 16.0, 0.2, 16.0, 0, None);
    bodies.push(floor.body(Matrix4::from_translation(Vector3::new(0.0, 0.0, 0.0))));

    let wall = NewtonCollision::cuboid(&world, 16.0, 2.0, 0.2, 0, None);
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(0.0, 1.0, 8.0))));
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(0.0, 1.0, -8.0))));

    let wall = NewtonCollision::cuboid(&world, 0.2, 2.0, 16.0, 0, None);
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(8.0, 1.0, 0.0))));
    bodies.push(wall.body(Matrix4::from_translation(Vector3::new(-8.0, 1.0, 0.0))));
}

fn main() {
    let world = NewtonWorld::new();

    let shapes = [
        NewtonCollision::cuboid(&world, 1.0, 1.0, 1.0, 0, None),
        NewtonCollision::cone(&world, 1.0, 1.0, 0, None),
        NewtonCollision::sphere(&world, 0.6, 0, None),
        NewtonCollision::cylinder(&world, 1.0, 1.0, 0, None),
    ];

    let polys = shapes[0].polygons(Matrix4::identity());
    //println!("{:#?}", polys);

    let mut bodies: Vec<_> = [(0.623, 0.245), (-0.123, -0.145), (0.023, -0.245)]
        .iter()
        .cycle()
        .take(24)
        .enumerate()
        .map(|(i, &(x, z))| Vector3::new(x, 4.0 + (i as f32) * 1.2, z))
        .map(Matrix4::from_translation)
        .enumerate()
        .map(|(i, m)| {
            let body = shapes[i % 4].body(m);
            body.set_mass(1.0);
            body.set_update::<Gravity>();
            body
        })
        .collect();

    // create a floor and some walls to contain the bodies
    create_container(&world, &mut bodies);

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
        .run(
            world,
            Some(MyHandler {
                collisions: shapes,
                bodies,
            }),
        );
}
