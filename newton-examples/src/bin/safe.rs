use newton::newton2::body::{DynamicBody, KinematicBody};
use newton::newton2::collision::BoxCollision;

use newton::newton2::sandbox::cgmath::prelude::*;
use newton::newton2::sandbox::cgmath::{vec3, Matrix4};
use newton::newton2::sandbox::Sandbox;

fn main() {
    let mut sandbox = Sandbox::new();

    let world = sandbox.world();

    let collision = [
        BoxCollision::new(world.borrow_mut(), 8.0, 1.0, 8.0, 0, None),
        BoxCollision::new(world.borrow_mut(), 1.0, 1.0, 1.0, 0, None),
    ];

    let floor = DynamicBody::new(collision[0].borrow_mut(), &position(0.0, 0.0, 0.0));
    let cube = DynamicBody::new(collision[1].borrow_mut(), &position(0.0, 8.0, 0.0));

    cube.borrow_mut().set_mass(1.0);

    run(sandbox);
}

fn position(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}

fn run(mut app: Sandbox) {
    app.window_size(1280, 720);
    app.render_solid(true);
    app.render_wireframe(true);
    app.render_aabb(true);
    app.simulate(false);
    app.run();
}
