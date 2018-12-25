use newton::body::{Body, BodyType};
use newton::collision::{Collision, CollisionParams as Params};

use newton::sandbox::cgmath::prelude::*;
use newton::sandbox::cgmath::{vec3, Matrix4};
use newton::sandbox::Sandbox;

fn main() {
    let mut sandbox = Sandbox::new();

    let world = sandbox.world();

    let pool = [
        Collision::new(world.borrow_mut(),
                       Params::Box { dx: 8.0, dy: 1.0, dz: 8.0, },
                       0,
                       None),
        Collision::new(world.borrow_mut(),
                       Params::Box { dx: 1.0, dy: 1.0, dz: 1.0 },
                       0,
                       None),
    ];

    let floor = Body::new(pool[0].borrow_mut(),
                          BodyType::Dynamic,
                          &position(0.0, 0.0, 0.0));

    let cube_1 = Body::new(pool[1].borrow_mut(),
                           BodyType::Dynamic,
                           &position(0.0, 8.0, 0.0));

    let cube_2 = Body::new(pool[1].borrow_mut(),
                           BodyType::Dynamic,
                           &position(0.5, 9.5, 0.25));

    cube_1.borrow_mut().set_mass(1.0);
    cube_2.borrow_mut().set_mass(1.0);

    run(sandbox);
}

fn position(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}

fn run(mut app: Sandbox) {
    app.window_size(1280, 720);
    app.render_solid(true);
    app.render_wireframe(true);
    app.render_aabb(false);
    app.simulate(false);
    app.run();
}
