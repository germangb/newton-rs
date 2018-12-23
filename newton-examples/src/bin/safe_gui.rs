#[macro_use]
extern crate newton_sandbox;

use newton_sandbox::math::*;
use newton_sandbox::{Event, Handler, Sandbox};

#[derive(Clone)]
enum Either<A, B> {
    A(A),
    B(B),
}

struct Example {}

impl Handler for Example {}

fn main() {
    let mut sandbox = sandbox();

    let collision = [
        Either::A(newton::BoxCollision::new(
            sandbox.world(),
            1.0,
            1.0,
            1.0,
            0,
            None,
        )),
        Either::B(newton::ConeCollision::new(
            sandbox.world(),
            1.0,
            2.0,
            0,
            None,
        )),
    ];

    let transform = [(-0.51_f32, -0.41_f32), (-0.25, 0.17), (0.35, -0.12)];

    let collision = collision.iter().cloned().cycle();

    let transform = transform
        .iter()
        .cycle()
        .enumerate()
        .map(|(i, (x, z))| Vector3::new(*x, (i as f32) * 1.5 + 4.0, *z))
        .map(Matrix4::from_translation);

    let mut bodies: Vec<_> = collision
        .zip(transform)
        .take(16)
        .map(|(c, m)| {
            let body = match c {
                Either::A(cuboid) => newton::Body::from(cuboid, m),
                Either::B(cone) => newton::Body::from(cone, m),
            };
            body.set_mass(1.0);
            body
        })
        .collect();

    let floor = newton::BoxCollision::new(sandbox.world(), 16.0, 1.0, 16.0, 0, None);
    bodies.push(newton::Body::from(floor, Matrix4::identity()));

    sandbox.run(bodies);
}

fn sandbox() -> Sandbox {
    let mut sandbox = Sandbox::new(Example {});

    sandbox
        .window_size(1280, 720)
        .background_color(rgba!(1.0))
        // default rendering params
        .render_solid(true)
        .render_wireframe(true)
        .render_aabb(false)
        // Don't start simulation right away
        .simulate(false);
    sandbox
}
