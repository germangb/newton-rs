use newton::sandbox::cgmath::prelude::*;
use newton::sandbox::cgmath::*;

use newton::sandbox::{BallJoint, DynamicBody, Event, Handler, Keycode, Sandbox, UpVectorJoint};

use newton::color;
use newton::prelude::*;

struct Input {
    w: bool,
    a: bool,
    s: bool,
    d: bool,
}

struct Example {
    input: Input,
    capsule: DynamicBody,
    up: Option<UpVectorJoint>,
}

impl Handler for Example {
    fn post_update(&mut self) {
        self.capsule.awake();
    }
}

fn main() {
    let mut sandbox = sandbox();

    let transform = [(-0.51_f32, -0.41_f32), (-0.25, 0.17), (0.35, -0.12)];

    let world = sandbox.world();

    let collision = [
        newton::collision::BoxCollision::new(world, 1.0, 1.0, 1.0, 0, None).into_collision(),
        newton::collision::ConeCollision::new(world, 1.0, 2.0, 0, None).into_collision(),
        newton::collision::SphereCollision::new(world, 0.8, 0, None).into_collision(),
    ];

    let collision = collision.iter().cloned().cycle();
    let transform = transform
        .iter()
        .cycle()
        .enumerate()
        .map(|(i, (x, z))| Vector3::new(*x, (i as f32) * 1.5 + 8.0, *z))
        .map(Matrix4::from_translation);

    let mut bodies: Vec<_> = collision
        .zip(transform)
        .take(24)
        .map(|(c, m)| {
            let body = newton::body::DynamicBody::new(c, m);
            body.set_mass(1.0);
            body
        })
        .collect();

    let floor = newton::collision::BoxCollision::new(sandbox.world(), 16.0, 1.0, 16.0, 0, None);
    //let floor = newton::DynamicBody::from(floor, Matrix4::identity());
    //bodies.push(floor);

    // heightfield
    let mut params = newton::collision::HeightFieldParams::<f32>::new(32, 32);
    params.set_vertical_scale(0.0);
    //params.set_horizontal_scale(8.0, 8.0);
    let heightfield = newton::collision::HeightFieldCollision::new(
        &world,
        params,
        0,
        Some(Matrix4::from_translation(Vector3::new(-16.0, 0.0, -16.0))),
    );

    let heightfield = newton::body::DynamicBody::new(heightfield, Matrix4::identity());
    heightfield.set_mass(1.0);
    bodies.push(heightfield);

    let capsule = newton::collision::CapsuleCollision::new(sandbox.world(), 1.0, 1.0, 1.0, 0, None);
    let t =
        Matrix4::from_translation(Vector3::new(4.0, 4.0, 0.0)) * Matrix4::from_angle_z(Deg(45.0));
    let capsule = DynamicBody::new(capsule, t);
    bodies.push(capsule.clone());

    let capsule = capsule.into_body();
    let up = newton::constraint::UpVectorJoint::new(&capsule, Vector3::new(0.0, 1.0, 0.0));
    let capsule = capsule.dynamic().unwrap();

    capsule.set_mass(1.0);
    sandbox.run(Example {
        capsule,
        up: Some(up),
        input: Input {
            w: false,
            a: false,
            s: false,
            d: false,
        },
    });
}

fn sandbox() -> Sandbox {
    let mut sandbox = Sandbox::new();

    sandbox
        .window_size(1280, 666)
        .background_color(color!(1.0))
        // default rendering params
        .render_solid(true)
        .render_wireframe(true)
        .render_aabb(true)
        // Don't start simulation right away
        .simulate(false);
    sandbox
}
