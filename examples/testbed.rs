use std::error::Error;
use std::time::Duration;

use newton::testbed::{Demo, Testbed};
use newton::{body, BodyOld, CollisionOld, Matrix, Newton, Vector};
use newton::body::{BodyTrait, DynamicBody};
use newton::collision::{BoxCollision, CollisionTrait, SphereCollision};

struct Example;

impl Demo for Example {
    fn reset(newton: &Newton) -> Self {
        let x = [1.0, 0.0, 0.0, 0.0];
        let y = [0.0, 1.0, 0.0, 0.0];
        let z = [0.0, 0.0, 1.0, 0.0];
        let w = [0.0, 0.0, 0.0, 1.0];

        let cube = BoxCollision::create(newton, 1.0, 1.0, 1.0, None, Some("box0"));
        let cube = SphereCollision::create(newton, 1.0, None, Some("sphere_a"));

        println!("{:?}", cube.name());
        let body = DynamicBody::create(newton, &cube, [x, y, z, w], Some("body0"));

        let gravity = [0.0, -9.8, 0.0];
        body.set_mass(1.0, &cube);
        body.set_force_and_torque_callback(move |b, _, _| b.set_force(gravity));

        //drop(cube);
        //println!("{:?}", body.collision().name());
        body.into_handle(newton);

        Self
    }
}

fn main() {
    Testbed::<Example>::run().unwrap();
}
