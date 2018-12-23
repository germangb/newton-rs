extern crate newton;

use std::time::Duration;

use newton::sandbox::{cgmath::prelude::*, cgmath::Matrix4, cgmath::Vector3, SandboxApp};

use newton::prelude::*;
use newton::BroadPhaseAlgorithm as Algorithm;
use newton::{BoxCollision, DynamicBody, KinematicBody};

fn main() {
    let world = newton::World::new(Algorithm::Default, SandboxApp);

    // create a collision shape
    let shape = BoxCollision::new(&world, 1.0, 1.0, 1.0, 0, None);

    // create a few bodies..
    let body_a = DynamicBody::from(shape.clone(), Matrix4::identity());
    let body_b = KinematicBody::from(shape, Matrix4::identity());

    // Create a joint
    {
        let body = body_a.clone().into_body();
        let j = newton::UpVectorJoint::new(&body_a, Vector3::new(0.0, 1.0, 0.0));
        assert_eq!(1, world.constraint_count());
    }

    assert_eq!(0, world.constraint_count());

    // heightfield
    let params = newton::collision::HeightFieldParams::<f32>::new(16, 16);
    let heightfield = newton::HeightFieldCollision::new(&world, params, 0);
    let heightfield = newton::DynamicBody::from(heightfield, Matrix4::identity());
}
