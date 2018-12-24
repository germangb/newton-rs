extern crate newton;

use std::time::Duration;

use newton::sandbox::{cgmath::prelude::*, cgmath::Matrix4, cgmath::Vector3, SandboxApp};

use newton::prelude::*;

fn main() {
    let world = World::new(BroadPhaseAlgorithm::Default, SandboxApp);

    // create a collision shape
    let shape = BoxCollision::new(&world, 1.0, 1.0, 1.0, 0, None);

    // create a few bodies..
    let body_a = DynamicBody::new(shape.clone(), Matrix4::identity());
    let body_b = KinematicBody::new(shape, Matrix4::identity());

    // Create a joint
    {
        let body = body_a.clone().into_body();
        let j = UpVectorJoint::new(&body, Vector3::new(0.0, 1.0, 0.0));
        assert_eq!(1, world.constraint_count());
    }

    assert_eq!(0, world.constraint_count());

    // heightfield
    let params = HeightFieldParams::<f32>::new(16, 16);
    let heightfield = HeightFieldCollision::new(&world, params, 0, None);
    let heightfield = DynamicBody::new(heightfield, Matrix4::identity());

    for body in world.bodies() {
        println!("{}", world.body_count());
        println!("{}", world.body_count() + 1);
        println!("{}", world.body_count() + 2);
        println!("{:?}", body);

        for body in world.bodies() {}
        //world.update(Duration::new(1, 0));
    }

    world.update(Duration::new(1, 0));
}
