use newton::world;
use newton::Types;

use cgmath::{prelude::*, Matrix4, Quaternion, Vector3};

#[derive(Clone)]
enum M {}
unsafe impl Types for M {
    type Vector = Vector3<f32>;
    type Matrix = Matrix4<f32>;
    type Quaternion = Quaternion<f32>;
}

fn main() {
    let gravity = Vector3::new(0.0, -9.8, 0.0);

    let world = world::WorldBuilder::<M>::new()
        // define the contact generation callback in the body builder
        .contact_gen_callback(|_mat, _b0, _c0, _b1, _c1, _cont: &[()]| true)
        // define the AABB callback in the builder
        //.collision_callback(|_mat, _b0, _b1| true)
        // force torque callback
        .force_torque_callback(move |b, _, _| b.set_force(&gravity))
        .build();

    let mut world = world.write();
    let (a, b, c) = world.create_materials();

    // assigns the callback to the material edge
    world.handle_contact(a, b);
    world.handle_contact(b, c);

    // contact collision callbacks
    world.handle_collision(a, c);
}
