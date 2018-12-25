#![allow(unused_variables)]

use newton::types::Cgmath;

use newton::body::{self, NewtonBody};
use newton::collision;
use newton::world;

use newton::sandbox;
use newton::sandbox::cgmath::{vec3, Matrix4, Quaternion, Vector3};

fn main() {
    let world = world::create::<Cgmath>();

    let mut w = world.borrow_mut();
    let pool = [
        collision::cuboid(&mut w, 12.0, 1.0, 12.0, 0, None),
        collision::cuboid(&mut w, 1.0, 1.0, 1.0, 0, None),
    ];
    drop(w);

    let floor = body::dynamic(&mut pool[0].borrow_mut(), &position(0.0, 0.0, 0.0));

    // fine
    for body in world.borrow().bodies() {
        println!("{:?}", body.borrow().position());
        drop(body)
    }
    for body in world.borrow_mut().bodies() {
        // NOT ok. Panics
        //println!("{:?}", body.borrow().position());
    }

    // When all references to a body are dropped, the body performs a mutable borrow
    // of the world, or panics if it is unsable to. This borrow checking is enforced
    // to prevent segfaults such as this one:
    {
        // `floor` is the only body in the world.
        let world = world.borrow();

        // We borrow world in order to get an body iterator.
        let mut bodies = world.bodies();

        // then drop the only body in the iterator
        //drop(floor);

        drop(bodies.next().unwrap()) // Segfault
    }

    // run a visual simulation
    let mut coll = pool[1].borrow_mut();

    let cube_2 = body::dynamic(&mut coll, &position(0.5, 9.5, 0.25));
    let cube_3 = body::dynamic(&mut coll, &position(-0.25, 5.0, -0.5));
    let cube_4 = body::dynamic(&mut coll, &position(0.1, 6.5, 0.4));

    drop(coll);

    cube_2.borrow_mut().set_mass(1.0);
    cube_2.borrow_mut().set_force_and_torque_callback::<Gravity>();

    cube_4.borrow_mut().set_mass(1.0);
    cube_4.borrow_mut().set_force_and_torque_callback::<Gravity>();

    sandbox::run(world.borrow().as_raw());
}

fn position(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}

pub enum Gravity {}
impl newton::ForceAndTorque<Cgmath> for Gravity {
    fn force_and_torque(body: &mut NewtonBody<Cgmath>) {
        body.set_force(&vec3(0.0, -9.81, 0.0));
    }
}
