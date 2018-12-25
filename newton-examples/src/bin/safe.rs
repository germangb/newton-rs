use newton::{Application, Types};

use newton::world::{World, BroadphaseAlgorithm};
use newton::body::{Body, BodyType};
use newton::collision::{Collision, CollisionParams as Params};

use newton::sandbox::cgmath::prelude::*;
use newton::sandbox::cgmath::{vec3, Matrix4, Vector3, Quaternion};
use newton::sandbox::Sandbox;
use newton::body::NewtonBody;

#[derive(Debug)]
pub enum A {}
pub enum T {}
impl Types for T {
    type Vector = Vector3<f32>;
    type Matrix = Matrix4<f32>;
    type Quaternion = Quaternion<f32>;
}
impl Application for A {
    type Types = T;
    fn force_and_torque(body: &mut NewtonBody<Self>) {
        body.set_force(&vec3(0.0, -9.81, 0.0));
    }
}

fn main() {
    let world = World::<A>::new(BroadphaseAlgorithm::Default);

    let pool = {
        let mut world = world.borrow_mut();
        [
            Collision::new(&mut world, Params::Box { dx: 12.0, dy: 1.0, dz: 12.0, }, 0, None),
            Collision::new(&mut world, Params::Box { dx: 1.0, dy: 1.0, dz: 1.0 }, 0, None),
        ]
    };

    let floor = Body::new(&mut pool[0].borrow_mut(),
                          BodyType::Dynamic,
                          &position(0.0, 0.0, 0.0));

    let cube_2 = Body::new(&mut pool[1].borrow_mut(),
                           BodyType::Dynamic,
                           &position(0.5, 9.5, 0.25));

    let cube_3 = Body::new(&mut pool[1].borrow_mut(),
                           BodyType::Dynamic,
                           &position(-0.25, 5.0, -0.5));

    cube_2.borrow_mut().set_mass(1.0);

    let world = world.borrow_mut().as_raw();

    run(world as _);
}

fn position(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}

fn run(world: *mut ()) {
    let mut app = Sandbox::new(world as _);
    app.window_size(1280, 720);
    app.render_solid(true);
    app.render_wireframe(true);
    app.render_aabb(false);
    app.simulate(false);
    app.run();
}
