use newton::newton2::{Application, Types};
use newton::newton2::world::World;
use newton::newton2::collision::BoxCollision;
use newton::newton2::body::DynamicBody;

use std::time::Duration;
use std::cell::RefMut;

use cgmath::Matrix4;
use cgmath::prelude::*;

#[derive(Debug)]
struct App;
impl Types for App {
    type Vector = [f32; 3];
    type Matrix = Matrix4<f32>;
    type Quaternion = [f32; 4];
}
impl Application for App {
    type Types = Self;
}

fn main() {
    let world = World::<App>::new();
    let collision = BoxCollision::new(world.borrow_mut(), 1.0, 1.0, 1.0, 0, None);

    {
        let c = collision.borrow();
        let w = world.borrow();
    }

    // panic!
    {
        let w = world.borrow_mut();
        //let c = collision.borrow();
    }

    {
        let collision = BoxCollision::new(world.borrow_mut(), 1.0, 1.0, 1.0, 0, None);
        let c = collision.borrow_mut();
        let body = DynamicBody::new(c, &Matrix4::identity());

        println!("bodies: {:?}", world.borrow().body_count());
    }

    println!("bodies: {:?}", world.borrow().body_count());

}

#[cfg(mockup)]
fn main() {
    // We can't perform API calls on this type
    let world: World = World::new(BroadPhaseAlgorithm::Default, SandboxApp);

    let collision = {
        // Instead we need to borrow it like this:
        let world_mut: RefMut<WorldInner> = world.borrow_mut();

        // So we can update simulation data like this:
        // signature: BoxCollision::new(&mut WorldInner, ...)

        BoxCollision::new(&world_mut, 1.0, 1.0, 1.0, None)
    };

    // ... TODO create collision & body
    let body;

    // transform body
    {
        let body = body.borrow_mut();

        body.set_matrix(identity);
        body.awake();
        body.set_mass(1.0);
    }

    // iterate world bodies
    for (i, body) in world.borrow().bodies() {
        let body = body.borrow();

        println!("Position: {:?}", body.position());
        println!("Rotation: {:?}", body.rotation());

        // panic!
        // body.set_position( ... );
    }

    // Updating the world
    //
    // {
    //     // this would panic!
    //
    //     let b = body.borrow();
    //     world.bottow_mut().step(Duration::new(1, 0));
    // }
    {
        world.borrow_mut().update(Duration::new(1, 0));
    }

    // another panic
    // {
    //     let world = world.borrow_mut();
    //     let iterator = world.bodies();
    // }

    // create a new body
    //
    // signature: DynamicBody::new(&mut Collision)
    //
    // We will need to store some data in the collision userdatum...
    let body = DynamicBody::new(collision.borrow_mut(), Matrix4::identity());

    {
        let body = body.borrow_mut();
        body.awake();
        body.set_matrix(..);
    }

    for joint in body.borrow().joints() {
        // ...

        // panic!
        // world.borrow_mut()

        // world holds the guard...
    }
}
