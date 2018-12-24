extern crate newton;

use std::time::Duration;

use newton::sandbox::{cgmath::prelude::*, cgmath::Matrix4, cgmath::Vector3, SandboxApp};

use newton::prelude::*;

fn main() {
    // We can't perform API calls on this type
    let world: World = World::new(BroadPhaseAlgorithm::Default, SandboxApp);

    let collision = {
        // Instead we need to borrow it like this:
        let world_mut: RefMut<WorldInner> = world.borrow_mut();

        // So we can update simulation data like this:
        // signature: BoxCollision::new()

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
    let collision = DynamicBody::new(&)
}
