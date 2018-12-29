use newton::sandbox;
use newton::types::Cgmath as C;

use newton::body::{Body, Type};
use newton::collision::{Collision, Params};
use newton::world::{Broadphase, Solver, Threads, World};

use cgmath::prelude::*;
use cgmath::{vec3, Matrix4};

use std::time::Duration;

#[cfg(bad_idea)]
#[cfg(idea_0)]
fn main_() {
    let world: World<C> = World::new(Broadphase::Default, Solver::Linear(8), Threads::One);

    // Locking and unlocking is not explicit.
    // No use of rust type system
    let coll = Collision::new(&world, Params::Box(1.0, 1.0, 1.0), 0);
    let cube = Body::new(&world, &coll, Type::Dynamic, &Matrix4::identity());
}

fn main() {
    // parameters seem a bit obscure. The user must be aware of the generic type
    let world: World<C> = World::new(Broadphase::Default, Solver::Linear(8), Threads::One);

    // borrow world and create a new collision
    // Do we need to borrow workd mutably?
    let mut world_w = world.write();

    let coll = Collision::new(&mut world_w, Params::Box(1.0, 1.0, 1.0), 0); // (0)

    // create a body. We need to borrow collision immutably first.
    let coll_r = coll.read();
    // Dynamic and kinematic bodies share the same type. I'd like to keep it that way (I can't think
    // of any benefits in not doing it). The types make it explicit that this object will mutate
    // thw world (out context)
    let cube = Body::new(&mut world_w, &coll_r, Type::Dynamic, &Matrix4::identity());

    // I want to mutate the body in order to set the force_and_torque callback and the mass
    // The problem is, world has been locked by the previous calla to `world.write()`, so if I
    // write this, I get a panic:
    #[cfg(do_not_do_this)]
    {
        cube.write().set_mass(1.0); // (1)
    }

    // (1) I have to do this instead. Manually dropping world is an ugly overhead that NEEDS to go away
    {
        drop(world_w);
        cube.write().set_mass(1.0);
        cube.write()
            .set_force_and_torque(|b, _, _| b.set_force(&vec3(0.0, -9.8, 0.0)));
    }

    // (2) This panics
    //     (UPDATE) Tried not dropping the body right away
    //#[cfg(do_not_do_this)]
    {
        let mut w = world.write();

        // drop a body panics because it internally locks world
        let _ = Body::new(&mut w, &coll.read(), Type::Dynamic, &Matrix4::identity()); // oh, no!
    }
    // Oh, and if by the end of the scope, we still have a lock to the World, we get a panic!
    // When a body is droped, it attempts to acquire the lock. Because the body is dropped before the
    // lock is, we get panic!...
    //drop(world_w);

    world.write().update(Duration::new(0, 1_000_000_000 / 60));

    let world_ptr = world.read().as_raw();
    sandbox::run(world_ptr);

    // compiler complains about
    #[cfg(do_not_do_this)]
    {
        sandbox::run(world.read().as_raw());
    }
}

// What needs to be improved:
//
// (0) We may not need to borrow world mutably to create new collisions, since they have no effect
//     unless they are attached to some dynamic body. We still need to lock the work in order to
//     update collision information though.
//
// (1) I like that locking the works is explicit. It allows a degree of optimization, and it uses
//     the type system (to some degree). But I'm not completely sold on having to manuallt call
//     `drop()` on these locks. I think the API should remove the friction common tasks.
//
// (2) This is unacceptable. Re-implement how bodies is dropped. Perhaps we can queue some sort
//     of "destroy body" event that gets dispatched on the world's `update()` and `update_async()`
//     methods (?)
//
//     (UPDATE)
//         We can't drop the body userdata because we may still need it in the body iterators!!
//         Perhaps this command sending mechanism needs to be meade explicit somehow
