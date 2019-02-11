use std::time::Duration;

#[cfg(world_owned)]
// a good thing about this design is that is uses
// static borrow checking rather than dynamic.
fn main() {
    // in this design mockup, all objects are owned by the world.
    // Creating an object (collision, body, etc) yeields a handle that can
    // be used to mutate the object later (always through the world)
    let world = Newton::create();

    // create a collision handle
    // the wrapper adds additional information in the userdata pointer
    // that is transparent for the user but is used by the newton world.
    let collision = Collision::builder(&world).shape_id(0).build_sphere(1.0);

    // We keep using the same builder pattern from the previous design
    let builder = Body::builder(&world, &collision).transform(IDENT).force_and_torque(|b| b.apply_force(&GRAVITY));
    let bodies: Vec<_> = (0..4).map(|_| builder.build_dynamic()).collect();
}

const IDENT: [[f32; 4]; 4] = [..];
const GRAVITY: [f32; 3] = [0.0, -9.8, 0.0];

// original design with dynamic borrow checking rules
#[cfg(original)]
fn main() {
    // Newton context
    let world = Newton::builder().debug("Newton world")
        .mul_thread()
        .build();

    let ident = [..];

    let col = Collision::builder()
        // the debug name is used for error reporting
        // whenever a borrowing error happens at runtime.
        .debug("collision-0")
        .cuboid(&world.read(), 1.0, 1.0, 1.0);

    let gravity = [0.0, -9.81, 0.0];
    let body = Body::builder().debug("body-0")
        .transform(ident)
        .force_and_torque(|b| b.apply_force(&gravity))
        // create a dynamic body with the given collision
        .dynamic(&world.read(), &col.read());

    // step
    world.update_async(FRAME_STEP).wait_finished();

    for body in world.read().bodies_mut() {
        let mat = body.matrix();

        body.set_matrix(&mat);
        body.clear_forces();
        // ...
    }

    // by dropping bodies, ownership of the body is transfered to the World
    drop(body);
    world.write().drop_bodies();
}

static FRAME_STEP: Duration = ...;
