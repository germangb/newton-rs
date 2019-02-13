use newton::{body, Body, Collision, Newton};

use std::time::Duration;

fn main() {
    let mut world = Newton::new();
    world.set_exact_solver();
    world.set_threads_count(num_cpus::get());

    let body = create_body(&world);

    //let _ = world.body_owned(&body);

    world.update(Duration::new(1, 0));
}

fn create_body(world: &Newton) -> body::Handle {
    let ident = newton::math::identity();
    let collision = Collision::box2(&world, 1.0, 1.0, 1.0, None);

    //collision.for_each_poly(&ident, |face, _| println!("{:?}", face));

    let body = Body::dynamic(world, &collision, &ident);

    let gravity = [0.0, -9.8, 0.0];
    body.set_force_and_torque_callback(move |mut b| b.set_force(&gravity));
    body.set_mass(1.0, &collision);
    body.into_handle(world)
}
