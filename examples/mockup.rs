use std::time::Duration;

use newton::{body, Body, Collision, Matrix, Newton, Vector};

fn main() {
    let mut world = Newton::create();

    let _body = create_body(&world);

    //let _ = world.body_owned(&body);

    world.update(Duration::new(1, 0));
}

fn create_body(world: &Newton) -> body::Handle {
    let ident = Matrix::identity();
    let collision = Collision::box2(&world, 1.0, 1.0, 1.0, None);

    //collision.for_each_poly(&ident, |face, _| println!("{:?}", face));

    let body = Body::dynamic(world, &collision, &ident);

    let gravity = Vector::new3(0.0, -9.8, 0.0);
    body.set_force_and_torque_callback(move |b| b.set_force(&gravity));
    body.set_mass(1.0, &collision);
    body.into_handle()
}
