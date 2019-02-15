use std::error::Error;
use std::time::Duration;

use newton::testbed::Testbed;
use newton::{body, Body, Collision, Matrix, Newton, Vector};

fn main() -> Result<(), Box<dyn Error>> {
    let mut world = Newton::create();
    let body = create_body(&world);

    let floor_c = Collision::box2(&world, 8.0, 0.1, 8.0, None);
    Body::dynamic(&world, &floor_c, &Matrix::identity()).into_handle();
    drop(floor_c);

    Testbed::new(world)?.run()?;

    Ok(())
}

fn create_body(world: &Newton) -> body::Handle {
    let mut ident = Matrix::identity();
    ident.c3.y = 6.0;
    let collision = Collision::box2(&world, 1.0, 1.0, 1.0, None);

    //collision.polygons(&ident, |face, _| println!("{:?}", face));

    let body = Body::dynamic(world, &collision, &ident);

    let gravity = Vector::new3(0.0, -9.8, 0.0);
    body.set_force_and_torque_callback(move |b, _| b.set_force(&gravity));
    body.set_mass(1.0, &collision);
    body.into_handle()
}
