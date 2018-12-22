extern crate newton;
extern crate newton_sandbox;

use newton_sandbox::math::*;
use newton_sandbox::{NewtonBody, NewtonWorld};

use newton::callback::Gravity;
use newton::collision::CollisionBox;

use std::time::Duration;

fn main() {
    let world = NewtonWorld::new();

    let body = NewtonBody::new(
        &world,
        CollisionBox::new(&world, 1.0, 1.0, 1.0, 0, None),
        Matrix4::identity(),
    );

    body.set_mass(1.0);
    body.set_update::<Gravity>();

    println!("Initial position: {:?}", body.position());

    for _ in 0..(60 * 4) {
        world.update(Duration::new(0, 1_000_000_000 / 60));
    }

    println!("Position after 4 seconds: {:?}", body.position());
}
