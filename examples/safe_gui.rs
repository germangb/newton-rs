#[macro_use]
extern crate lazy_static;
extern crate newton_dynamics;
extern crate cgmath;
extern crate sdl2;
extern crate gl;

mod aux;

use aux::Example;

use newton_dynamics::{
    NewtonWorld, NewtonBody, NewtonCollision,

    traits::NewtonCgmath as Cgmath,
};

use cgmath::Matrix4;
use cgmath::prelude::*;

use std::time::Duration;

struct Demo {
    world: NewtonWorld<Cgmath>,
    bodies: Vec<NewtonBody<Cgmath>>,
}

impl Example<Cgmath> for Demo {
    fn bodies(&self) -> &[NewtonBody<Cgmath>] {
        &self.bodies[..]
    }

    fn world(&self) -> (&NewtonWorld<Cgmath>, Duration) {
        (&self.world, Duration::new(0, 1_000_000_000 / 60))
    }
}

fn main() {
    let mut demo = Demo {
        world: NewtonWorld::new(),
        bodies: Vec::new(),
    };

    let shape = NewtonCollision::new_box(&demo.world, (1.0, 0.0, 0.0), 0.into(), None);
    let body = NewtonBody::new(&demo.world, shape, Matrix4::identity());
    demo.bodies.push(body);

    aux::run(demo);
}
