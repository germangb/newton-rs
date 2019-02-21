use newton::prelude::*;
use newton::{
    BoxCollision, Collision, CompoundCollision, CylinderCollision, DynamicBody, Newton,
    SphereCollision,
};

use newton::testbed;
use newton::testbed::Demo;

struct Compound;
impl Demo for Compound {
    fn reset(newton: &mut Newton) -> Self {
        let nodes = &[
            Collision::Box(BoxCollision::create(newton, 1.0, 1.0, 1.0, None, None)),
            Collision::Sphere(SphereCollision::create(
                newton,
                0.5,
                Some(transform(0.0, 1.0, 0.0)),
                None,
            )),
            Collision::Cylinder(CylinderCollision::create(
                newton,
                0.5,
                0.5,
                1.0,
                Some(transform(1.0, 0.0, 0.0)),
                None,
            )),
        ];

        let compound = CompoundCollision::create(newton, nodes, None);
        let body = DynamicBody::create(newton, &compound, transform(0.0, 0.0, 0.0), None);

        body.set_mass(1.0, &compound);
        body.set_force_and_torque_callback(|b, _, _| b.set_force([0.0, -9.8, 0.0]));
        body.into_handle(newton);

        // ground
        let ground = BoxCollision::create(newton, 8.0, 0.5, 8.0, None, None);
        DynamicBody::create(newton, &ground, transform(0.0, 0.0, 0.0), None).into_handle(newton);

        Compound
    }
}

const fn transform(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [x, y, z, 1.0],
    ]
}

fn main() {
    testbed::run::<Compound>()
}
