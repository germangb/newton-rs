use newton::prelude::*;
use newton::{
    BoxCollision, Collision, CompoundCollision, ConeCollision, CylinderCollision, DynamicBody,
    Newton, SphereCollision,
};

use newton::testbed;
use newton::testbed::Demo;

struct Compound;
impl Demo for Compound {
    fn reset(newton: &mut Newton) -> Self {
        let a = BoxCollision::create(newton, 1.0, 1.0, 1.0, None);
        let b = SphereCollision::create(newton, 0.5, Some(transform(0.0, 1.0, 0.0)));
        let c = CylinderCollision::create(newton, 0.5, 0.5, 1.0, Some(transform(1.0, 0.0, 0.0)));
        let d = ConeCollision::create(newton, 0.5, 1.0, Some(transform2(0.0, -1.0, 0.0)));

        let mut compound = CompoundCollision::create(newton);
        {
            let builder = compound.begin();
            let h = builder.add(&a);
            builder.add(&b);
            builder.add(&c);
            builder.add(&d);
            //builder.remove(h);
        }

        DynamicBody::create(newton, &b, transform(0.0, 0.0, 0.1), None).into_handle(newton);

        let body = DynamicBody::create(
            newton,
            &compound,
            transform(0.0, 2.0, 0.0),
            Some("compound"),
        );

        body.set_mass(1.0, &compound);
        body.set_force_and_torque_callback(|b, _, _| b.set_force([0.0, -9.8, 0.0]));
        body.into_handle(newton);

        // ground
        let ground = BoxCollision::create(newton, 8.0, 0.5, 8.0, None);
        DynamicBody::create(newton, &ground, transform(0.0, 0.0, 0.0), None).into_handle(newton);

        Compound
    }
}

const fn transform2(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [x, y, z, 1.0],
    ]
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
