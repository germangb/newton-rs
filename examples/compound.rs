use newton::prelude::*;
use newton::{Collision, Compound, Cone, Cuboid, Cylinder, DynamicBody, Handle, Newton, Sphere};

use newton::testbed;
use newton::testbed::Demo;

struct Example;
impl Demo for Example {
    fn reset(newton: &mut Newton) -> Self {
        let a = Cuboid::create(newton, 1.0, 1.0, 1.0, None);
        let b = Sphere::create(newton, 0.5, Some(transform(0.0, 1.0, 0.0)));
        let c = Cylinder::create(newton, 0.5, 0.5, 1.0, Some(transform(1.0, 0.0, 0.0)));
        let d = Cone::create(newton, 0.5, 1.0, Some(transform2(0.0, -1.0, 0.0)));

        let mut compound = Compound::create(newton);
        {
            let builder = compound.begin();
            let h = builder.add(&a);
            builder.add(&b);
            builder.add(&c);
            builder.add(&d);

            builder.remove(Handle::Index(0));
            builder.remove(Handle::Index(0));
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
        let ground = Cuboid::create(newton, 8.0, 0.5, 8.0, None);
        DynamicBody::create(newton, &ground, transform(0.0, 0.0, 0.0), None).into_handle(newton);

        Self
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
    testbed::run::<Example>()
}
