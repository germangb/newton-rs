use std::error::Error;
use std::time::Duration;

use newton::prelude::*;
use newton::{BoxCollision, DynamicBody, Newton};

use newton::testbed::{self, Demo};

struct Jenga;

impl Demo for Jenga {
    fn reset(newton: &mut Newton) -> Self {
        let x = [1.0, 0.0, 0.0, 0.0];
        let y = [0.0, 1.0, 0.0, 0.0];
        let z = [0.0, 0.0, 1.0, 0.0];
        let w = [0.0, 0.0, 0.0, 1.0];

        let floor = BoxCollision::create(newton, 8.0, 0.1, 8.0, None, Some("box0"));
        DynamicBody::create(newton, &floor, [x, y, z, w], Some("floor")).into_handle(newton);

        let p0 = BoxCollision::create(newton, 3.0, 0.5, 1.0, None, Some("box0"));
        let p1 = BoxCollision::create(newton, 1.0, 0.5, 3.0, None, Some("box0"));

        for i in 0..12 {
            let mut w = w;
            w[1] = i as f32 * 0.7 + 1.0;
            let (a, b, c) = match i & 1 {
                0 => {
                    w[2] = -1.1;
                    let a = DynamicBody::create(newton, &p0, [x, y, z, w], Some("piece0"));
                    a.set_mass(1.0, &p0);
                    w[2] = 0.0;
                    let b = DynamicBody::create(newton, &p0, [x, y, z, w], Some("piece0"));
                    b.set_mass(1.0, &p0);
                    w[2] = 1.1;
                    let c = DynamicBody::create(newton, &p0, [x, y, z, w], Some("piece0"));
                    c.set_mass(1.0, &p0);
                    (a, b, c)
                }
                _ => {
                    w[0] = -1.1;
                    let a = DynamicBody::create(newton, &p1, [x, y, z, w], Some("piece1"));
                    a.set_mass(1.0, &p1);
                    w[0] = 0.0;
                    let b = DynamicBody::create(newton, &p1, [x, y, z, w], Some("piece1"));
                    b.set_mass(1.0, &p1);
                    w[0] = 1.1;
                    let c = DynamicBody::create(newton, &p1, [x, y, z, w], Some("piece1"));
                    c.set_mass(1.0, &p1);
                    (a, b, c)
                }
            };

            let gravity = [0.0, -9.8, 0.0];
            a.set_force_and_torque_callback(move |b, _, _| b.set_force(gravity));
            b.set_force_and_torque_callback(move |b, _, _| b.set_force(gravity));
            c.set_force_and_torque_callback(move |b, _, _| b.set_force(gravity));

            a.into_handle(newton);
            b.into_handle(newton);
            c.into_handle(newton);
        }

        Self
    }
}

fn main() {
    testbed::run::<Jenga>()
}
