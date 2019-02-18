use std::error::Error;
use std::time::Duration;

use newton::testbed::{Demo, Testbed};
use newton::{body, Body, Collision, Matrix, Newton, Vector};

struct Example;

impl Demo for Example {
    fn reset(newton: &Newton) -> Self {
        let floor = Collision::cuboid(newton, 8.0, 0.5, 8.0, None);
        let floor =
            Body::dynamic(newton, &floor, &Matrix::identity(), Some("floor_body")).into_handle();
        let cuboid = Collision::cuboid(newton, 1.0, 1.0, 1.0, None);
        let sphere = Collision::sphere(newton, 0.5, None);
        let cylinder = Collision::capsule(newton, 0.1, 0.5, 1.0, None);

        let mut trans = Matrix::identity();
        trans.c3.y = 6.0;
        let body0 = Body::dynamic(newton, &sphere, &trans, Some("sphere"));
        trans.c3.y = 8.0;
        trans.c3.x = 0.25;
        trans.c3.z = 0.1;
        let body1 = Body::dynamic(newton, &cuboid, &trans, Some("box_0"));

        trans.c3.y = 9.25;
        trans.c3.x = 0.0;
        trans.c3.z = 0.0;
        let body2 = Body::dynamic(newton, &cylinder, &trans, Some("box_1"));

        body0.set_force_and_torque_callback(|b, _| b.set_force(&Vector::new3(0.0, -9.8, 0.0)));
        body1.set_force_and_torque_callback(|b, _| b.set_force(&Vector::new3(0.0, -9.8, 0.0)));
        body2.set_force_and_torque_callback(|b, _| b.set_force(&Vector::new3(0.0, -9.8, 0.0)));

        body0.set_mass(1.0, &sphere);
        body1.set_mass(1.0, &cuboid);
        body2.set_mass(1.0, &cylinder);

        body0.into_handle();
        body1.into_handle();
        body2.into_handle();

        Self
    }
}

fn main() {
    Testbed::<Example>::run().unwrap();
}
