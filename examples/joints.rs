use newton::prelude::*;
use newton::{Ball, DynamicBody, Newton, Sphere};

use newton::testbed::{run, Testbed};

struct Joints;
impl Testbed for Joints {
    fn reset(newton: &mut Newton) -> Self {
        let sphere = Sphere::create(newton, 0.5, None);

        let mut last_body = DynamicBody::create(newton,
                                                &sphere,
                                                pos(0.0, 0.0, 0.0),
                                                Some("ground")).into_handle(newton);

        for (offset, pivot) in (1..8).map(|i| (0.5 * i as f32, 0.5 * (i - 1) as f32)) {
            let parent = newton.storage().body(last_body).unwrap();

            let ball = DynamicBody::create(newton, &sphere, pos(offset, offset, 0.0), Some("ball"));
            let joint = Ball::create(newton, [pivot, pivot, 0.0], &ball, Some(&parent), None);

            joint.set_stiffness(10.0);
            joint.set_destroy_callback(|| println!("Destroy joint"));
            joint.into_handle(&newton);

            ball.set_mass(1.0, &sphere);
            ball.set_destroy_callback(|_| println!("Destroy ball"));
            ball.set_force_and_torque_callback(|b, _, _| b.set_force([0.0, -9.8, 0.0]));

            last_body = ball.into_handle(newton);
        }

        Self
    }
}

fn main() {
    run::<Joints>(Some(file!()));
}

const fn pos(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [x, y, z, 1.0]]
}
