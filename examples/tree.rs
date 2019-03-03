use newton::prelude::*;
use newton::{Cuboid, DynamicBody, Newton, Tree};

use newton::testbed::{run, Testbed};

struct TreeDemo;
impl Testbed for TreeDemo {
    fn reset(newton: &Newton) -> Self {
        let floor = Cuboid::create(newton, 8.0, 0.2, 8.0, None);
        let mut tree = Tree::create(newton);
        {
            let build = tree.begin_build();
            let face = &[[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
            build.add(face, 0);
            build.optimize();
        }

        DynamicBody::create(newton, &floor, trans(0.0, 0.0, 0.0), None).into_handle(newton);

        let body = DynamicBody::create(newton, &tree, trans(0.0, 4.0, 0.0), None);
        body.set_mass(1.0, &tree);
        body.set_force_and_torque_callback(|b, _, _| b.set_force([0.0, -9.8, 0.0]));
        body.into_handle(newton);

        Self
    }
}

fn main() {
    run::<TreeDemo>(Some(file!()));
}

const fn trans(x: f32, y: f32, z: f32) -> [[f32; 4]; 4] {
    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [x, y, z, 1.0]]
}
