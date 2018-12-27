use newton::types::Cgmath;

use newton::body::{self, Body, BodyType};
use newton::collision::{self, Collision};
use newton::world::{self, Filter, World};

use newton::sandbox::{Input, Sandbox};

use cgmath::{prelude::*, Deg, Matrix4, Vector3};

fn main() {
    let world: World<Cgmath> = world::create();
    let (_floor, _obstacle, (agent, collision)) = init(&world);

    let mut sandbox = Sandbox::default(world.write().as_raw());

    agent
        .write()
        .set_matrix(&Matrix4::from_translation(Vector3::new(6.0, 2.0, 6.0)));

    sandbox.set_handler(
        move |Input { w, a, s, d, look, space, lshift, }| {
            let mut position = agent.read().position();
            let matrix = agent.read().matrix();

            let mut dp = Vector3::new(0.0, 0.0, 0.0);

            let look = Vector3::new(look.x, 0.0, look.z);
            let left = Vector3::new(look.z, 0.0, -look.x);
            let up = Vector3::new(0.0, 1.0, 0.0);

            if w { dp -= look; }
            if s { dp += look; }
            if a { dp -= left; }
            if d { dp += left; }
            if space { dp += up; }
            if lshift { dp -= up; }

            let mut target = position + dp.normalize() * 3.0 / 60.0;

            // convex cast
            let collision = collision.read();
            for (b, info) in world.read().convex_cast(&matrix, &target, &collision, 1, |b, _| b.is_dynamic())
            {
                target += info.normal * info.penetration;
            }

            if dp.magnitude() > 0.001 {
                agent.write().set_matrix(&Matrix4::from_translation(target))
            }
        },
    );

    sandbox.run();
}

fn init(
    world: &World<Cgmath>,
) -> (
    Body<Cgmath>,
    Body<Cgmath>,
    (Body<Cgmath>, Collision<Cgmath>),
) {
    let floor = collision::cuboid(&mut world.write(), 16.0, 0.5, 16.0, 0, None);
    let obstacle = collision::cuboid(&mut world.write(), 4.0, 4.0, 6.0, 0, None);
    let agent = collision::cylinder(&mut world.write(), 0.5, 0.5, 1.5, 0, Some(&rotz(90.0)));
    let agent = collision::sphere(&mut world.write(), 1.0, 0, None);
    let floor = body::dynamic(
        &mut world.write(),
        &floor.read(),
        &Matrix4::identity(),
        None,
    );
    let obstacle = body::dynamic(
        &mut world.write(),
        &obstacle.read(),
        &(pos(-4.0, 2.0, 4.0) * roty(40.0)),
        None,
    );
    let agent_body = body::kinematic(&mut world.write(), &agent.read(), &pos(0.0, 2.0, 0.0), None);
    (floor, obstacle, (agent_body, agent))
}

fn pos(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(Vector3::new(x, y, z))
}

fn roty(x: f32) -> Matrix4<f32> {
    Matrix4::from_angle_y(Deg(x))
}

fn rotx(x: f32) -> Matrix4<f32> {
    Matrix4::from_angle_x(Deg(x))
}

fn rotz(x: f32) -> Matrix4<f32> {
    Matrix4::from_angle_z(Deg(x))
}
