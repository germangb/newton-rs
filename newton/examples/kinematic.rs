use newton::types::Cgmath;

use newton::body::{self, Body, Type};
use newton::collision::{self, Collision};
use newton::world::{self, Filter, World};

use newton::sandbox::{Input, Sandbox};

use cgmath::{prelude::*, vec3, Deg, Matrix4, Vector3};
use newton::world::NewtonWorld;
use std::time::Duration;

struct Scene {
    bodies: Vec<Body<Cgmath>>,
    /// kinematic body
    agent: Body<Cgmath>,
    /// collision detection primitive
    collision: Collision<Cgmath>,
}

fn main() {
    let world: World<Cgmath> = world::create();
    let Scene {
        bodies,
        agent,
        collision,
    } = init_scene(&world);

    for b in world.read().bodies() {
        println!("{:?}", b)
    }
    for b in world.write().bodies() {
        println!("{:?}", b)
    }

    for _ in 0..60 * 120 {
        //world.write().update(Duration::new(0, 1_000_000_000 / 60));
    }

    let mut sandbox = Sandbox::default(world.write().as_raw());

    agent.write().set_matrix(&pos(6.0, 2.0, 6.0));

    sandbox.set_handler(move |input| {
        let mut position = agent.read().position();
        let matrix = agent.read().matrix();

        let mut dp = Vector3::new(0.0, 0.0, 0.0);

        let look = Vector3::new(input.look.x, 0.0, input.look.z);
        let left = Vector3::new(look.z, 0.0, -look.x);
        let up = Vector3::new(0.0, 1.0, 0.0);

        if input.w {
            dp -= look;
        }
        if input.s {
            dp += look;
        }
        if input.a {
            dp -= left;
        }
        if input.d {
            dp += left;
        }
        if input.space {
            dp += up;
        }
        if input.lshift {
            dp -= up;
        }

        if dp.magnitude() < 0.001 {
            return;
        }

        let mut dp = dp.normalize() * 4.0 / 60.0;
        let mut target = position + dp;

        // convex cast against dynamic bodies
        let collision = collision.read();

        while let Some((_, info)) = world
            .read()
            .convex_cast(&matrix, &target, &collision, 1, |b, _| b.is_dynamic())
            .next()
        {
            dp = (dp - info.normal * cgmath::dot(info.normal, dp)) * 0.5;

            if dp.magnitude() < 0.01 {
                return;
            } else {
                target = position + dp;
            }
        }

        agent.write().set_matrix(&Matrix4::from_translation(target))
    });

    sandbox.run();
}

fn init_scene(w: &World<Cgmath>) -> Scene {
    let mut world = w.write();
    let pool = [
        collision::cuboid(&mut world, 16.0, 0.5, 16.0, 0),
        collision::cuboid(&mut world, 4.0, 4.0, 6.0, 0),
        collision::sphere(&mut world, 1.0, 0),
        collision::cuboid(&mut world, 1.0, 1.0, 1.0, 0),
    ];

    let floor = Body::new(
        &mut world,
        &pool[0].read(),
        Type::Dynamic,
        &Matrix4::identity(),
    );
    let obstacle0 = Body::new(
        &mut world,
        &pool[1].read(),
        Type::Dynamic,
        &(pos(-4.0, 2.0, 4.0) * roty(40.0)),
    );
    let obstacle1 = Body::new(
        &mut world,
        &pool[1].read(),
        Type::Dynamic,
        &(pos(-4.0, 0.0, -2.0) * roty(-30.0)),
    );

    let agent_body = Body::new(
        &mut world,
        &pool[2].read(),
        Type::Kinematic,
        &pos(0.0, 2.0, 0.0),
    );

    // simulated body
    let cube = Body::new(
        &mut world,
        &pool[3].read(),
        Type::Dynamic,
        &(pos(0.0, 4.0, 0.0) * rotx(20.0) * roty(-30.0)),
    );
    drop(world);
    cube.write().set_mass(1.0);
    cube.write()
        .set_force_and_torque(|b, _, _| b.set_force(&vec3(0.0, -9.8, 0.0)));

    agent_body.write().set_collidable(true);

    Scene {
        bodies: vec![floor, obstacle0, obstacle1, cube],
        agent: agent_body,
        collision: pool[2].clone(),
    }
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
