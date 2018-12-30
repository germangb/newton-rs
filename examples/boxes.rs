use newton::Cgmath;

use newton::body::{self, Body, Type};
use newton::collision::{self, params::Params, Collision};
use newton::world::World;

use newton::sandbox::{self, Sandbox};

use cgmath::{prelude::*, vec3, Matrix4, Vector3};
use std::time::Duration;
use newton::body::SleepState;
use newton::collision::NewtonCollision;
use newton::world::NewtonWorld;
use newton::collision::params::HeightFieldParams;

fn controller(world: World<Cgmath>, sandbox: &mut Sandbox) {
    let sphere = world.write().collision().sphere(1.0).debug("sphere_col").build();
    let agent = world.write().body(&sphere.read()).kinematic().transform(pos(8.0, 2.0, 8.0)).build();
    agent.write().set_collidable(true);

    sandbox.set_handler(move |input| {
        let position = agent.read().position();
        let matrix = agent.read().matrix();

        let mut dp = vec3(0.0, 0.0, 0.0);

        let look = vec3(input.look.x, 0.0, input.look.z);
        let left = vec3(look.z, 0.0, -look.x);
        let up = vec3(0.0, 1.0, 0.0);

        if input.w { dp -= look; }
        if input.s { dp += look; }
        if input.a { dp -= left; }
        if input.d { dp += left; }
        if input.space { dp += up; }
        if input.lshift { dp -= up; }
        if dp.magnitude() < 0.001 {
            agent.write().set_sleep_state(SleepState::Sleeping);
            return;
        }

        let mut dp = dp.normalize() * 4.0 / 60.0;
        let mut target = position + dp;
        let sphere = sphere.read();
        while let Some((_, info)) = world
            .read()
            .convex_cast(&matrix, &target, &sphere, 1, |b, _| b.is_dynamic()).next()
            {
                dp = (dp - info.normal * cgmath::dot(info.normal, dp)) * 0.5;

                if dp.magnitude() < 0.01 {
                    return;
                } else {
                    target = position + dp;
                }
            }

        agent.write().set_matrix(&Matrix4::from_translation(target))
    })
}

fn main() {
    let world: World<Cgmath> = World::builder()
        .debug("world_0")
        .threads(1)
        .exact_solver()
        .build();

    // collision pool
    let pool = {
        let mut world = world.write();
        [
            world.collision().cuboid(24.0, 1.0, 24.0).build(),
            world.collision().cuboid(1.0, 1.0, 1.0).build(),
        ]
    };

    let floor = world
        .write()
        .body(&pool[0].read())
        .transform(Matrix4::identity())
        .build();

    let transforms = (0..16 * 4)
        .map(|i| (i % 16 / 4, i / 16, i % 16 % 4))
        .map(|(x, y, z)| vec3(x as f32, y as f32, z as f32))
        .map(|p| Matrix4::from_translation(p - vec3(1.5, -4.0, 1.5)));

    let cubes: Vec<_> = transforms
        .map(|p| {
            let cube = world.write().body(&pool[1].read()).transform(p).build();
            let g = vec3(0.0, -9.8, 0.0);

            cube.write().set_mass(1.0);
            cube.write()
                .set_force_and_torque(move |b, _, _| b.set_force(&g));
            cube
        })
        .collect();

    for _ in world.read().bodies() {}

    let ptr = world.read().as_raw();
    let mut sandbox = Sandbox::default(ptr);

    controller(world.clone(), &mut sandbox);

    // heightfield
    let mut params = HeightFieldParams::<f32>::new(32, 32);
    let collision = collision::Builder::new(&mut world.write()).heightfield_f32(params).offset(pos(-16.0, 0.0, -16.0)).build();
    let terrain = body::Builder::new(&mut world.write(), &collision.read()).dynamic().transform(pos(0.0, 0.0, 0.0)).build();

    world.write().update(Duration::new(0, 1000000000 / 60));
    sandbox.run();
}

fn pos(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}
