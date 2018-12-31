use newton::Cgmath;

use newton::body::{self, Body, Type};
use newton::collision::{self, params::Params, Collision};
use newton::material::GroupId;
use newton::world::{self, World};

use newton::sandbox::{self, Sandbox};

use cgmath::{prelude::*, vec3, Matrix4, Vector3};
use newton::body::SleepState;
use newton::collision::params::HeightFieldParams;
use newton::collision::NewtonCollision;
use newton::world::NewtonWorld;
use std::time::Duration;

fn controller(world: World<Cgmath>, sandbox: &mut Sandbox) {
    let sphere = collision::Builder::new(&mut world.write())
        .capsule(1.0, 1.0, 2.0)
        .cuboid(1.0, 1.0, 1.0)
        .debug("sphere_col")
        .build();

    let mut faces = 0;
    let id = Matrix4::identity();
    sphere.read().polygons(&id, |_, face| {
        println!("face: {:?}", face.chunks(3).collect::<Vec<_>>());
        faces += 1;
    });

    let agent = body::Builder::new(&mut world.write(), &sphere.read())
        .kinematic()
        .transform(pos(8.0, 2.0, 8.0))
        .build();

    world.read().ray_cast(
        &vec3(0.0, 1.0, 0.0),
        &vec3(0.0, 16.0, 0.0),
        |_, _, _, _, _, _| 1.0,
        |_, _| true,
    );

    sandbox.set_handler(move |input| {
        let position = agent.read().position();
        let matrix = agent.read().matrix();

        let mut dp = vec3(0.0, 0.0, 0.0);

        let look = vec3(input.look.x, 0.0, input.look.z);
        let left = vec3(look.z, 0.0, -look.x);
        let up = vec3(0.0, 1.0, 0.0);

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
            agent.write().set_sleep_state(SleepState::Sleeping);
            return;
        }

        let mut dp = dp.normalize() * 4.0 / 60.0;
        let mut target = position + dp;

        let mut param = 0.0;
        if let Some((_, info)) = world
            .read()
            .convex_cast(&matrix, &target, &sphere.read(), &mut param, 1, |b, _| {
                b.is_dynamic()
            })
            .next()
        {
            //println!("{}", param);
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
    let world: World<Cgmath> = world::Builder::new()
        .force_torque_callback(|b, _, _| b.set_force(&vec3(0.0, -9.8, 0.0)))
        .build();

    let (_a, _b, _c): (GroupId, GroupId, GroupId) = world.write().create_materials();

    // collision pool
    let pool = {
        let mut world = world.write();
        [
            collision::Builder::new(&mut world)
                .cuboid(24.0, 1.0, 24.0)
                .build(),
            collision::Builder::new(&mut world)
                .cuboid(0.75, 0.75, 0.75)
                .build(),
        ]
    };

    let floor = body::Builder::new(&mut world.write(), &pool[0].read())
        .transform(Matrix4::identity())
        .build();
    drop(floor);

    let transforms = (0..16 * 8)
        .map(|i| (i % 16 / 4, i / 16, i % 16 % 4))
        .map(|(x, y, z)| vec3(x as f32, y as f32, z as f32))
        .map(|p| Matrix4::from_translation(p - vec3(1.5, -3.0, 1.5)));

    let cubes: Vec<_> = transforms
        .map(|p| {
            let cube = body::Builder::new(&mut world.write(), &pool[1].read())
                .transform(p)
                .build();
            let g = vec3(0.0, -9.8, 0.0);

            cube.write().set_mass(1.0);
            cube.write().apply_force_and_torque();
            cube
        })
        .collect();

    for _ in world.read().bodies() {}

    let ptr = world.read().as_raw();
    let mut sandbox = Sandbox::default(ptr);

    controller(world.clone(), &mut sandbox);

    // heightfield
    let mut params = HeightFieldParams::<f32>::new(32, 32);
    let collision = collision::Builder::new(&mut world.write())
        .heightfield_f32(params)
        .offset(pos(-16.0, 0.0, -16.0))
        .build();
    let terrain = body::Builder::new(&mut world.write(), &collision.read())
        .dynamic()
        .transform(pos(0.0, 0.0, 0.0))
        .build();

    world.write().update(Duration::new(0, 1000000000 / 60));
    sandbox.run();
}

fn pos(x: f32, y: f32, z: f32) -> Matrix4<f32> {
    Matrix4::from_translation(vec3(x, y, z))
}
