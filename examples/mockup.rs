use newton::Body;
use newton::Collision;
use newton::Newton;

fn main() {
    let world = Newton::default();

    let cube = Collision::builder().build_box(&world, 1.0, 1.0, 1.0);

    let body = Body::builder().build_dynamic(&world, &cube);

    let gravity = [0.0, -9.8, 0.0];
    body.set_force_and_torque_callback(move |b| b.set_force(&gravity));

    newton::sandbox::run(world.as_ptr());
}
