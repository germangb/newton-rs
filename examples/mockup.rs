use newton::body::BodyHandle;
use newton::Body;
use newton::Collision;
use newton::Newton;

fn main() {
    let world = Newton::new();

    let body = create_body(&world);
}

fn create_body(world: &Newton) -> BodyHandle {
    let collision = Collision::sphere(&world, 1.0, None);
    let body = Body::dynamic(world, &collision, &newton::math::identity());

    let gravity = [0.0, -9.8, 0.0];
    body.set_force_and_torque_callback(move |b| b.set_force(&gravity));
    body.set_mass(1.0, &collision);

    body.into_handle()
}
