use newton::{Body, Collision, World};

fn main() {
    let world = World::<(), ()>::builder().build();

    let identity = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    let collision = Collision::builder(&mut world.write()).sphere(1.0);
    let body = Body::builder(&mut world.write(), &collision.read()).kinematic(&identity);

    collision
        .read()
        .polygons(&identity, |_, face| println!("{:?}", face));

    drop(collision);
}
