use newton::World;
use newton::Collision;
use newton::collision::params::HeightFieldParams;

fn main() {
    let world = World::<(), ()>::builder().build();

    let params = HeightFieldParams::<f32>::new(16, 16);

    let collision = Collision::builder(&mut world.write()).heightfield_f32(params);
}