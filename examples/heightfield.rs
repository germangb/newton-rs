use newton::heightfield::HeightField;
use newton::Collision;
use newton::World;

fn main() {
    let world = World::<(), ()>::builder().build();

    let params = HeightField::<f32>::new(16, 16);

    let collision = Collision::builder(&mut world.write()).heightfield_f32(params);
}
