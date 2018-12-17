use std::slice::Iter;

use newton_dynamics::body::NewtonBody;
use newton_dynamics::traits::NewtonMath;

pub trait Example<V: NewtonMath> {
    fn bodies(&self) -> Iter<NewtonBody<V>>;
}
