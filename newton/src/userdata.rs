use crate::pointer::*;

use std::rc::Weak;

pub struct BodyUserData<C> {
    pub(crate) body: Weak<NewtonBodyPtr<C>>,
    pub(crate) collision: Weak<NewtonCollisionPtr<C>>,
}
