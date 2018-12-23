pub use crate::body::Body;
use crate::ffi;
use crate::pointer::*;
pub use crate::world::World;
use crate::NewtonConfig;

use std::mem;
use std::rc::Rc;

macro_rules! constraints {
    ($(
        $(#[$($meta:meta)+])*
        pub struct $struct_ident:ident<C>;
    )+) => {$(
        $(#[$($meta)+])*
        pub struct $struct_ident<C> {
            pub(crate) joint: Rc<NewtonJointPtr<C>>,
            pub(crate) raw: *mut ffi::NewtonJoint,
        }
    )+}
}

constraints! {
    #[derive(Debug, Clone)]
    pub struct BallJoint<C>;
}

impl<C: NewtonConfig> BallJoint<C> {
    pub fn new(child: &Body<C>, parent: &Body<C>, pivot: C::Vector) -> Self {
        unsafe {
            let raw = ffi::NewtonConstraintCreateBall(
                (child.body.1).0,
                mem::transmute(&pivot),
                child.raw,
                parent.raw,
            );
            BallJoint {
                joint: Rc::new(NewtonJointPtr(
                    raw,
                    Rc::downgrade(&child.body),
                    Rc::downgrade(&parent.body),
                )),
                raw,
            }
        }
    }
}
