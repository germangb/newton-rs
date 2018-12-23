pub use crate::body::Body;
use crate::ffi;
use crate::pointer::*;
pub use crate::world::World;
use crate::NewtonApp;

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

    #[derive(Debug, Clone)]
    pub struct HingeJoint<C>;

    #[derive(Debug, Clone)]
    pub struct CrokscrewJoint<C>;

    #[derive(Debug, Clone)]
    pub struct SliderJoint<C>;
}

impl<C: NewtonApp> BallJoint<C> {
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
