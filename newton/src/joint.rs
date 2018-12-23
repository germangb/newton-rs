use ffi;

use crate::body::Body;
use crate::pointer::*;
use crate::world::World;
use crate::NewtonApp;

use std::mem;
use std::rc::{Rc, Weak};

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
    pub struct CorkscrewJoint<C>;

    #[derive(Debug, Clone)]
    pub struct SliderJoint<C>;
}

macro_rules! joint_method {
    (fn new( $($vec:ident),+ ) -> ffi:: $ffi:ident) => {
        pub fn new(child: &Body<C>, parent: &Body<C>, $($vec: C::Vector),+ ) -> Self {
            unsafe {
                let raw = ffi::$ffi(
                    (child.body.1).0,
                    $(mem::transmute(&$vec),)+
                    child.raw,
                    parent.raw,
                );
                Self {
                    joint: Rc::new(NewtonJointPtr(
                        raw,
                        Rc::downgrade(&child.body),
                        Rc::downgrade(&parent.body),
                    )),
                    raw,
                }
            }
        }
        fn valid(&self) -> bool {
            let parent = Weak::upgrade(&self.joint.1);
            let child = Weak::upgrade(&self.joint.2);
            parent.and(child).is_some()
        }
    };
    (
        $(#[$($meta:meta)+])*
        euler fn $ident:ident(&self) -> ffi:: $ffi:ident
    ) => {
        $(#[$($meta)+])*
        ///
        /// Returns `None` If either of the two bodies has been dropped
        pub fn $ident(&self) -> Option<(f32, f32, f32)> {
            if !self.valid() { return None; }
            unsafe {
                let mut euler = [0.0, 0.0, 0.0];
                ffi::$ffi(self.raw, euler.as_mut_ptr());
                Some((euler[0], euler[1], euler[2]))
            }
        }
    };

    (
        $(#[$($meta:meta)+])*
        fn $ident:ident(&self) -> ffi:: $ffi:ident
    ) => {
        $(#[$($meta)+])*
        ///
        /// Returns `None` If either of the two bodies has been dropped
        pub fn $ident(&self) -> Option<f32> {
            if !self.valid() { return None; }
            unsafe {
                Some(ffi::$ffi(self.raw))
            }
        }
    };
}

impl<C: NewtonApp> BallJoint<C> {
    joint_method!(fn new(pivot) -> ffi::NewtonConstraintCreateBall);
    joint_method!(euler fn angle(&self) -> ffi::NewtonBallGetJointAngle);
    joint_method!(euler fn omega(&self) -> ffi::NewtonBallGetJointOmega);
    joint_method!(euler fn force(&self) -> ffi::NewtonBallGetJointForce);
}

impl<C: NewtonApp> HingeJoint<C> {
    joint_method!(fn new(pivot, pin_dir) -> ffi::NewtonConstraintCreateHinge);
    joint_method!(fn angle(&self) -> ffi::NewtonHingeGetJointAngle);
    joint_method!(fn omega(&self) -> ffi::NewtonHingeGetJointOmega);
    joint_method!(euler fn force(&self) -> ffi::NewtonHingeGetJointForce);
}

impl<C: NewtonApp> CorkscrewJoint<C> {
    joint_method!(fn new(pivot, pin_dir) -> ffi::NewtonConstraintCreateCorkscrew);
    joint_method!(fn angle(&self) -> ffi::NewtonCorkscrewGetJointAngle);
    joint_method!(fn omega(&self) -> ffi::NewtonCorkscrewGetJointOmega);
    joint_method!(euler fn force(&self) -> ffi::NewtonCorkscrewGetJointForce);
}

impl<C: NewtonApp> SliderJoint<C> {
    joint_method!(fn new(pivot, pin_dir) -> ffi::NewtonConstraintCreateSlider);
    joint_method!(euler fn force(&self) -> ffi::NewtonSliderGetJointForce);
}
