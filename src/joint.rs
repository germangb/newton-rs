//!
//! - ***This module is a Work in progress (all modules are, but this one specially)***
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::time::Duration;

use crate::body::{Body, NewtonBody};
use crate::ffi;
use crate::newton::Newton;
use crate::Vec3;
use crate::{AsHandle, Handle, IntoHandle};

/// Joint iterators
pub mod iter;

#[repr(i32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum CollisionState {
    NonCollidable = 0,
    Collidable = 1,
}

struct UserData {
    /// I need a pointer to the NewtonWorld in order to destroy the collision...
    world: *const ffi::NewtonWorld,
    /// Human readable name (useful for debugging)
    name: Option<&'static str>,

    /// Ball joint callback
    ball_callback: Option<Box<dyn FnMut(Ball, Duration)>>,
    ///// Slider joint callback
    //slider_callback: Option<Box<dyn FnMut(Slider, Duration)>>,
    /// Joint destructor callback.
    /// Common to all joint types.
    destroy_callback: Option<Box<dyn FnMut()>>,
}

macro_rules! joints {
    ($(
        {
            $( #[ $($meta:meta)+ ] )*
            struct $joint:ident
        }
    )*) => {
        $(
            $( #[ $($meta)+ ] )*
            pub struct $joint<'a> {
                raw: *const ffi::NewtonJoint,
                owned: bool,
                _phantom: PhantomData<&'a ()>,
            }

            impl<'a> NewtonJoint for $joint<'a> {
                fn as_raw(&self) -> *const ffi::NewtonJoint {
                    self.raw
                }
            }

            impl<'a> Drop for $joint<'a> {
                fn drop(&mut self) {
                    if self.owned {
                        let world = unsafe {
                            let udata = ffi::NewtonJointGetUserData(self.raw);
                            let udata: &Box<UserData> = mem::transmute(&udata);
                            udata.world
                        };
                        unsafe {
                            ffi::NewtonDestroyJoint(world, self.raw);
                        }
                    }
                }
            }

            impl<'a> $joint<'a> {
                pub unsafe fn from_raw(raw: *const ffi::NewtonJoint, owned: bool) -> Self {
                    Self {
                        raw,
                        owned,
                        _phantom: PhantomData,
                    }
                }
            }

            impl<'a> AsHandle for $joint<'a> {
                fn as_handle(&self, _: &Newton) -> Handle {
                    Handle::from_ptr(self.raw as _)
                }
            }
            impl<'a> IntoHandle for $joint<'a> {
                fn into_handle(mut self, newton: &Newton) -> Handle {
                    self.owned = false;
                    //newton.storage().move_constraint(newton);
                    Handle::from_ptr(self.raw as _)
                }
            }
        )*

        /// Enum grouping all Newton joints
        #[derive(Debug)]
        pub enum Constraint<'a> {
            $( $joint($joint<'a>) ),*
        }

        impl<'a> Constraint<'a> {
            pub unsafe fn from_raw(raw: *const ffi::NewtonJoint, owned: bool) -> Self {
                Constraint::Ball(Ball::from_raw(raw, owned))
            }
        }

        impl<'a> NewtonJoint for Constraint<'a> {
            fn as_raw(&self) -> *const ffi::NewtonJoint {
                match self {
                    $(Constraint::$joint(ref joint) => joint.as_raw() ),*
                }
            }
        }
    }
}

joints! {
    {
        #[derive(Debug)]
        struct Ball
    }
    {
        #[derive(Debug)]
        struct Slider
    }
    {
        #[derive(Debug)]
        struct UpVector
    }
    {
        #[derive(Debug)]
        struct Hinge
    }
    {
        #[derive(Debug)]
        struct Corkscrew
    }
    {
        #[derive(Debug)]
        struct Universal
    }
    {
        #[derive(Debug)]
        struct UserJoint
    }
}

impl<'a> Ball<'a> {
    pub fn set_callback<F>(&self, callback: F)
        where F: FnMut(Ball, Duration) + 'static
    {
        unsafe {
            let udata = ffi::NewtonJointGetUserData(self.raw);
            let mut udata: Box<UserData> = Box::from_raw(udata as _);

            udata.ball_callback = Some(Box::new(callback));
            ffi::NewtonBallSetUserCallback(self.raw, Some(c_callback));
        }

        unsafe extern "C" fn c_callback(joint: *const ffi::NewtonJoint, timestep: f32) {
            let udata = ffi::NewtonJointGetUserData(joint);
            let mut udata: Box<UserData> = Box::from_raw(udata as _);

            let seconds = timestep.floor() as u64;
            let nanos = (timestep.fract() * 1_000_000_000.0) as u32;
            let timestep = Duration::new(seconds, nanos);

            if let Some(callback) = &mut udata.ball_callback {
                callback(Ball::from_raw(joint, false), timestep);
            }
        }
    }

    pub fn create<A, B>(newton: &Newton,
                        pivot: Vec3,
                        child: &'a A,
                        parent: Option<&'a B>,
                        name: Option<&'static str>)
                        -> Self
        where A: NewtonBody,
              B: NewtonBody
    {
        unsafe {
            let world = newton.as_raw();
            let child = child.as_raw();
            let parent = parent.map(|b| b.as_raw()).unwrap_or(ptr::null());
            let raw = ffi::NewtonConstraintCreateBall(world, pivot.as_ptr(), child, parent);
            let udata = UserData { name,
                                   world: newton.as_raw(),
                                   ball_callback: None,
                                   destroy_callback: None };

            ffi::NewtonJointSetDestructor(raw, Some(joint_destroy));
            ffi::NewtonJointSetUserData(raw, mem::transmute(Box::new(udata)));
            Self::from_raw(raw, true)
        }
    }

    pub fn set_cone_limits(&self, pin: Vec3, max_cone_angle: f32, max_twist_angle: f32) {
        unsafe {
            ffi::NewtonBallSetConeLimits(self.raw, pin.as_ptr(), max_cone_angle, max_twist_angle)
        };
    }

    pub fn joint_angle(&self) -> Vec3 {
        let mut angles = [0.0, 0.0, 0.0];
        unsafe { ffi::NewtonBallGetJointAngle(self.raw, angles.as_mut_ptr()) }
        angles
    }

    pub fn joint_force(&self) -> Vec3 {
        let mut force = [0.0, 0.0, 0.0];
        unsafe { ffi::NewtonBallGetJointForce(self.raw, force.as_mut_ptr()) }
        force
    }

    pub fn joint_omega(&self) -> Vec3 {
        let mut omega = [0.0, 0.0, 0.0];
        unsafe { ffi::NewtonBallGetJointOmega(self.raw, omega.as_mut_ptr()) }
        omega
    }
}

impl<'a> Slider<'a> {
    pub fn create<A, B>(newton: &Newton,
                        pivot: Vec3,
                        pin_dir: Vec3,
                        child: &'a A,
                        parent: Option<&'a B>,
                        name: Option<&'static str>)
                        -> Self
        where A: NewtonBody,
              B: NewtonBody
    {
        unsafe {
            let world = newton.as_raw();
            let child = child.as_raw();
            let parent = parent.map(|b| b.as_raw()).unwrap_or(ptr::null());
            let raw = ffi::NewtonConstraintCreateSlider(world,
                                                        pivot.as_ptr(),
                                                        pin_dir.as_ptr(),
                                                        child,
                                                        parent);
            let udata = UserData { name,
                                   world: newton.as_raw(),
                                   ball_callback: None,
                                   destroy_callback: None };

            ffi::NewtonJointSetDestructor(raw, Some(joint_destroy));
            ffi::NewtonJointSetUserData(raw, mem::transmute(Box::new(udata)));
            Self::from_raw(raw, true)
        }
    }
}

unsafe extern "C" fn joint_destroy(me: *const ffi::NewtonJoint) {
    let udata = ffi::NewtonJointGetUserData(me);
    let mut udata: Box<UserData> = Box::from_raw(udata as _);
    if let Some(mut destroy) = udata.destroy_callback.take() {
        destroy()
    }
}

/// NewtonJoint API functions.
pub trait NewtonJoint {
    fn as_raw(&self) -> *const ffi::NewtonJoint;

    fn into_raw(self) -> *const ffi::NewtonJoint
        where Self: Sized
    {
        self.as_raw()
    }

    fn name(&self) -> Option<&'static str> {
        unsafe {
            let udata = ffi::NewtonJointGetUserData(self.as_raw());
            let udata: &Box<UserData> = mem::transmute(&udata);
            udata.name
        }
    }

    fn bodies(&self) -> (Body, Option<Body>) {
        unsafe {
            let body0 = ffi::NewtonJointGetBody0(self.as_raw());
            let body1 = ffi::NewtonJointGetBody1(self.as_raw());

            let body0 = Body::from_raw(body0, false);
            let body1 = if body1.is_null() { None } else { Some(Body::from_raw(body1, false)) };

            (body0, body1)
        }
    }

    fn stiffness(&self) -> f32 {
        unsafe { ffi::NewtonJointGetStiffness(self.as_raw()) }
    }

    fn set_stiffness(&self, stiff: f32) {
        unsafe {
            ffi::NewtonJointSetStiffness(self.as_raw(), stiff);
        }
    }

    fn set_destroy_callback<F>(&self, callback: F)
        where F: FnMut() + 'static
    {
        unsafe {
            let udata = &mut ffi::NewtonJointGetUserData(self.as_raw());
            let mut udata: &mut Box<UserData> = mem::transmute(udata);

            udata.destroy_callback = Some(Box::new(callback));
        }
    }
}
