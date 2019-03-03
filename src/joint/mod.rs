//!
//! - ***This module is a Work in progress (all modules are, but this one specially)***
//! - Official wiki docs: [**official wiki**][wiki].
//!
//! [wiki]: http://www.newtondynamics.com/wiki/index.php5?title=Category:Joint_functions
//!
use std::marker::PhantomData;
use std::mem;
use std::ptr;
use std::time::Duration;

use crate::body::{Body, NewtonBody};
use crate::ffi;
use crate::handle::{AsHandle, Handle, IntoHandle};
use crate::math::Vec3;
use crate::newton::Newton;

pub mod iter;

/// Collision state between two joined bodies. Either collidable or non-collidable.
#[repr(i32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum CollisionState {
    /// Collision is disabled between the two joined bodies.
    NonCollidable = 0,
    /// The two connected bodies can collide with each other.
    Collidable = 1,
}

struct UserData {
    // The concrete type is stored in order to build a Constraint from the raw pointer...
    // I suppose newton joints all derive from a universal type and share the same data structure.
    joint_type: Type,

    // I need a pointer to the NewtonWorld in order to destroy the collision.
    // Newton probably keeps some internal data structure for the constraints, but I haven't checked the implementation.
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

joints! {
    /// The movement and rotation of a body is limited to a given solid angle,
    /// and angle respectively.
    #[derive(Debug)]
    struct Ball
    fn ball, is_ball

    /// Constraints body movement to a single axis (car damper shaft).
    #[derive(Debug)]
    struct Slider
    fn slider, is_slider

    /// Limits rotation of a body to a single axis.
    #[derive(Debug)]
    struct UpVector
    fn up_vector, is_up_vector

    /// Allows rotation movement perpendicular to a given axis. (door , propeller, etc..)
    #[derive(Debug)]
    struct Hinge
    fn hinge, is_hinge

    /// Limits body movement to a single axis, and rotation perpendicular to the same.
    #[derive(Debug)]
    struct Corkscrew
    fn corkscrew, is_corkscrew

    /// Allows a body to rotate on two axis (like a hinge joint with an added *DoF*).
    #[derive(Debug)]
    struct Universal
    fn universal, is_universal

    /// Generic user-defined joint (for advanced and very specific cases).
    #[derive(Debug)]
    struct UserJoint
    fn user_joint, is_user_joint
}

impl<'a> Ball<'a> {
    pub fn set_callback<F>(&self, callback: F)
        where F: FnMut(Ball, Duration) + Send + 'static
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

    pub fn create<'b, 'c, 'd, B, C>(newton: &'d Newton,
                                    pivot: Vec3,
                                    child: &'b B,
                                    parent: Option<&'c C>,
                                    name: Option<&'static str>)
                                    -> Self
        where 'b: 'a,
              'c: 'a,
              'd: 'b + 'c,
              B: NewtonBody,
              C: NewtonBody
    {
        unsafe {
            let world = newton.as_raw();
            let child = child.as_raw();
            let parent = parent.map(|b| b.as_raw()).unwrap_or(ptr::null());
            let raw = ffi::NewtonConstraintCreateBall(world, pivot.as_ptr(), child, parent);
            let udata = UserData { name,
                                   joint_type: Type::Ball,
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
    // Example playground with lifetimes and lifetime bounds:
    // https://play.rust-lang.org/?version=stable&mode=debug&edition=2018&gist=e1ec143dee1f116fa106ff7f6bfbb3de
    pub fn create<'b, 'c, 'd, B, C>(newton: &'d Newton,
                                    pivot: Vec3,
                                    pin_dir: Vec3,
                                    child: &'b B,
                                    parent: Option<&'c C>,
                                    name: Option<&'static str>)
                                    -> Self
        where 'b: 'a,
              'c: 'a,
              'd: 'b + 'c,
              B: NewtonBody,
              C: NewtonBody
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
                                   joint_type: Type::Slider,
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

    /// Returns the pair of bodies that are linked by this joint
    fn bodies<'a: 'b, 'b>(&'a self) -> (Body<'b>, Option<Body<'b>>) {
        unsafe {
            let body0 = ffi::NewtonJointGetBody0(self.as_raw());
            let body1 = ffi::NewtonJointGetBody1(self.as_raw());

            let body0 = Body::from_raw(body0, false);
            let body1 = if body1.is_null() { None } else { Some(Body::from_raw(body1, false)) };

            (body0, body1)
        }
    }

    fn collision_state(&self) -> CollisionState {
        unsafe { mem::transmute(ffi::NewtonJointGetCollisionState(self.as_raw())) }
    }

    fn set_collision_state(&self, state: CollisionState) {
        unsafe {
            let state = mem::transmute(state);
            ffi::NewtonJointSetCollisionState(self.as_raw(), state);
        }
    }

    fn is_active(&self) -> bool {
        unsafe { ffi::NewtonJointIsActive(self.as_raw()) == 1 }
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
        where F: FnMut() + Send + 'static
    {
        unsafe {
            let udata = &mut ffi::NewtonJointGetUserData(self.as_raw());
            let mut udata: &mut Box<UserData> = mem::transmute(udata);

            udata.destroy_callback = Some(Box::new(callback));
        }
    }
}
