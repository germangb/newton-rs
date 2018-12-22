use crate::callback;
use crate::ffi;
use crate::pointer::*;
use crate::NewtonConfig;

use crate::collision::NewtonCollision;
use crate::collision::{CollisionBox, CollisionCone, CollisionSphere};
use crate::world::NewtonWorld;
use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};
use std::time::Duration;

/// A reference counted body
#[derive(Debug, Clone)]
pub struct NewtonBody<V> {
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,
    pub(crate) body: Rc<NewtonBodyPtr<V>>,

    pub(crate) raw: *mut ffi::NewtonBody,
}

pub(crate) struct UserData<C> {
    pub(crate) body: Weak<NewtonBodyPtr<C>>,
    pub(crate) collision: Weak<NewtonCollisionPtr<C>>,
}

/// Indicates if the body is in a Sleeping or Awake state
#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    Sleeping = 1,
}

/// Type returned by the `mass` with info on the mass and moments of inertia of a given body
#[derive(Debug, Copy, Clone)]
pub struct Mass {
    /// Body mass
    pub mass: f32,
    /// Moment of inertia of the principal components of the I matrix
    pub inertia: (f32, f32, f32),
}

macro_rules! match_rule {
    // used in `new` method
    (
        ( $collision:ident , $world:ident, $matrix:ident ) => $( $var:ident ),+
    ) => {
        match &$collision {
            $(
                &NewtonCollision::$var(ref b) => (
                    ffi::NewtonCreateDynamicBody($world.raw, b.raw, mem::transmute(&$matrix)),
                    b.collision.clone(),
                ),
            )+
            _ => unimplemented!(),
        }
    };
    // used in `collision` method
    (
        ($selfi:ident, $raw:expr) =>
        $( $enumi:ident -> $structi:tt ,)+
    ) => {{
        match ffi::NewtonCollisionGetType($raw) {
            $(
                $structi ::<C>::TYPE_ID => NewtonCollision:: $enumi ($structi {
                    collision: $selfi.collision.clone(),
                    raw: $raw,
                }),
            )+
            _ => unreachable!(),
        }
    }}
}

// XXX pointers
impl<C> NewtonBody<C>
where
    C: NewtonConfig,
{
    pub fn new<N>(world: &NewtonWorld<C>, collision: N, matrix: C::Matrix4) -> Self
    where
        N: Into<NewtonCollision<C>>,
    {
        unsafe {
            let collision = collision.into();
            let (raw, collision) = match_rule! { (collision, world, matrix) =>
                Box,
                Sphere,
                Cone
            };

            let body = Rc::new(NewtonBodyPtr(raw, PhantomData));
            let datum = Box::new(UserData {
                body: Rc::downgrade(&body),
                collision: Rc::downgrade(&collision),
            });

            ffi::NewtonBodySetUserData(raw, mem::transmute(datum));
            NewtonBody {
                collision,
                body,
                raw,
            }
        }
    }

    pub fn collision(&self) -> NewtonCollision<C> {
        unsafe {
            let raw = ffi::NewtonBodyGetCollision(self.raw);

            match_rule! { (self, raw) =>
                Box -> CollisionBox,
                Sphere -> CollisionSphere,
                Cone -> CollisionCone,
            }
        }
    }

    pub fn set_mass(&self, mass: f32) {
        unsafe {
            ffi::NewtonBodySetMassProperties(self.raw, mass, ffi::NewtonBodyGetCollision(self.raw));
        }
    }

    pub fn set_update<F>(&self)
    where
        F: callback::ForceAndTorque<C>,
    {
        unsafe {
            ffi::NewtonBodySetForceAndTorqueCallback(self.raw, Some(force_torque_callback::<C, F>));
        }
    }

    pub fn mass(&self) -> Mass {
        unsafe {
            let mut mass = Mass {
                mass: 0.0,
                inertia: (0.0, 0.0, 0.0),
            };
            ffi::NewtonBodyGetMass(
                self.raw,
                &mut mass.mass,
                &mut mass.inertia.0,
                &mut mass.inertia.1,
                &mut mass.inertia.2,
            );
            mass
        }
    }

    pub fn matrix(&self) -> C::Matrix4 {
        unsafe {
            let mut mat = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.raw, mem::transmute(&mut mat));
            mat
        }
    }

    pub fn set_force(&self, force: C::Vector3) {
        unsafe { ffi::NewtonBodySetForce(self.raw, mem::transmute(&force)) }
    }

    pub fn set_linear_damping(&self, damp: f32) {
        unsafe { ffi::NewtonBodySetLinearDamping(self.raw, damp) }
    }

    pub fn center_of_mass(&self) -> C::Vector3 {
        unsafe {
            unsafe {
                let mut pos = mem::zeroed();
                ffi::NewtonBodyGetCentreOfMass(self.raw, mem::transmute(&mut pos));
                pos
            }
        }
    }

    pub fn position(&self) -> C::Vector3 {
        unsafe {
            let mut pos = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.raw, mem::transmute(&mut pos));
            pos
        }
    }

    pub fn point_velocity(&self, point: C::Vector3) -> C::Vector3 {
        unsafe {
            let mut v = mem::zeroed();
            ffi::NewtonBodyGetPointVelocity(
                self.raw,
                mem::transmute(&point),
                mem::transmute(&mut v),
            );
            v
        }
    }

    pub fn linear_velocity(&self) -> C::Vector3 {
        unsafe {
            let mut v = mem::zeroed();
            ffi::NewtonBodyGetVelocity(self.raw, mem::transmute(&mut v));
            v
        }
    }

    pub fn rotation(&self) -> C::Quaternion {
        unsafe {
            let mut rotation = mem::zeroed();
            ffi::NewtonBodyGetRotation(self.raw, mem::transmute(&mut rotation));
            rotation
        }
    }

    pub fn aabb(&self) -> (C::Vector3, C::Vector3) {
        unsafe {
            let mut p0: C::Vector3 = mem::zeroed();
            let mut p1: C::Vector3 = mem::zeroed();
            ffi::NewtonBodyGetAABB(self.raw, mem::transmute(&mut p0), mem::transmute(&mut p1));
            (p0, p1)
        }
    }

    pub fn set_sleep_state(&self, state: SleepState) {
        unsafe { ffi::NewtonBodySetSleepState(self.raw, mem::transmute(state)) }
    }

    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.raw)) }
    }
}

extern "C" fn force_torque_callback<V, C>(
    body: *const ffi::NewtonBody,
    timestep: f32,
    _thread_idx: i32,
) where
    V: NewtonConfig,
    C: callback::ForceAndTorque<V>,
{
    unsafe {
        let udata: Box<UserData<V>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
        match (Weak::upgrade(&udata.body), Weak::upgrade(&udata.collision)) {
            (Some(body), Some(collision)) => {
                let raw = body.0;
                let nanos = timestep.fract() * 1_000_000_000.0_f32;
                C::force_and_torque(
                    NewtonBody {
                        body,
                        collision,
                        raw,
                    },
                    Duration::new(timestep as u64, nanos as u32),
                );
            }
            _ => {}
        }

        mem::forget(udata);
    }
}
