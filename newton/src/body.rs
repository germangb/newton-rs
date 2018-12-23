use crate::callback;
use crate::ffi;
use crate::pointer::*;
use crate::userdata::*;
use crate::NewtonConfig;

use crate::collision::NewtonCollision;
use crate::collision::{Capsule, Cone, Cuboid, Cylinder, Sphere};
use crate::world::World;
use crate::Gravity;
use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};
use std::time::Duration;

/// A reference counted body
#[derive(Debug, Clone)]
pub struct Body<V> {
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,
    pub(crate) body: Rc<NewtonBodyPtr<V>>,

    pub(crate) raw: *mut ffi::NewtonBody,
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
        }
    };
    // used in `collision` method
    (
        ($selfi:ident, $raw:expr) =>
        $( $enumi:ident -> $structi:tt ,)+
    ) => {{
        match ffi::NewtonCollisionGetType($raw) {
            $(
                $structi ::<A>::TYPE_ID => NewtonCollision:: $enumi ($structi {
                    collision: $selfi.collision.clone(),
                    raw: $raw,
                }),
            )+
            _ => unreachable!(),
        }
    }}
}

impl<A> Body<A>
where
    A: NewtonConfig,
{
    // TODO refactor this mess
    pub fn from<C>(collision: C, matrix: A::Matrix) -> Self
    where
        C: Into<NewtonCollision<A>>,
    {
        unsafe {
            let collision = collision.into();
            let collision = collision.pointer().clone(); // Rc<CollisionPtr>

            let raw =
                ffi::NewtonCreateDynamicBody((collision.1).0, collision.0, mem::transmute(&matrix));
            let body = Rc::new(NewtonBodyPtr(raw, (collision.1).clone()));

            let datum = Box::new(BodyUserData {
                body: Rc::downgrade(&body),
                collision: Rc::downgrade(&collision),
            });

            ffi::NewtonBodySetUserData(raw, mem::transmute(datum));
            ffi::NewtonBodySetForceAndTorqueCallback(raw, Some(force_torque_callback::<A>));

            Body {
                collision,
                body,
                raw,
            }
        }
    }

    pub fn collision(&self) -> NewtonCollision<A> {
        unsafe {
            let raw = ffi::NewtonBodyGetCollision(self.raw);

            match_rule! { (self, raw) =>
                Box -> Cuboid,
                Sphere -> Sphere,
                Cone -> Cone,
                Cylinder -> Cylinder,
                Capsule -> Capsule,
            }
        }
    }

    pub fn set_mass(&self, mass: f32) {
        unsafe {
            ffi::NewtonBodySetMassProperties(self.raw, mass, ffi::NewtonBodyGetCollision(self.raw));
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

    pub fn matrix(&self) -> A::Matrix {
        unsafe {
            let mut mat = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.raw, mem::transmute(&mut mat));
            mat
        }
    }

    pub fn set_force(&self, force: A::Vector) {
        unsafe { ffi::NewtonBodySetForce(self.raw, mem::transmute(&force)) }
    }

    pub fn set_linear_damping(&self, damp: f32) {
        unsafe { ffi::NewtonBodySetLinearDamping(self.raw, damp) }
    }

    pub fn center_of_mass(&self) -> A::Vector {
        unsafe {
            unsafe {
                let mut pos = mem::zeroed();
                ffi::NewtonBodyGetCentreOfMass(self.raw, mem::transmute(&mut pos));
                pos
            }
        }
    }

    pub fn position(&self) -> A::Vector {
        unsafe {
            let mut pos = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.raw, mem::transmute(&mut pos));
            pos
        }
    }

    pub fn point_velocity(&self, point: A::Vector) -> A::Vector {
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

    pub fn linear_velocity(&self) -> A::Vector {
        unsafe {
            let mut v = mem::zeroed();
            ffi::NewtonBodyGetVelocity(self.raw, mem::transmute(&mut v));
            v
        }
    }

    pub fn rotation(&self) -> A::Quaternion {
        unsafe {
            let mut rotation = mem::zeroed();
            ffi::NewtonBodyGetRotation(self.raw, mem::transmute(&mut rotation));
            rotation
        }
    }

    pub fn aabb(&self) -> (A::Vector, A::Vector) {
        unsafe {
            let mut p0: A::Vector = mem::zeroed();
            let mut p1: A::Vector = mem::zeroed();
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

    pub fn as_raw(&self) -> *const ffi::NewtonBody {
        self.raw
    }
}

extern "C" fn force_torque_callback<V>(
    body: *const ffi::NewtonBody,
    timestep: f32,
    _thread_idx: i32,
) where
    V: NewtonConfig,
{
    unsafe {
        ffi::NewtonBodySetForce(body, [0.0, -10.0, 0.0].as_ptr());
        /*
        let udata: Box<BodyUserData<V>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
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
        */
    }
}
