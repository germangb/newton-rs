use crate::callback;
use crate::ffi;
use crate::world::WorldRef;
use crate::NewtonConfig;

use crate::collision::NewtonCollision;
use crate::world::NewtonWorld;
use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};

/// Reference counted body
pub type NewtonBody<V> = Rc<NewtonBodyInner<V>>;

/// The result of downgrading a `NewtonBody`
type WeakNewtonBody<V> = Weak<NewtonBodyInner<V>>;

#[derive(Debug)]
pub struct NewtonBodyInner<V> {
    pub(crate) world: Rc<WorldRef>,
    pub(crate) body: *mut ffi::NewtonBody,
    pub(crate) _ph: PhantomData<V>,
}

pub(crate) struct UserData<V> {
    pub body: WeakNewtonBody<V>,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    Sleeping = 1,
}

// XXX pointers
impl<V> NewtonBodyInner<V>
where
    V: NewtonConfig,
{
    pub fn set_mass(&self, mass: f32, collision: &NewtonCollision<V>) {
        unsafe {
            ffi::NewtonBodySetMassProperties(self.body, mass, collision.collision);
        }
    }

    pub fn set_update<C: callback::ForceAndTorque<V>>(&self) {
        unsafe {
            ffi::NewtonBodySetForceAndTorqueCallback(self.body, Some(cb_apply_force::<V, C>));
        }
        // TODO move this function somewhere else
        extern "C" fn cb_apply_force<V, C>(
            body: *const ffi::NewtonBody,
            _timestep: f32,
            _thread_idx: i32,
        ) where
            V: NewtonConfig,
            C: callback::ForceAndTorque<V>,
        {
            // TODO audit unsafe
            unsafe {
                let udata: Box<UserData<V>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
                if let Some(body) = Weak::upgrade(&udata.body) {
                    C::force_and_torque(body);
                }
                mem::forget(udata);
            }
        }
    }

    pub fn matrix(&self) -> V::Matrix4 {
        unsafe {
            let mut mat = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.body, mem::transmute(&mut mat));
            mat
        }
    }

    pub fn set_force(&self, force: V::Vector3) {
        unsafe { ffi::NewtonBodySetForce(self.body, mem::transmute(&force)) }
    }

    pub fn set_linear_damping(&self, damp: f32) {
        unsafe { ffi::NewtonBodySetLinearDamping(self.body, damp) }
    }

    pub fn center_of_mass(&self) -> V::Vector3 {
        unsafe {
            unsafe {
                let mut pos = mem::zeroed();
                ffi::NewtonBodyGetCentreOfMass(self.body, mem::transmute(&mut pos));
                pos
            }
        }
    }

    pub fn position(&self) -> V::Vector3 {
        unsafe {
            let mut pos = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.body, mem::transmute(&mut pos));
            pos
        }
    }

    pub fn point_velocity(&self, point: V::Vector3) -> V::Vector3 {
        unsafe {
            let mut v = mem::zeroed();
            ffi::NewtonBodyGetPointVelocity(
                self.body,
                mem::transmute(&point),
                mem::transmute(&mut v),
            );
            v
        }
    }

    pub fn linear_velocity(&self) -> V::Vector3 {
        unsafe {
            let mut v = mem::zeroed();
            ffi::NewtonBodyGetVelocity(self.body, mem::transmute(&mut v));
            v
        }
    }

    pub fn rotation(&self) -> V::Quaternion {
        unsafe {
            let mut rotation = mem::zeroed();
            ffi::NewtonBodyGetRotation(self.body, mem::transmute(&mut rotation));
            rotation
        }
    }

    pub fn aabb(&self) -> (V::Vector3, V::Vector3) {
        unsafe {
            let mut p0: V::Vector3 = mem::zeroed();
            let mut p1: V::Vector3 = mem::zeroed();
            ffi::NewtonBodyGetAABB(self.body, mem::transmute(&mut p0), mem::transmute(&mut p1));
            (p0, p1)
        }
    }

    pub fn set_sleep_state(&self, state: SleepState) {
        unsafe { ffi::NewtonBodySetSleepState(self.body, mem::transmute(state)) }
    }

    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.body)) }
    }
}

impl<V> Drop for NewtonBodyInner<V> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<UserData<V>> = Box::from_raw(ffi::NewtonBodyGetUserData(self.body) as _);
            ffi::NewtonDestroyBody(self.body);
        }
    }
}

pub(crate) fn create_dynamic_body<V>(
    world: Rc<WorldRef>,
    collision: *mut ffi::NewtonCollision,
    matrix: V::Matrix4,
) -> NewtonBody<V>
where
    V: NewtonConfig,
{
    assert_config!(V);

    unsafe {
        let body = ffi::NewtonCreateDynamicBody(world.0, collision, mem::transmute(&matrix));
        let body = Rc::new(NewtonBodyInner {
            world,
            body,
            _ph: PhantomData,
        });

        let datum = mem::transmute(Box::new(UserData {
            body: Rc::downgrade(&body),
        }));
        ffi::NewtonBodySetUserData(body.body, datum);

        body
    }
}
