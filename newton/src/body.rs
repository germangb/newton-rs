use crate::callback;
use crate::ffi;
use crate::world::NewtonWorldPtr;
use crate::NewtonConfig;

use crate::collision::{NewtonCollision, NewtonCollisionPtr};
use crate::world::NewtonWorld;
use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonBodyPtr<C>(pub(crate) *mut ffi::NewtonBody, PhantomData<C>);

impl<C> Drop for NewtonBodyPtr<C> {
    fn drop(&mut self) {
        unsafe {
            let _: Box<UserData<C>> = Box::from_raw(ffi::NewtonBodyGetUserData(self.0) as _);
            ffi::NewtonDestroyBody(self.0)
        }
    }
}

#[derive(Debug, Clone)]
pub struct NewtonBody<V> {
    pub(crate) world: Rc<NewtonWorldPtr<V>>,
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,
    pub(crate) body: Rc<NewtonBodyPtr<V>>,

    raw: *mut ffi::NewtonBody,
}

pub(crate) struct UserData<C> {
    pub(crate) world: Weak<NewtonWorldPtr<C>>,
    pub(crate) body: Weak<NewtonBodyPtr<C>>,
    pub(crate) collision: Weak<NewtonCollisionPtr<C>>,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    Sleeping = 1,
}

#[derive(Debug, Copy, Clone)]
pub struct Mass {
    /// Body mass
    pub mass: f32,
    /// Moment of inertia of the principal components of the I matrix
    pub inertia: (f32, f32, f32),
}

// XXX pointers
impl<C> NewtonBody<C>
where
    C: NewtonConfig,
{
    pub fn collision(&self) -> NewtonCollision<C> {
        unsafe {
            NewtonCollision {
                collision: self.collision.clone(),

                raw: ffi::NewtonBodyGetCollision(self.raw),
            }
        }
    }

    pub fn set_mass(&self, mass: f32, collision: &NewtonCollision<C>) {
        unsafe {
            ffi::NewtonBodySetMassProperties(self.raw, mass, collision.collision.0);
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

pub(crate) fn create_dynamic_body<V>(
    world_ptr: Rc<NewtonWorldPtr<V>>,
    collision: Rc<NewtonCollisionPtr<V>>,
    matrix: V::Matrix4,
) -> NewtonBody<V>
where
    V: NewtonConfig,
{
    assert_config!(V);

    unsafe {
        let body = ffi::NewtonCreateDynamicBody(world_ptr.0, collision.0, mem::transmute(&matrix));
        let body_ptr = Rc::new(NewtonBodyPtr(body, PhantomData));

        let datum = Box::new(UserData {
            body: Rc::downgrade(&body_ptr),
            world: Rc::downgrade(&world_ptr),
            collision: Rc::downgrade(&collision),
        });

        let body = NewtonBody {
            world: world_ptr,
            body: body_ptr,
            collision,
            raw: body,
        };

        ffi::NewtonBodySetUserData(body.raw, mem::transmute(datum));
        body
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

        match (
            Weak::upgrade(&udata.world),
            Weak::upgrade(&udata.body),
            Weak::upgrade(&udata.collision),
        ) {
            (Some(world), Some(body), Some(collision)) => {
                let raw = body.0;
                C::force_and_torque(
                    NewtonBody {
                        world,
                        body,
                        collision,
                        raw,
                    },
                    timestep,
                );
            }
            _ => {}
        }

        // TODO is it ok to forget the pointer here?
        mem::forget(udata);
    }
}
