use ffi;

use crate::pointer::*;
use crate::NewtonApp;

use crate::collision::Collision;
use crate::collision::IntoCollision;
use crate::collision::{
    BoxCollision, CapsuleCollision, ConeCollision, CylinderCollision, HeightFieldCollision,
    SphereCollision,
};
use crate::world::World;
use std::cell::Ref;
use std::marker::PhantomData;
use std::mem;
use std::rc::{Rc, Weak};
use std::time::Duration;

#[derive(Debug, Clone)]
pub enum Body<C> {
    Dynamic(DynamicBody<C>),
    Kinematic(KinematicBody<C>),
}

pub trait IntoBody<C> {
    fn into_body(self) -> Body<C>;
}

impl<C> Body<C> {
    pub fn dynamic(self) -> Option<DynamicBody<C>> {
        match self {
            Body::Dynamic(body) => Some(body),
            _ => None,
        }
    }
    pub fn kinematic(self) -> Option<KinematicBody<C>> {
        match self {
            Body::Kinematic(body) => Some(body),
            _ => None,
        }
    }
    pub fn is_dynamic(&self) -> bool {
        match &self {
            &Body::Dynamic(_) => true,
            _ => false,
        }
    }
    pub fn is_kinematic(&self) -> bool {
        match &self {
            &Body::Kinematic(_) => true,
            _ => false,
        }
    }
    pub fn as_raw(&self) -> *const ffi::NewtonBody {
        match &self {
            &Body::Dynamic(ref body) => body.raw,
            &Body::Kinematic(ref body) => body.raw,
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct BodyUserData<C> {
    pub(crate) dynamic: bool,
    pub(crate) body: Weak<NewtonBodyPtr<C>>,
    pub(crate) collision: Weak<NewtonCollisionPtr<C>>,
}

#[derive(Debug)]
pub struct Bodies<'a, C> {
    pub(crate) world: Ref<'a, NewtonWorldPtr<C>>,
    pub(crate) world_ptr: *mut ffi::NewtonWorld,
    pub(crate) current: *mut ffi::NewtonBody,
    pub(crate) _ph: PhantomData<C>,
}

impl<'a, C> Iterator for Bodies<'a, C> {
    type Item = Body<C>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current.is_null() {
            return None;
        }

        unsafe {
            // userdata
            let userdata: Box<BodyUserData<C>> =
                mem::transmute(ffi::NewtonBodyGetUserData(self.current));
            let next = if userdata.dynamic {
                Body::Dynamic(DynamicBody {
                    raw: self.current,
                    collision: Weak::upgrade(&userdata.collision)
                        .expect("Couldn't upgrade from Weak pointer"),
                    body: Weak::upgrade(&userdata.body)
                        .expect("Couldn't upgrade from Weak pointer"),
                })
            } else {
                Body::Kinematic(KinematicBody {
                    raw: self.current,
                    collision: Weak::upgrade(&userdata.collision)
                        .expect("Couldn't upgrade from Weak pointer"),
                    body: Weak::upgrade(&userdata.body)
                        .expect("Couldn't upgrade from Weak pointer"),
                })
            };

            mem::forget(userdata);
            self.current = ffi::NewtonWorldGetNextBody(self.world_ptr, self.current);
            Some(next)
        }
    }
}

impl<C> IntoBody<C> for DynamicBody<C> {
    fn into_body(self) -> Body<C> {
        Body::Dynamic(self)
    }
}

impl<C> IntoBody<C> for KinematicBody<C> {
    fn into_body(self) -> Body<C> {
        Body::Kinematic(self)
    }
}

impl<C> IntoBody<C> for Body<C> {
    fn into_body(self) -> Body<C> {
        self
    }
}

#[derive(Debug, Clone)]
pub struct DynamicBody<V> {
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,
    pub(crate) body: Rc<NewtonBodyPtr<V>>,
    pub(crate) raw: *mut ffi::NewtonBody,
}

#[derive(Debug, Clone)]
pub struct KinematicBody<V> {
    pub(crate) collision: Rc<NewtonCollisionPtr<V>>,
    pub(crate) body: Rc<NewtonBodyPtr<V>>,
    pub(crate) raw: *mut ffi::NewtonBody,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    Sleeping = 1,
}

macro_rules! impl_body {
    ($type:ident) => {
        impl<A> $type<A>
        where
            A: NewtonApp,
        {
            pub fn new<C>(collision: C, matrix: A::Matrix) -> Self
            where
                C: IntoCollision<A>,
            {
                unsafe {
                    let collision = collision.into_collision();
                    let collision = collision.pointer().clone(); // Rc<CollisionPtr>
                    let world = collision
                        .1
                        .try_borrow_mut()
                        .expect("Cannot perform mutable borrow");

                    let raw =
                        ffi::NewtonCreateDynamicBody(world.0, collision.0, mem::transmute(&matrix));
                    let body = Rc::new(NewtonBodyPtr(raw, (collision.1).clone()));

                    let datum = Box::new(BodyUserData {
                        dynamic: true,
                        body: Rc::downgrade(&body),
                        collision: Rc::downgrade(&collision),
                    });

                    ffi::NewtonBodySetUserData(raw, mem::transmute(datum));
                    ffi::NewtonBodySetForceAndTorqueCallback(raw, Some(force_torque_callback::<A>));

                    drop(world);
                    Self {
                        collision,
                        body,
                        raw,
                    }
                }
            }

            pub fn collision(&self) -> Collision<A> {
                unsafe {
                    let raw = ffi::NewtonBodyGetCollision(self.as_raw());
                    let collision = self.collision.clone();

                    match ffi::NewtonCollisionGetType(collision.0) {
                        BoxCollision::<A>::TYPE_ID => {
                            Collision::Box(BoxCollision { collision, raw })
                        }
                        SphereCollision::<A>::TYPE_ID => {
                            Collision::Sphere(SphereCollision { collision, raw })
                        }
                        ConeCollision::<A>::TYPE_ID => {
                            Collision::Cone(ConeCollision { collision, raw })
                        }
                        CylinderCollision::<A>::TYPE_ID => {
                            Collision::Cylinder(CylinderCollision { collision, raw })
                        }
                        CapsuleCollision::<A>::TYPE_ID => {
                            Collision::Capsule(CapsuleCollision { collision, raw })
                        }
                        HeightFieldCollision::<A>::TYPE_ID => {
                            Collision::HeightField(HeightFieldCollision { collision, raw })
                        }
                        _ => unreachable!(),
                    }
                }
            }

            pub fn set_mass(&self, mass: f32) {
                unsafe {
                    ffi::NewtonBodySetMassProperties(
                        self.as_raw(),
                        mass,
                        ffi::NewtonBodyGetCollision(self.raw),
                    )
                };
            }

            pub fn matrix(&self) -> A::Matrix {
                unsafe {
                    let mut mat = mem::zeroed();
                    ffi::NewtonBodyGetMatrix(self.as_raw(), mem::transmute(&mut mat));
                    mat
                }
            }

            pub fn set_force(&self, force: A::Vector) {
                unsafe { ffi::NewtonBodySetForce(self.as_raw(), mem::transmute(&force)) }
            }

            pub fn set_velocity(&self, velocity: A::Vector) {
                unsafe { ffi::NewtonBodySetVelocity(self.as_raw(), mem::transmute(&velocity)) }
            }

            pub fn set_velocity_no_sleep(&self, velocity: A::Vector) {
                unsafe {
                    ffi::NewtonBodySetVelocityNoSleep(self.as_raw(), mem::transmute(&velocity))
                }
            }

            pub fn add_force(&self, force: A::Vector) {
                unsafe { ffi::NewtonBodyAddForce(self.as_raw(), mem::transmute(&force)) }
            }

            pub fn set_linear_damping(&self, damp: f32) {
                unsafe { ffi::NewtonBodySetLinearDamping(self.as_raw(), damp) }
            }

            pub fn center_of_mass(&self) -> A::Vector {
                unsafe {
                    unsafe {
                        let mut pos = mem::zeroed();
                        ffi::NewtonBodyGetCentreOfMass(self.as_raw(), mem::transmute(&mut pos));
                        pos
                    }
                }
            }

            pub fn position(&self) -> A::Vector {
                unsafe {
                    let mut pos = mem::zeroed();
                    ffi::NewtonBodyGetPosition(self.as_raw(), mem::transmute(&mut pos));
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
                    ffi::NewtonBodyGetVelocity(self.as_raw(), mem::transmute(&mut v));
                    v
                }
            }

            pub fn rotation(&self) -> A::Quaternion {
                unsafe {
                    let mut rotation = mem::zeroed();
                    ffi::NewtonBodyGetRotation(self.as_raw(), mem::transmute(&mut rotation));
                    rotation
                }
            }

            pub fn aabb(&self) -> (A::Vector, A::Vector) {
                unsafe {
                    let mut p0: A::Vector = mem::zeroed();
                    let mut p1: A::Vector = mem::zeroed();
                    ffi::NewtonBodyGetAABB(
                        self.as_raw(),
                        mem::transmute(&mut p0),
                        mem::transmute(&mut p1),
                    );
                    (p0, p1)
                }
            }

            pub fn awake(&self) {
                self.set_sleep_state(SleepState::Awake)
            }

            pub fn set_sleep_state(&self, state: SleepState) {
                unsafe { ffi::NewtonBodySetSleepState(self.as_raw(), mem::transmute(state)) }
            }

            pub fn sleep_state(&self) -> SleepState {
                unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.as_raw())) }
            }

            pub fn as_raw(&self) -> *const ffi::NewtonBody {
                self.raw
            }
        }
    };
}

impl_body!(DynamicBody);
impl_body!(KinematicBody);

extern "C" fn force_torque_callback<V>(body: *const ffi::NewtonBody, _time: f32, _idx: i32)
where
    V: NewtonApp,
{
    unsafe {
        ffi::NewtonBodySetForce(body, [0.0, -10.0, 0.0].as_ptr());
    }
}
