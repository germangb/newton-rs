use std::marker::PhantomData;
use std::mem;
use std::time::Duration;

use super::collision::{Collision, NewtonCollision};
use super::ffi;
use super::newton::Newton;
use super::Handle;
use super::{AsHandle, IntoHandle};

pub mod iters;

#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum SleepState {
    Active = 0,
    Asleep = 1,
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum Type {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY,
    Kinematic = ffi::NEWTON_KINEMATIC_BODY,
}

#[derive(Default)]
struct UserData {
    /// Callback only applicable to dynamic bodies.
    /// Where you apply weight and other forces on the body.
    force_and_torque: Option<Box<dyn FnMut(&Body, Duration, usize)>>,

    /// An optional name given to the body when it is created.
    name: Option<&'static str>,
}

// Where should I place the lifetime, in the type, or in the method??
trait IntoBody<'a> {
    fn into_body(self) -> Body<'a>;
}

impl<'a> IntoBody<'a> for Body<'a> {
    fn into_body(self) -> Body<'a> {
        self
    }
}

macro_rules! bodies {
    ($(
        $(#[$($meta:meta)+])*
        ($enum:ident, $ffi:ident) => pub struct $body:ident<'a>(...);
    )*) => {
        #[derive(Debug, Eq, PartialEq)]
        pub enum Body<'a> {
            $( $enum($body<'a>) ),*
        }

        impl<'a> NewtonBody for Body<'a> {
            fn as_raw(&self) -> *const ffi::NewtonBody {
                match self {
                    $(Body::$enum(body) => body.as_raw()),*
                }
            }
        }

        impl<'a> IntoHandle for Body<'a> {
            fn into_handle(mut self, newton: &Newton) -> Handle {
                match &mut self {
                    $(Body::$enum(ref mut body) => if !body.owned { panic!() } else { body.owned = false; }),*
                }
                newton.move_body2(self)
            }
        }

        impl<'a> AsHandle for Body<'a> {
            fn as_handle(&self) -> Handle {
                Handle::from_ptr(self.as_raw() as _)
            }
        }

        impl<'a> Body<'a> {
            pub(crate) unsafe fn from_raw(raw: *const ffi::NewtonBody, owned: bool) -> Self {
                let body_type = ffi::NewtonBodyGetType(raw);
                match body_type as _ {
                    ffi::NEWTON_DYNAMIC_BODY => Body::Dynamic(DynamicBody::from_raw(raw, owned)),
                    ffi::NEWTON_KINEMATIC_BODY => Body::Kinematic(KinematicBody::from_raw(raw, owned)),
                    _ => unreachable!("Unexpected body type ({})", body_type),
                }
            }

            pub fn dynamic(self) -> Option<DynamicBody<'a>> {
                match self {
                    Body::Dynamic(body) => Some(body),
                    _ => None,
                }
            }

            pub fn kinematic(self) -> Option<KinematicBody<'a>> {
                match self {
                    Body::Kinematic(body) => Some(body),
                    _ => None,
                }
            }
        }

        $(
            $(#[$($meta)+])*
            pub struct $body<'a> {
                raw: *const ffi::NewtonBody,
                _phantom: PhantomData<&'a ()>,
                // Bodies from iterators or callbacks generally won't be owned.
                // When they are, the memory is freed when the object is dropped.
                pub(crate) owned: bool,
            }

            impl<'a> From<$body<'a>> for Body<'a> {
                fn from(body: $body<'a>) -> Self {
                    Body::$enum ( body )
                }
            }

            unsafe impl<'a> Send for $body<'a> {}
            unsafe impl<'a> Sync for $body<'a> {}

            impl<'a> IntoBody<'a> for $body<'a> {
                fn into_body(self) -> Body<'a> {
                    Body::$enum(self)
                }
            }

            impl<'a> NewtonBody for $body<'a> {
                fn as_raw(&self) -> *const ffi::NewtonBody {
                    self.raw
                }
            }

            impl<'a> Drop for $body<'a> {
                fn drop(&mut self) {
                    if self.owned {
                        unsafe {
                            let udata = ffi::NewtonBodyGetUserData(self.raw);
                            let udata: Box<UserData> = mem::transmute(udata);
                            ffi::NewtonDestroyBody(self.raw);
                        }
                    }
                }
            }

            impl<'a> IntoHandle for $body<'a> {
                fn into_handle(mut self, newton: &Newton) -> Handle {
                    if !self.owned { panic!() }
                    self.owned = false;
                    newton.move_body2(self.into_body())
                }
            }

            impl<'a> AsHandle for $body<'a> {
                fn as_handle(&self) -> Handle {
                    Handle::from_ptr(self.raw as _)
                }
            }

            impl<'a> $body<'a> {
                pub unsafe fn from_raw(raw: *const ffi::NewtonBody, owned: bool) -> Self {
                    Self {
                        raw,
                        owned,
                        _phantom: PhantomData,
                    }
                }

                pub fn create<C>(newton: &'a Newton,
                                 collision: &C,
                                 matrix: [[f32; 4]; 4],
                                 name: Option<&'static str>) -> Self
                where
                    C: NewtonCollision,
                {
                    unsafe {
                        let newton = newton.as_raw();
                        let matrix = matrix[0].as_ptr();
                        let collision = collision.as_raw();

                        let body = ffi::$ffi(newton, collision, matrix);
                        let mut userdata = Box::new(UserData { name, ..Default::default() });

                        ffi::NewtonBodySetUserData(body, mem::transmute(userdata));
                        Self { raw: body, owned: true, _phantom: PhantomData }
                    }
                }
            }
        )*
    }
}

bodies! {
    /// Dynamic body wrapper.
    ///
    /// If a dynamic body has no mass, it is equivalent to a static body.
    #[derive(Debug, Eq, PartialEq)]
    (Dynamic, NewtonCreateDynamicBody) => pub struct DynamicBody<'a>(...);

    /// A body that is not affected by forces and is controlled by the application.
    #[derive(Debug, Eq, PartialEq)]
    (Kinematic, NewtonCreateKinematicBody) => pub struct KinematicBody<'a>(...);
}

impl<'a> Dynamic for DynamicBody<'a> {}
impl<'a> Dynamic for KinematicBody<'a> {}

/// Trait for bodies that have a mass and are affected by forces
pub trait Dynamic: NewtonBody {}

/// Implementation of most of the NewtonBody API.
pub trait NewtonBody {
    fn as_raw(&self) -> *const ffi::NewtonBody;

    fn matrix(&self) -> [[f32; 4]; 4] {
        let mut mat: [[f32; 4]; 4] = Default::default();
        unsafe { ffi::NewtonBodyGetMatrix(self.as_raw(), mat[0].as_mut_ptr()) }
        mat
    }

    fn set_continuous(&self, cont: bool) {
        unsafe {
            let state = if cont { 1 } else { 0 };
            ffi::NewtonBodySetContinuousCollisionMode(self.as_raw(), state);
        }
    }

    fn is_continuous(&self) -> bool {
        unsafe {
            let state = ffi::NewtonBodyGetContinuousCollisionMode(self.as_raw());
            state == 1
        }
    }

    fn set_matrix(&self, matrix: [[f32; 4]; 4]) {
        unsafe { ffi::NewtonBodySetMatrix(self.as_raw(), matrix[0].as_ptr()) }
    }

    fn position(&self) -> [f32; 3] {
        let mut pos: [f32; 3] = Default::default();
        unsafe { ffi::NewtonBodyGetPosition(self.as_raw(), pos.as_mut_ptr()) }
        pos
    }

    fn into_raw(self) -> *const ffi::NewtonBody
    where
        Self: Sized,
    {
        self.as_raw()
    }

    fn velocity(&self) -> [f32; 3] {
        let mut velo: [f32; 3] = Default::default();
        unsafe { ffi::NewtonBodyGetVelocity(self.as_raw(), velo.as_mut_ptr()) }
        velo
    }

    fn aabb(&self) -> ([f32; 3], [f32; 3]) {
        let mut min: [f32; 3] = Default::default();
        let mut max: [f32; 3] = Default::default();
        unsafe {
            ffi::NewtonBodyGetAABB(self.as_raw(), min.as_mut_ptr(), max.as_mut_ptr());
        }
        (min, max)
    }

    fn body_type(&self) -> Type {
        unsafe {
            let btype = ffi::NewtonBodyGetType(self.as_raw());
            mem::transmute(btype)
        }
    }

    fn sleep_state(&self) -> SleepState {
        unsafe {
            let state = ffi::NewtonBodyGetSleepState(self.as_raw());
            mem::transmute(state)
        }
    }

    fn set_sleep_state(&self, state: SleepState) {
        unsafe {
            let state = mem::transmute(state);
            ffi::NewtonBodySetSleepState(self.as_raw(), state);
        }
    }

    fn set_active(&self) {
        self.set_sleep_state(SleepState::Active)
    }

    fn set_asleep(&self) {
        self.set_sleep_state(SleepState::Asleep)
    }

    fn set_force(&self, force: [f32; 3]) {
        unsafe { ffi::NewtonBodySetForce(self.as_raw(), force.as_ptr()) }
    }

    fn collision(&self) -> Collision {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.as_raw());
            Collision::from_raw(collision, false)
        }
    }

    fn name(&self) -> Option<&'static str> {
        unsafe {
            let udata = &ffi::NewtonBodyGetUserData(self.as_raw());
            let udata: &Box<UserData> = mem::transmute(udata);
            udata.name
        }
    }

    fn mass(&self) -> (f32, [f32; 3]) {
        let mut mass = 0.0;
        let mut i: [f32; 3] = [0.0, 0.0, 0.0];
        unsafe { ffi::NewtonBodyGetMass(self.as_raw(), &mut mass, &mut i[0], &mut i[1], &mut i[2]) }
        (mass, i)
    }

    fn set_mass<C: NewtonCollision>(&self, mass: f32, collision: &C) {
        let collision = collision.as_raw();
        unsafe {
            ffi::NewtonBodySetMassProperties(self.as_raw(), mass, collision);
        }
    }

    fn set_force_and_torque_callback<F>(&self, callback: F)
    where
        F: FnMut(&Body, Duration, usize) + 'static,
    {
        unsafe {
            let mut udata: &mut Box<UserData> =
                mem::transmute(&mut ffi::NewtonBodyGetUserData(self.as_raw()));
            udata.force_and_torque = Some(Box::new(callback));
            ffi::NewtonBodySetForceAndTorqueCallback(self.as_raw(), Some(force_and_torque));
        }

        unsafe extern "C" fn force_and_torque(
            body: *const ffi::NewtonBody,
            timestep: f32,
            thread: std::os::raw::c_int,
        ) {
            let seconds = timestep.floor() as u64;
            let nanos = (timestep.fract() * 1_000_000_000.0) as u32;
            let timestep = Duration::new(seconds, nanos);

            let mut udata = ffi::NewtonBodyGetUserData(body);
            let mut udata: &mut Box<UserData> = mem::transmute(&mut udata);
            if let Some(callback) = &mut udata.force_and_torque {
                let body = match ffi::NewtonBodyGetType(body) as _ {
                    ffi::NEWTON_DYNAMIC_BODY => Body::Dynamic(DynamicBody {
                        raw: body,
                        _phantom: PhantomData,
                        owned: false,
                    }),
                    ffi::NEWTON_KINEMATIC_BODY => Body::Kinematic(KinematicBody {
                        raw: body,
                        _phantom: PhantomData,
                        owned: false,
                    }),
                    _ => unreachable!(),
                };
                callback(&body, timestep, thread as usize);
            }
        }
    }
}
