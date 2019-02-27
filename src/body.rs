use std::marker::PhantomData;
use std::mem;
use std::sync::RwLock;
use std::time::Duration;

use crate::collision::{Collision, NewtonCollision};
use crate::ffi;
use crate::handle::{AsHandle, FromHandle, Handle, IntoHandle};
use crate::joint::iter::Joints;
use crate::newton::Newton;
use crate::{Mat4, Vec3};

/// Body iterators.
pub mod iter;

#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum SleepState {
    Active = 0,
    Sleeping = 1,
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
enum Type {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY,
    Kinematic = ffi::NEWTON_KINEMATIC_BODY,
}

#[derive(Default)]
struct UserData {
    /// Callback only applicable to dynamic bodies.
    /// Where you apply weight and other forces on the body.
    force_and_torque: Option<Box<dyn FnMut(Body, Duration, usize)>>,

    /// Body transform callback.
    /// This closure is called whenever there is a change in the transformation of a body.
    transform: Option<Box<dyn FnMut(Body, Mat4, usize)>>,

    /// Destructor callback
    // Type is FnMut because FnOnce cannot be consumed when Boxed.
    destructor: Option<Box<dyn FnMut(Body)>>,

    /// An optional name given to the body when it is created.
    name: Option<&'static str>,

    // Lock used to write/read body properties such as position, matrix, collision,
    // etc, ...
    //
    // Everything else that is critical to safety (use after free, null references, etc)
    // should be handled at compile time by the borrow checker.
    lock: RwLock<()>,
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

bodies! {
    /// Dynamic body wrapper.
    ///
    /// If a dynamic body has no mass, it is equivalent to a static body.
    #[derive(Debug, Eq, PartialEq)]
    (Dynamic, NewtonCreateDynamicBody, dynamic) => pub struct DynamicBody<'a>(...);

    /// A body that is not affected by forces and is controlled by the application.
    #[derive(Debug, Eq, PartialEq)]
    (Kinematic, NewtonCreateKinematicBody, kinematic) => pub struct KinematicBody<'a>(...);
}

macro_rules! lock {
    ($body:ident, read) => {
        unsafe {
            let udata = &ffi::NewtonBodyGetUserData($body.as_raw());
            let udata: &Box<UserData> = mem::transmute(udata);
            let _lock = udata.lock.read().unwrap();
        }
    };
    ($body:ident, write) => {
        unsafe {
            let udata = &ffi::NewtonBodyGetUserData($body.as_raw());
            let udata: &Box<UserData> = mem::transmute(udata);
            let _lock = udata.lock.write().unwrap();
        }
    };
}

unsafe extern "C" fn body_destructor(body: *const ffi::NewtonBody) {
    let udata = ffi::NewtonBodyGetUserData(body);
    let mut udata: Box<UserData> = Box::from_raw(udata as _);

    if let Some(mut destructor) = udata.destructor.take() {
        let body = Body::from_raw(body, false);
        destructor(body);
    }
}

/// Implementation of most of the NewtonBody API.
pub trait NewtonBody {
    fn as_raw(&self) -> *const ffi::NewtonBody;

    fn matrix(&self) -> Mat4 {
        lock!(self, read);
        let mut mat: Mat4 = Default::default();
        unsafe { ffi::NewtonBodyGetMatrix(self.as_raw(), mat[0].as_mut_ptr()) }
        mat
    }

    fn set_continuous(&self, cont: bool) {
        lock!(self, write);
        unsafe {
            let state = if cont { 1 } else { 0 };
            ffi::NewtonBodySetContinuousCollisionMode(self.as_raw(), state);
        }
    }

    fn is_continuous(&self) -> bool {
        lock!(self, write);
        unsafe {
            let state = ffi::NewtonBodyGetContinuousCollisionMode(self.as_raw());
            state == 1
        }
    }

    fn set_matrix(&self, matrix: Mat4) {
        lock!(self, write);
        unsafe { ffi::NewtonBodySetMatrix(self.as_raw(), matrix[0].as_ptr()) }
    }

    fn position(&self) -> Vec3 {
        lock!(self, read);
        let mut pos: Vec3 = Default::default();
        unsafe { ffi::NewtonBodyGetPosition(self.as_raw(), pos.as_mut_ptr()) }
        pos
    }

    fn into_raw(self) -> *const ffi::NewtonBody
        where Self: Sized
    {
        self.as_raw()
    }

    fn velocity(&self) -> Vec3 {
        lock!(self, read);
        let mut velo: Vec3 = Default::default();
        unsafe { ffi::NewtonBodyGetVelocity(self.as_raw(), velo.as_mut_ptr()) }
        velo
    }

    fn aabb(&self) -> (Vec3, Vec3) {
        lock!(self, read);
        let mut min: Vec3 = Default::default();
        let mut max: Vec3 = Default::default();
        unsafe {
            ffi::NewtonBodyGetAABB(self.as_raw(), min.as_mut_ptr(), max.as_mut_ptr());
        }
        (min, max)
    }

    /*
    fn body_type(&self) -> Type {
        unsafe {
            let btype = ffi::NewtonBodyGetType(self.as_raw());
            mem::transmute(btype)
        }
    }
    */

    fn sleep_state(&self) -> SleepState {
        lock!(self, read);
        unsafe {
            let state = ffi::NewtonBodyGetSleepState(self.as_raw());
            mem::transmute(state)
        }
    }

    fn set_sleep_state(&self, state: SleepState) {
        lock!(self, write);
        unsafe {
            let state = mem::transmute(state);
            ffi::NewtonBodySetSleepState(self.as_raw(), state);
        }
    }

    fn set_active(&self) {
        lock!(self, write);
        self.set_sleep_state(SleepState::Active)
    }

    fn set_sleeping(&self) {
        lock!(self, write);
        self.set_sleep_state(SleepState::Sleeping)
    }

    fn set_force(&self, force: Vec3) {
        lock!(self, write);
        unsafe { ffi::NewtonBodySetForce(self.as_raw(), force.as_ptr()) }
    }

    fn set_collision<C: NewtonCollision>(&self, collision: C) {
        lock!(self, write);
        unsafe { ffi::NewtonBodySetCollision(self.as_raw(), collision.as_raw()) }
    }

    fn collision(&self) -> Collision {
        lock!(self, read);
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

    fn mass(&self) -> (f32, Vec3) {
        lock!(self, read);
        let mut mass = 0.0;
        let mut i: Vec3 = [0.0, 0.0, 0.0];
        unsafe { ffi::NewtonBodyGetMass(self.as_raw(), &mut mass, &mut i[0], &mut i[1], &mut i[2]) }
        (mass, i)
    }

    fn set_mass<C: NewtonCollision>(&self, mass: f32, collision: &C) {
        lock!(self, write);
        let collision = collision.as_raw();
        unsafe {
            ffi::NewtonBodySetMassProperties(self.as_raw(), mass, collision);
        }
    }

    fn set_destroy_callback<F>(&self, callback: F)
        where F: FnMut(Body) + Send + 'static
    {
        lock!(self, write);
        unsafe {
            let mut udata: &mut Box<UserData> =
                mem::transmute(&mut ffi::NewtonBodyGetUserData(self.as_raw()));

            udata.destructor = Some(Box::new(callback));
        }
    }

    fn set_transform_callback<F>(&self, callback: F)
        where F: FnMut(Body, Mat4, usize) + Send + 'static
    {
        lock!(self, write);
        unsafe {
            let mut udata: &mut Box<UserData> =
                mem::transmute(&mut ffi::NewtonBodyGetUserData(self.as_raw()));
            udata.transform = Some(Box::new(callback));
            ffi::NewtonBodySetTransformCallback(self.as_raw(), Some(set_transform));
        }

        unsafe extern "C" fn set_transform(body: *const ffi::NewtonBody,
                                           matrix: *const f32,
                                           thread: std::os::raw::c_int) {
            let mut udata = ffi::NewtonBodyGetUserData(body);
            let udata: &mut Box<UserData> = mem::transmute(&mut udata);

            if let Some(callback) = &mut udata.transform {
                let matrix = mem::transmute::<_, &Mat4>(matrix).clone();
                callback(Body::from_raw(body, false), matrix, thread as _);
            }
        }
    }

    fn joints(&self) -> Joints {
        let joint = unsafe { ffi::NewtonBodyGetFirstJoint(self.as_raw()) };
        Joints { joint, body: self.as_raw(), _phantom: PhantomData }
    }

    fn set_force_and_torque_callback<F>(&self, callback: F)
        where F: FnMut(Body, Duration, usize) + Send + 'static
    {
        lock!(self, write);
        unsafe {
            let mut udata: &mut Box<UserData> =
                mem::transmute(&mut ffi::NewtonBodyGetUserData(self.as_raw()));
            udata.force_and_torque = Some(Box::new(callback));
            ffi::NewtonBodySetForceAndTorqueCallback(self.as_raw(), Some(force_and_torque));
        }

        unsafe extern "C" fn force_and_torque(body: *const ffi::NewtonBody,
                                              timestep: f32,
                                              thread: std::os::raw::c_int) {
            let seconds = timestep.floor() as u64;
            let nanos = (timestep.fract() * 1_000_000_000.0) as u32;
            let timestep = Duration::new(seconds, nanos);

            let mut udata = ffi::NewtonBodyGetUserData(body);
            let mut udata: &mut Box<UserData> = mem::transmute(&mut udata);
            if let Some(callback) = &mut udata.force_and_torque {
                let body = match mem::transmute::<_, Type>(ffi::NewtonBodyGetType(body)) {
                    Type::Dynamic => Body::Dynamic(DynamicBody { raw: body,
                                                                 _phantom: PhantomData,
                                                                 owned: false }),
                    Type::Kinematic => Body::Kinematic(KinematicBody { raw: body,
                                                                       _phantom: PhantomData,
                                                                       owned: false }),
                };
                callback(body, timestep, thread as usize);
            }
        }
    }
}
