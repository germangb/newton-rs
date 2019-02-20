use std::mem;
use std::time::Duration;
use std::marker::PhantomData;

use super::collision::CollisionOld;
use super::collision::{Collision, CollisionTrait};
use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};
use super::Handle;
use crate::body::BodyType::Kinematic;

#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum SleepState {
    Active = 0,
    Sleeping = 1,
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum BodyType {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY,
    Kinematic = ffi::NEWTON_KINEMATIC_BODY,
}

struct UserData {
    /// A pointer to the structure that owns the Newton object.
    ///
    /// # Safety
    /// It must be ensured that this value always points to valid Newton memory,
    /// otherwise it will never be freed and could cause a memory leak!
    owner: *const (),

    /// Callback only applicable to dynamic bodies.
    /// Where you apply weight and other forces on the body.
    force_and_torque: Option<Box<dyn FnMut(&Body, Duration, usize)>>,

    /// An optional name given to the body when it is created.
    name: Option<&'static str>,
}

// Where should I place the lifetime, in the type, or in the method??
pub trait IntoBody<'a> {
    fn into_body(self) -> Body<'a>;
}

macro_rules! bodies {
    ($(
        $(#[$($meta:meta)+])*
        ($enum:ident, $ffi:ident) => pub struct $body:ident<'world>(...);
    )*) => {
        impl<'world> BodyTrait for Body<'world> {
            fn as_raw(&self) -> *const ffi::NewtonBody {
                match self {
                    $(Body::$enum(body) => body.as_raw()),*
                }
            }
        }

        /// An *either* type for Dynamic and Kinematic bodies.
        #[derive(Debug, Eq, PartialEq)]
        pub enum Body<'world> {
            $( $enum($body<'world>) ),*
        }

        $(
            $(#[$($meta)+])*
            pub struct $body<'world>(*const ffi::NewtonBody, PhantomData<&'world ()>);

            impl<'world> IntoBody<'world> for $body<'world> {
                fn into_body(self) -> Body<'world> {
                    Body::$enum(self)
                }
            }

            impl<'world> BodyTrait for $body<'world> {
                fn as_raw(&self) -> *const ffi::NewtonBody {
                    self.0
                }
            }

            impl<'world> Drop for $body<'world> {
                fn drop(&mut self) {
                    // if body owns itself, then free the memory
                    unsafe {
                        let udata = ffi::NewtonBodyGetUserData(self.0);
                        let udata: Box<UserData> = mem::transmute(udata);
                        if udata.owner == self.0 as *const () {
                            //ffi::NewtonDestroyBody(self.0);
                            println!("drop");
                        } else {
                            mem::forget(udata);
                        }
                    }
                }
            }

            impl<'world> $body<'world> {
                pub fn create<C>(newton: &'world Newton,
                                 collision: C,
                                 matrix: [[f32; 4]; 4],
                                 name: Option<&'static str>) -> Self
                where
                    C: CollisionTrait,
                {
                    unsafe {
                        let newton = newton.as_raw();
                        let matrix = matrix[0].as_ptr();
                        let collision = collision.as_raw();

                        let body = ffi::$ffi(newton, collision, matrix);
                        let userdata = Box::new(UserData { owner: body as _, force_and_torque: None, name });

                        ffi::NewtonBodySetUserData(body, mem::transmute(userdata));
                        $body(body, PhantomData)
                    }
                }

                pub(crate) fn set_owner(&mut self, owner: *const ()) {
                    unsafe {
                        let mut udata = ffi::NewtonBodyGetUserData(self.0);
                        let mut udata: &mut Box<UserData> = mem::transmute(&mut udata);
                        udata.owner = owner;
                    }
                }
            }
        )*
    }
}

bodies! {
    /// Dynamic body wrapper
    ///
    /// Bodies cannot outlive the NewtonWorld they are created from.
    /// The size of this type is still the same as the raw pointer.
    #[derive(Debug, Eq, PartialEq)]
    (Dynamic, NewtonCreateDynamicBody) => pub struct DynamicBody<'world>(...);

    /// Kinematic body wrapper
    ///
    /// Kinematic bodies are not affected by forces.
    /// Bodies cannot outlive the NewtonWorld they are created from.
    /// The size of this type is still the same as the raw pointer.
    #[derive(Debug, Eq, PartialEq)]
    (Kinematic, NewtonCreateKinematicBody) => pub struct KinematicBody<'world>(...);
}

pub trait BodyTrait {
    fn as_raw(&self) -> *const ffi::NewtonBody;

    fn into_handle<'w>(self, newton: &'w Newton) -> Handle
    where
        Self: Sized + IntoBody<'w>,
    {
        newton.move_body(self)
    }

    fn matrix(&self) -> [[f32; 4]; 4] {
        let mut mat: [[f32; 4]; 4] = Default::default();
        unsafe { ffi::NewtonBodyGetMatrix(self.as_raw(), mat[0].as_mut_ptr()) }
        mat
    }

    fn set_matrix(&self, matrix: [[f32; 4]; 4]) {
        unsafe { ffi::NewtonBodySetMatrix(self.as_raw(), matrix[0].as_ptr()) }
    }

    fn position(&self) -> [f32; 3] {
        let mut pos: [f32; 3] = Default::default();
        unsafe { ffi::NewtonBodyGetPosition(self.as_raw(), pos.as_mut_ptr()) }
        pos
    }

    fn body_type(&self) -> BodyType {
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

    fn get_mass(&self) -> (f32, [f32; 3]) {
        let mut mass = 0.0;
        let mut i: [f32; 3] = [0.0, 0.0, 0.0];
        unsafe {
            ffi::NewtonBodyGetMass(self.as_raw(),
                                   &mut mass,
                                   &mut i[0],
                                   &mut i[1],
                                   &mut i[2])
        }
        (mass, i)
    }

    fn set_mass<C: CollisionTrait>(&self, mass: f32, collision: C) {
        let collision = collision.as_raw();
        unsafe { ffi::NewtonBodySetMassProperties(self.as_raw(), mass, collision); }
    }

    fn set_force(&self, force: [f32; 3]) {
        unsafe { ffi::NewtonBodySetForce(self.as_raw(), force.as_ptr()) }
    }

    fn collision(&self) -> Collision {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.as_raw());
            Collision::from_raw(collision)
        }
    }

    fn set_force_and_torque_callback<F>(&self, callback: F)
    where
        F: FnMut(&Body, Duration, usize) + 'static,
    {
        unsafe {
            let mut udata: &mut Box<UserData> = mem::transmute(&mut ffi::NewtonBodyGetUserData(self.as_raw()));
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
                    ffi::NEWTON_DYNAMIC_BODY => Body::Dynamic(DynamicBody(body, PhantomData)),
                    ffi::NEWTON_KINEMATIC_BODY => Body::Kinematic(KinematicBody(body, PhantomData)),
                    _ => unreachable!(),
                };
                callback(&body, timestep, thread as usize);
            }
        }
    }
}

#[derive(Default)]
struct BodyData {
    /// Force and torque callback
    force_and_torque: Option<Box<dyn Fn(BodyOld, Duration)>>,
    /// Debug name
    name: Option<&'static str>,
}

/// NewtonBody Wrapper
///
/// The borrow type will prevent the underlying NewtonWorld pointer from
/// outliving the NewtonWorld.
pub struct BodyOld<'world> {
    pub(crate) newton: &'world Newton,
    pub(crate) body: *const ffi::NewtonBody,
    pub(crate) owned: bool,
}

unsafe impl<'world> Send for BodyOld<'world> {}
unsafe impl<'world> Sync for BodyOld<'world> {}

/// Opaque handle to a NewtonBody
///
/// The underlying NewtonBody is owned by a [`Newton`][Newton], and can be accessed
/// through its [`body`][body], [`body_mut`][body_mut] and [`body_owned`][body_owned] methods.
///
/// [Newton]: #
/// [body]: #
/// [body_mut]: #
/// [body_owned]: #
#[derive(Debug, Hash, Eq, PartialEq, Clone, Copy)]
pub struct HandleOld(pub(crate) *const ffi::NewtonBody);

unsafe impl Send for HandleOld {}
unsafe impl Sync for HandleOld {}

#[derive(Debug)]
pub struct Bodies<'world>(pub(crate) &'world Newton);

pub struct Iter<'world> {
    newton: &'world Newton,
    next: *const ffi::NewtonBody,
}

impl<'world> Iterator for Iter<'world> {
    type Item = BodyOld<'world>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.next;
        if current.is_null() {
            None
        } else {
            self.next = unsafe { ffi::NewtonWorldGetNextBody(self.newton.as_raw(), current) };
            Some(BodyOld {
                newton: self.newton,
                body: current,
                owned: false,
            })
        }
    }
}

impl<'world> Bodies<'world> {
    pub fn count(&self) -> usize {
        unsafe { ffi::NewtonWorldGetBodyCount(self.0.as_raw()) as usize }
    }

    pub fn iter(&self) -> Iter {
        Iter {
            newton: self.0,
            next: unsafe { ffi::NewtonWorldGetFirstBody(self.0.as_raw()) },
        }
    }
}

impl<'world> BodyOld<'world> {
    /// Wraps a raw NewtonBody pointer and returns a **non-owned** Body.
    ///
    /// Being non-owned means the wrapped pointer won't be freed when the body is
    /// dropped. If you intend it to do so, call the `from_raw_owned` method instead.
    pub unsafe fn from_raw(newton: &'world Newton, body: *mut ffi::NewtonBody) -> Self {
        let mut body = Self::from_raw_owned(newton, body);
        body.owned = false;
        body
    }

    pub unsafe fn from_raw_owned(newton: &'world Newton, body: *mut ffi::NewtonBody) -> Self {
        Self {
            newton,
            body,
            owned: true,
        }
    }

    pub fn as_handle(&self) -> &HandleOld {
        unsafe { mem::transmute(self.body) }
    }

    pub fn name(&self) -> Option<&'static str> {
        unsafe {
            let udata = userdata(self.as_ptr());
            let name = udata.name.clone();
            name
        }
    }

    /// Returns underlying NewtonBody pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.body
    }
}

/// FFI wrappers
impl<'world> BodyOld<'world> {
    pub fn set_sleep_state(&self, state: SleepState) {
        unsafe {
            let state = mem::transmute(state);
            ffi::NewtonBodySetSleepState(self.as_ptr(), state);
        }
    }

    pub fn active(&self) {
        self.set_sleep_state(SleepState::Active)
    }

    pub fn asleep(&self) {
        self.set_sleep_state(SleepState::Sleeping)
    }

    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.as_ptr())) }
    }

    pub fn body_type(&self) -> BodyType {
        unsafe { mem::transmute(ffi::NewtonBodyGetType(self.as_ptr())) }
    }

    pub fn aabb(&self) -> (Vector, Vector) {
        let mut aabb: (Vector, Vector) = Default::default();
        unsafe {
            ffi::NewtonBodyGetAABB(self.as_ptr(), aabb.0.as_mut_ptr(), aabb.1.as_mut_ptr());
        }
        aabb
    }

    pub fn matrix(&self) -> Matrix {
        unsafe {
            let mut matrix: Matrix = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.as_ptr(), matrix.as_mut_ptr() as *const f32);
            matrix
        }
    }

    pub fn velocity(&self) -> Vector {
        let mut vel = Vector::zero();
        unsafe { ffi::NewtonBodyGetVelocity(self.as_ptr(), vel.as_mut_ptr()) }
        vel
    }

    pub fn position(&self) -> Vector {
        let mut vel = Vector::zero();
        unsafe { ffi::NewtonBodyGetPosition(self.as_ptr(), vel.as_mut_ptr()) }
        vel
    }

    pub fn set_matrix(&self, mat: &Matrix) {
        unsafe { ffi::NewtonBodySetMatrix(self.as_ptr(), mat.as_ptr()) }
    }

    /// Sets the mass using the given collision to compute the inertia
    /// matrix components.
    pub fn set_mass(&self, mass: f32, collision: &CollisionOld) {
        unsafe { ffi::NewtonBodySetMassProperties(self.as_ptr(), mass, collision.as_raw()) }
    }

    /// Sets the mass matrix of a rigid body from its principal inertial components.
    pub fn set_mass_matrix(&self, mass: f32, ine: &Vector) {
        unsafe { ffi::NewtonBodySetMassMatrix(self.as_ptr(), mass, ine.x, ine.y, ine.z) }
    }

    /// Sets the mass matrix of the rigid body.
    pub fn set_full_mass_matrix(&self, mass: f32, matrix: &Matrix) {
        unsafe { ffi::NewtonBodySetFullMassMatrix(self.as_ptr(), mass, matrix.as_ptr() as _) }
    }

    pub fn set_force(&self, force: &Vector) {
        unsafe { ffi::NewtonBodySetForce(self.as_ptr(), force.as_ptr()) }
    }

    pub fn collision(&self) -> CollisionOld {
        CollisionOld {
            newton: self.newton,
            collision: unsafe { ffi::NewtonBodyGetCollision(self.as_ptr()) },
            owned: false,
        }
    }
}

unsafe fn userdata<'body>(ptr: *const ffi::NewtonBody) -> &'body Box<BodyData> {
    mem::transmute(&ffi::NewtonBodyGetUserData(ptr))
}

unsafe fn userdata_mut<'body>(ptr: *const ffi::NewtonBody) -> &'body mut Box<BodyData> {
    mem::transmute(&mut ffi::NewtonBodyGetUserData(ptr))
}
