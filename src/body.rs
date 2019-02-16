use std::mem;
use std::time::Duration;

use super::collision::Collision;
use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

#[repr(i32)]
pub enum SleepState {
    Active = 0,
    Sleeping = 1,
}

#[derive(Default)]
struct BodyData {
    /// Force and torque callback
    force_and_torque: Option<Box<dyn Fn(Body, Duration)>>,
}

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
pub struct Handle(pub(crate) *const ffi::NewtonBody);

#[derive(Debug)]
pub struct Bodies<'world>(pub(crate) &'world Newton);

pub struct Iter<'world> {
    newton: &'world Newton,
    next: *const ffi::NewtonBody,
}

impl<'world> Iterator for Iter<'world> {
    type Item = Body<'world>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.next;
        if current.is_null() {
            None
        } else {
            self.next = unsafe { ffi::NewtonWorldGetNextBody(self.newton.as_ptr(), current) };
            Some(Body {
                newton: self.newton,
                body: current,
                owned: false,
            })
        }
    }
}

impl<'world> Bodies<'world> {
    pub fn count(&self) -> usize {
        unsafe { ffi::NewtonWorldGetBodyCount(self.0.as_ptr()) as usize }
    }

    pub fn iter(&self) -> Iter {
        Iter {
            newton: self.0,
            next: unsafe { ffi::NewtonWorldGetFirstBody(self.0.as_ptr()) },
        }
    }

    pub fn get(&self, handle: &Handle) -> Option<Body> {
        self.0.body(handle)
    }
}

/// NewtonBody Wrapper
///
/// The borrow type will prevent the underlying NewtonWorld pointer from
/// outliving the NewtonWorld.
pub struct Body<'world> {
    pub(crate) newton: &'world Newton,
    /// The wrapped NewtonBody pointer
    ///
    /// This is the pointer passed to all calls to `NewtonBody*` API functions.
    pub(crate) body: *const ffi::NewtonBody,

    /// If owned is set to true, the underlying NewtonBody
    /// will be dropped along with this type.
    pub(crate) owned: bool,
}

impl<'world> Body<'world> {
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

    /// Transfer ownership to the Newton context and return a handle to access
    /// the body later.
    ///
    /// Method intended to be used for long-lived bodies.
    pub fn into_handle(self) -> Handle {
        self.newton.move_body(self)
    }

    /// Creates a dynamic body
    /// Calls `NewtonCreateDynamicBody` with the given collision and transform.
    #[inline]
    pub fn dynamic(newton: &'world Newton, collision: &Collision, transform: &Matrix) -> Self {
        unsafe {
            let transform = mem::transmute(transform);
            let body = ffi::NewtonCreateDynamicBody(newton.as_ptr(), collision.as_ptr(), transform);
            ffi::NewtonBodySetUserData(body, mem::transmute(Box::new(BodyData::default())));

            Self {
                newton,
                body,
                owned: true,
            }
        }
    }

    /// Returns underlying NewtonBody pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.body
    }
}

/// FFI wrappers
impl<'world> Body<'world> {
    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.as_ptr())) }
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

    /// Sets the mass using the given collision to compute the inertia
    /// matrix components.
    pub fn set_mass(&self, mass: f32, collision: &Collision) {
        unsafe { ffi::NewtonBodySetMassProperties(self.as_ptr(), mass, collision.as_ptr()) }
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

    pub fn collision(&self) -> Collision {
        Collision {
            newton: self.newton,
            collision: unsafe { ffi::NewtonBodyGetCollision(self.as_ptr()) },
            owned: false,
        }
    }

    pub fn set_force_and_torque_callback<F: Fn(Body, Duration) + 'static>(&self, callback: F) {
        unsafe {
            let mut udata = userdata(self.as_ptr());
            udata.force_and_torque = Some(Box::new(callback));
            mem::forget(udata);

            ffi::NewtonBodySetForceAndTorqueCallback(self.as_ptr(), Some(force_and_torque));
        }

        unsafe extern "C" fn force_and_torque(
            body: *const ffi::NewtonBody,
            timestep: f32,
            _thread_idx: std::os::raw::c_int,
        ) {
            let seconds = timestep.floor() as u64;
            let nanos = (timestep.fract() * 1_000_000_000.0) as u32;
            let timestep = Duration::new(seconds, nanos);

            let udata = userdata(body);
            if let Some(callback) = &udata.force_and_torque {
                let newton = Newton(ffi::NewtonBodyGetWorld(body));
                callback(
                    Body {
                        newton: &newton,
                        body,
                        owned: false,
                    },
                    timestep,
                );
                newton.leak();
            }
            mem::forget(udata);
        }
    }
}

impl<'world> Drop for Body<'world> {
    fn drop(&mut self) {
        if self.owned {
            unsafe {
                // drop udata
                let _ = userdata(self.as_ptr());
                ffi::NewtonDestroyBody(self.as_ptr());
            }
        }
    }
}

unsafe fn userdata(ptr: *const ffi::NewtonBody) -> Box<BodyData> {
    Box::from_raw(ffi::NewtonBodyGetUserData(ptr) as _)
}
