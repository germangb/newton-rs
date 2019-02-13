use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;

use super::collision::Collision;
use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

// Type to store information about bodies on the Heap
#[doc(hidden)]
#[derive(Default)]
pub struct BodyData {
    /// Force and torque callback
    force_and_torque: Option<Box<dyn Fn(Body)>>,
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
#[derive(Debug, Hash, Eq, PartialEq, Clone)]
pub struct Handle(pub(crate) NonNull<ffi::NewtonBody>);

impl Handle {
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.0.as_ptr()
    }
}

/// NewtonBody Wrapper
///
/// The borrow type will prevent the underlying NewtonWorld pointer from
/// outliving the NewtonWorld.
pub struct Body<'world> {
    pub(crate) _phantom: PhantomData<&'world ()>,
    /// The wrapped NewtonBody pointer
    ///
    /// This is the pointer passed to all calls to `NewtonBody*` API functions.
    pub(crate) body: NonNull<ffi::NewtonBody>,
    /// If owned is set to true, the underlying NewtonBody
    /// will be dropped along with this type.
    pub(crate) owned: bool,
}

impl<'world> Body<'world> {
    /// Wraps a raw NewtonBody pointer and returns a **non-owned** Body.
    ///
    /// Being non-owned means the wrapped pointer won't be freed when the body is
    /// dropped. If you intend it to do so, call the `from_raw_owned` method instead.
    pub unsafe fn from_raw(body: *mut ffi::NewtonBody) -> Self {
        let mut body = Self::from_raw_owned(body);
        body.owned = false;
        body
    }

    pub unsafe fn from_raw_owned(body: *mut ffi::NewtonBody) -> Self {
        Self {
            body: NonNull::new_unchecked(body),
            owned: true,
            _phantom: PhantomData,
        }
    }

    /// Transfer ownership to the Newton context and return a handle to access
    /// the body later.
    ///
    /// Method intended to be used for long-lived bodies.
    pub fn into_handle(mut self, newton: &Newton) -> Handle {
        self.owned = false;
        let mut set = newton.bodies.write().unwrap();
        let handle = Handle(self.body);
        set.insert(handle.clone());
        handle
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
                _phantom: PhantomData,
                body: NonNull::new_unchecked(body),
                owned: true,
            }
        }
    }

    /// Returns underlying NewtonBody pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.body.as_ptr()
    }

    /// Sets the mass using the given collision to compute the inertia
    /// matrix components.
    #[inline]
    pub fn set_mass(&self, mass: f32, collision: &Collision) {
        unsafe { ffi::NewtonBodySetMassProperties(self.as_ptr(), mass, collision.as_ptr()) }
    }

    pub fn set_force_and_torque_callback<F: Fn(Body) + 'static>(&self, callback: F) {
        unsafe {
            let mut udata = userdata(self.as_ptr());
            udata.force_and_torque = Some(Box::new(callback));
            mem::forget(udata);

            ffi::NewtonBodySetForceAndTorqueCallback(self.as_ptr(), Some(force_and_torque));
        }

        unsafe extern "C" fn force_and_torque(
            body: *const ffi::NewtonBody,
            _timestep: f32,
            _thread_idx: std::os::raw::c_int,
        ) {
            let udata = userdata(body);
            if let Some(callback) = &udata.force_and_torque {
                callback(Body {
                    _phantom: PhantomData,
                    body: NonNull::new_unchecked(body as _),
                    owned: false,
                })
            }
            mem::forget(udata);
        }
    }

    #[inline]
    pub fn set_force(&self, force: &Vector) {
        unsafe { ffi::NewtonBodySetForce(self.as_ptr(), force.as_ptr()) }
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
