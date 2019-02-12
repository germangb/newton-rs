use std::mem;
use std::ptr::NonNull;

use super::collision::Collision;
use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

#[derive(Default)]
pub struct BodyData {
    /// Force and torque callback
    force_and_torque: Option<Box<dyn FnMut(Body)>>,
}

/// NewtonBody Wrapper
///
/// ## Notes
/// Some information about the body is stored in the Heap and accessible through the
/// usedata pointer.
///
/// * ForceAndTorqueCallback
pub struct Body<'world> {
    pub(crate) newton: &'world Newton,
    /// The wrapped NewtonBody pointer
    ///
    /// This is the pointer passed to all calls to `NewtonBody*` API functions.
    pub(crate) body: NonNull<ffi::NewtonBody>,
    /// If owned is set to true, the underlying NewtonBody
    /// will be dropped along with this type.
    pub(crate) owned: bool,
}

impl<'world> Body<'world> {
    /// Transfer ownership to the Newton context and return a handle to access
    /// the body later.
    ///
    /// Method intended to be used for long-lived bodies.
    pub fn into_handle(mut self) -> BodyHandle {
        self.owned = false;
        let mut set = self.newton.bodies.write().unwrap();
        let handle = BodyHandle(self.body);
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
                newton,
                body: NonNull::new_unchecked(body),
                owned: true,
            }
        }
    }

    /// Sets the mass using the given collision to compute the inertia
    /// matrix components.
    #[inline]
    pub fn set_mass(&self, mass: f32, collision: &Collision) {
        unsafe { ffi::NewtonBodySetMassProperties(self.as_ptr(), mass, collision.as_ptr()) }
    }

    pub fn set_force_and_torque_callback<F: FnMut(Body) + 'static>(&self, callback: F) {
        unsafe {
            let mut udata = self.userdata();
            udata.force_and_torque = Some(Box::new(callback));
            mem::forget(udata);
        }
    }

    /// Returns underlying NewtonBody pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.body.as_ptr()
    }

    #[inline]
    pub fn set_force(&self, force: &Vector) {
        unsafe { ffi::NewtonBodySetForce(self.as_ptr(), force.as_ptr()) }
    }

    unsafe fn userdata(&self) -> Box<BodyData> {
        Box::from_raw(ffi::NewtonBodyGetUserData(self.as_ptr()) as _)
    }
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
pub struct BodyHandle(pub NonNull<ffi::NewtonBody>);

impl BodyHandle {
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.0.as_ptr()
    }
}

impl<'world> Drop for Body<'world> {
    fn drop(&mut self) {
        if self.owned {
            unsafe {
                // drop udata
                let _ = self.userdata();

                ffi::NewtonDestroyBody(self.as_ptr());
            }
        }
    }
}
