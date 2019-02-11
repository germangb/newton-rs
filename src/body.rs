use std::mem;
use std::ptr::NonNull;

use super::collision::Collision;
use super::world::Newton;

use super::ffi;
use super::{Matrix, Vector};

#[derive(Debug)]
pub struct Builder {
    /// Initial body transform
    ///
    /// By default, it is set to an identity matrix.
    transform: Matrix,
}

impl Builder {
    pub fn transform(&mut self, transform: Matrix) -> &mut Self {
        self.transform = transform;
        self
    }

    pub fn build_dynamic<'world>(
        &self,
        newton: &'world Newton,
        collision: &Collision<'world>,
    ) -> Body<'world> {
        unsafe {
            let transform = self.transform.as_ptr() as _;
            let body = ffi::NewtonCreateDynamicBody(newton.as_ptr(), collision.as_ptr(), transform);

            let udata = Box::new(BodyData::default());
            ffi::NewtonBodySetUserData(body, mem::transmute(udata));

            Body {
                newton,
                body: NonNull::new_unchecked(body),
                owned: true,
            }
        }
    }
}

#[derive(Default)]
pub struct BodyData {
    /// Force and torque callback
    force_and_torque: Option<Box<dyn FnMut(Body)>>,
}

/// NewtonBody Wrapper
///
/// ## Notes
///
/// The ForceAndTorque callback closure is stored in the heap and referenced in the userdatum
/// of the NewtonBody object.
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
    pub fn builder() -> Builder {
        Builder {
            transform: super::identity(),
        }
    }

    /// Transfer ownership of this body to the Newton context.
    ///
    /// The handle can later be used to retrieve the original `Body` by calling the `body,
    /// `body_mut`, and `body_owned`, in the `Newton` type.
    pub fn into_handle(mut self) -> BodyHandle {
        self.newton.move_body(self)
    }

    pub fn set_force_and_torque_callback<F: FnMut(Body) + 'static>(&self, callback: F) {
        unsafe {
            let mut udata: Box<BodyData> =
                Box::from_raw(ffi::NewtonBodyGetUserData(self.as_ptr()) as _);

            udata.force_and_torque = Some(Box::new(callback));

            mem::forget(udata);
        }
    }

    /// Returns underlying NewtonBody pointer
    pub const fn as_ptr(&self) -> *const ffi::NewtonBody {
        self.body.as_ptr()
    }

    pub fn set_force(&self, force: &Vector) {
        unsafe { ffi::NewtonBodySetForce(self.as_ptr(), force.as_ptr()) }
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
                let _: Box<BodyData> =
                    Box::from_raw(ffi::NewtonBodyGetUserData(self.as_ptr()) as _);
                ffi::NewtonDestroyBody(self.as_ptr());
            }
        }
    }
}
