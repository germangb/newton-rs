use std::marker::PhantomData;
use std::mem;
use std::os::raw;
use std::ptr::NonNull;
use std::slice;

use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

/// A handle to a collision that is owned by the underlying NewtonWorld.
///
/// This type is meant for long-lived collisions. The underlying NewtonCollision
/// is accessible through [`collision`][collision] and [`collision_mut`][collision_mut]
///
/// [collision]: #
/// [collision_mut]: #
#[derive(Debug, Hash, Eq, PartialEq, Clone)]
pub struct Handle(pub(crate) NonNull<ffi::NewtonCollision>);

impl Handle {
    /// Returns the underlying NewtonCollision pointer.
    pub const fn as_ptr(&self) -> *const ffi::NewtonCollision {
        self.0.as_ptr()
    }
}

/// NewtonCollision wrapper.
///
/// Collisions in Rust are borrow types of the Newton context.
#[derive(Debug)]
pub struct Collision<'world> {
    pub(crate) _phantom: PhantomData<&'world ()>,
    /// Underlying NewtonCollision pointer
    ///
    /// This is the pointer passed to all calls to `NewtonCollision*` API functions.
    pub(crate) collision: NonNull<ffi::NewtonCollision>,
    /// If owned is set to true, the underlying NewtonCollision
    /// will be dropped along with this type.
    pub(crate) owned: bool,
}

impl<'world> Collision<'world> {
    /// Wraps a raw NewtonCollision pointer and returns a **non-owned** Collision.
    ///
    /// Being non-owned means the wrapped pointer won't be freed when the body is
    /// dropped. If you intend it to do so, call the `from_raw_owned` method instead.
    pub unsafe fn from_raw(collision: *mut ffi::NewtonCollision) -> Self {
        let mut collision = Self::from_raw_owned(collision);
        collision.owned = false;
        collision
    }

    pub unsafe fn from_raw_owned(collision: *mut ffi::NewtonCollision) -> Self {
        Self {
            collision: NonNull::new_unchecked(collision),
            owned: true,
            _phantom: PhantomData,
        }
    }

    unsafe fn from_ptr(newton: &'world Newton, ptr: *mut ffi::NewtonCollision) -> Self {
        Self {
            _phantom: PhantomData,
            collision: NonNull::new_unchecked(ptr),
            owned: true,
        }
    }

    /// Creates a sphere collision
    pub fn sphere(newton: &'world Newton, radius: f32, offset: Option<&Matrix>) -> Self {
        unsafe {
            let offset = mem::transmute(offset);
            let collision = ffi::NewtonCreateSphere(newton.as_ptr(), radius, 0, offset);
            Self::from_ptr(newton, collision)
        }
    }

    // Box collision
    pub fn box2(
        newton: &'world Newton,
        dx: f32,
        dy: f32,
        dz: f32,
        offset: Option<&Matrix>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset);
            let collision = ffi::NewtonCreateBox(newton.as_ptr(), dx, dy, dz, 0, offset);
            Self::from_ptr(newton, collision)
        }
    }

    /// Null collision
    pub fn null(newton: &'world Newton, dx: f32, dy: f32, dz: f32) -> Self {
        unsafe { Self::from_ptr(newton, ffi::NewtonCreateNull(newton.as_ptr())) }
    }

    /// Transfer ownership to the Newton context and return a handle to access
    /// the collision later.
    ///
    /// Method intended to be used for long-lived bodies.
    pub fn into_handle(mut self, newton: &Newton) -> Handle {
        self.owned = false;
        let mut set = newton.collisions.write().unwrap();
        let handle = Handle(self.collision);
        set.insert(handle.clone());
        handle
    }

    /// Returns underlying NewtonCollision pointer
    pub const fn as_ptr(&self) -> *const ffi::NewtonCollision {
        self.collision.as_ptr()
    }

    pub fn for_each_poly<F: FnMut(&[f32], raw::c_int)>(&self, matrix: &Matrix, mut callback: F) {
        unsafe {
            let udata = mem::transmute(&mut callback);
            ffi::NewtonCollisionForEachPolygonDo(
                self.as_ptr(),
                matrix.as_ptr() as _,
                Some(iterator::<F>),
                udata,
            );
        }

        unsafe extern "C" fn iterator<F: FnMut(&[f32], raw::c_int)>(
            udata: *const raw::c_void,
            vert_count: raw::c_int,
            face_array: *const f32,
            face_id: raw::c_int,
        ) {
            let slice = slice::from_raw_parts(face_array, vert_count as usize * 3);
            mem::transmute::<_, &mut F>(udata)(slice, face_id);
        }
    }

    pub fn set_scale(&self, scale: &Vector) {
        unsafe {
            ffi::NewtonCollisionSetScale(self.as_ptr(), scale[0], scale[1], scale[2]);
        }
    }

    pub fn set_matrix(&self, mat: &Matrix) {
        unsafe {
            ffi::NewtonCollisionSetMatrix(self.as_ptr(), mat.as_ptr() as _);
        }
    }

    pub fn matrix(&self) -> Matrix {
        unsafe {
            let mut matrix: Matrix = mem::zeroed();
            ffi::NewtonCollisionGetMatrix(self.as_ptr(), matrix.as_mut_ptr() as *const f32);
            matrix
        }
    }

    pub fn scale(&self) -> Vector {
        let mut s = [0.0, 0.0, 0.0];
        unsafe {
            ffi::NewtonCollisionGetScale(self.as_ptr(), &mut s[0], &mut s[1], &mut s[2]);
        }
        s
    }
}

impl<'world> Drop for Collision<'world> {
    fn drop(&mut self) {
        if self.owned {
            unsafe { ffi::NewtonDestroyCollision(self.as_ptr()) }
        }
    }
}
