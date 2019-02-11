use ffi;

use std::marker::PhantomData;
use std::os::raw;
use std::ptr::NonNull;

/// Material group ID
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct GroupId(pub(crate) raw::c_int);

impl GroupId {
    #[inline]
    pub fn as_raw(&self) -> raw::c_int {
        self.0
    }
}

#[derive(Debug, Clone)]
pub struct NewtonMaterial(*mut ffi::NewtonMaterial);

impl NewtonMaterial {
    pub unsafe fn from_raw(mat: *mut ffi::NewtonMaterial) -> Self {
        NewtonMaterial(mat)
    }

    /// Get material as a raw pointer
    pub fn as_raw(&self) -> *const ffi::NewtonMaterial {
        self.0
    }
}

/// Trait to generate touples of `GroupId`s
pub trait Materials: Sized {
    unsafe fn from(world: NonNull<ffi::NewtonWorld>) -> Self;
}

impl NewtonMaterial {
    /// Override the default value of the kinetic coefficient of friction for this contact.
    pub fn set_friction(&mut self, static_coef: f32, kinematic_coef: f32, index: raw::c_int) {
        assert!(static_coef >= 0.0 && kinematic_coef >= 0.0);
        unsafe {
            ffi::NewtonMaterialSetContactFrictionCoef(self.0, static_coef, kinematic_coef, index)
        };
    }

    /// Sets restitution coefficient
    pub fn set_elasticity(&mut self, rest: f32) {
        assert!(rest >= 0.0);
        unsafe { ffi::NewtonMaterialSetContactElasticity(self.0, rest) };
    }
}

macro_rules! materials {
    ( $($gen:ident),+ ) => {
        impl< $($gen : Materials,)+ > Materials for ( $($gen,)+ ) {
            #[inline]
            unsafe fn from(world: NonNull<ffi::NewtonWorld>) -> Self {
                ($( <$gen as Materials>::from(world) ,)+)
            }
        }
    }
}

impl Materials for GroupId {
    #[inline]
    unsafe fn from(world: NonNull<ffi::NewtonWorld>) -> Self {
        GroupId(ffi::NewtonMaterialCreateGroupID(world.as_ptr()))
    }
}

materials! { A }
materials! { A, B }
materials! { A, B, C }
materials! { A, B, C, D }
materials! { A, B, C, D, E }
materials! { A, B, C, D, E, F }
materials! { A, B, C, D, E, F, G }
materials! { A, B, C, D, E, F, G, H }
materials! { A, B, C, D, E, F, G, H, I }
materials! { A, B, C, D, E, F, G, H, I, J }
materials! { A, B, C, D, E, F, G, H, I, J, K }
