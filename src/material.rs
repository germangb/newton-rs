use ffi;

use std::marker::PhantomData;
use std::os::raw;

/// Material group ID
pub type GroupId = raw::c_int;

#[derive(Debug, Clone)]
pub struct NewtonMaterial<T>(*mut ffi::NewtonMaterial, PhantomData<T>);

/// Trait to generate touples of `GroupId`s
pub trait Materials: Sized {
    unsafe fn from(world: *mut ffi::NewtonWorld) -> Self;
}

pub(crate) fn create_materials<T: Materials>(world: *mut ffi::NewtonWorld) -> T {
    unsafe { T::from(world) }
}

impl<T> NewtonMaterial<T> {
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
            unsafe fn from(world: *mut ffi::NewtonWorld) -> Self {
                ($( <$gen as Materials>::from(world) ,)+)
            }
        }
    }
}

impl Materials for GroupId {
    #[inline]
    unsafe fn from(world: *mut ffi::NewtonWorld) -> Self {
        ffi::NewtonMaterialCreateGroupID(world)
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
