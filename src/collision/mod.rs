pub mod params;

use ffi;

use self::params::Params;
use super::world::{NewtonWorld, WorldLockedMut};
use super::{Lock, Locked, LockedMut, Result, Shared, Types, Weak};

use self::params::HeightFieldParams;

use std::{
    mem,
    ptr,
    ops::{Deref, DerefMut},
    os::raw,
};

#[derive(Debug, Clone)]
pub struct Collision<T>(
    Shared<Lock<NewtonCollision<T>>>,
    Shared<Lock<NewtonWorld<T>>>,
    *const ffi::NewtonCollision,
);

unsafe impl<T> Send for Collision<T> {}
unsafe impl<T> Sync for Collision<T> {}

#[derive(Debug)]
pub struct NewtonCollision<T> {
    // TODO remove pub(crate) visibility
    pub(crate) collision: *mut ffi::NewtonCollision,

    /// We need all collisions to `drop` before the `NewtonWorld` is.
    world: Shared<Lock<NewtonWorld<T>>>,
    /// If `owned` is set to true, the `NewtonCollision` will get destroyed
    owned: bool,
}

#[doc(hidden)]
#[derive(Debug)]
pub struct CollisionData<T> {
    /// Reference to the `NewtonWorld` that allocated this `NewtonCollision`
    world: Weak<Lock<NewtonWorld<T>>>,
    /// Reference to the `NewtonCollision` itself.
    collision: Weak<Lock<NewtonCollision<T>>>,

    /// Debug name
    debug: Option<&'static str>,

    /// collision parameters
    params: Params,
}

impl<T> CollisionData<T> {
    /// Get collision data from a given collision
    ///
    /// Collision data is unique to the NewtonCollision object
    pub fn from(collision: &NewtonCollision<T>) -> Shared<Self> {
        unsafe { userdata(collision.as_raw()) }
    }

    /// The debug name given to the collision on creation
    pub fn debug(&self) -> Option<&'static str> {
        self.debug
    }

    /// Collision params given to the collision on creation
    pub fn params(&self) -> &Params {
        &self.params
    }
}

/*
impl<T> Drop for CollisionData<T> {
    fn drop(&mut self) {
        println!("Dropping collision!");
    }
}
*/

pub struct Builder<'a, T: Types> {
    world: &'a mut NewtonWorld<T>,

    /// Collision shape params
    params: Params,
    // TODO docs
    shape_id: raw::c_int,
    /// Collision offset transform
    offset: Option<T::Matrix>,

    /// A name given to the collision.
    debug: Option<&'static str>,
}

impl<'a, T: Types> Builder<'a, T> {
    pub fn new(world: &mut NewtonWorld<T>) -> Builder<T> {
        Builder {
            world,
            params: Params::Null,
            shape_id: 0,
            debug: None,
            offset: None,
        }
    }

    /// Consumes the builder and returns a collision
    pub fn build(self) -> Collision<T> {
        Collision::new(self.world, self.params, self.shape_id, self.debug)
    }

    pub fn offset(mut self, offset: T::Matrix) -> Self {
        self.offset = Some(offset);
        self
    }

    /// Set a descriptive name
    pub fn debug(mut self, name: &'static str) -> Self {
        self.debug = Some(name);
        self
    }

    pub fn shape_id(mut self, shape_id: raw::c_int) -> Self {
        self.shape_id = shape_id;
        self
    }

    pub fn params(mut self, params: Params) -> Self {
        self.params = params;
        self
    }

    pub fn sphere(mut self, radius: f32) -> Self {
        self.params(Params::Sphere(radius))
    }

    /// Set Box collision params. `Volume = dx*dy*dz`
    pub fn cuboid(mut self, dx: f32, dy: f32, dz: f32) -> Self {
        self.params(Params::Box(dx, dy, dz))
    }

    pub fn heightfield_f32(mut self, params: HeightFieldParams<f32>) -> Self {
        self.params(Params::HeightFieldF32(params))
    }

    pub fn heightfield_u16(mut self, params: HeightFieldParams<u16>) -> Self {
        self.params(Params::HeightFieldU16(params))
    }
}

impl<T> NewtonCollision<T> {
    pub(crate) unsafe fn null(world: Shared<Lock<NewtonWorld<T>>>) -> Self {
        NewtonCollision {
            owned: false,
            collision: ptr::null_mut(),
            world,
        }
    }

    /// Wraps a raw `ffi::NewtonCollision` pointer
    pub(crate) unsafe fn new_not_owned(collision: *mut ffi::NewtonCollision) -> Self {
        let udata = userdata::<T>(collision);
        NewtonCollision {
            owned: false,
            collision,
            world: Weak::upgrade(&udata.world).unwrap(),
        }
    }
}

/// Reference to a `NewtonCollision`.
#[derive(Debug)]
pub struct CollisionLocked<'a, T>(Locked<'a, NewtonCollision<T>>);

/// Mutable reference to a `NewtonCollision`.
#[derive(Debug)]
pub struct CollisionLockedMut<'a, T>(
    LockedMut<'a, NewtonCollision<T>>,
    LockedMut<'a, NewtonWorld<T>>,
);

impl<T: Types> Collision<T> {
    /// Creates a new collision.
    pub fn new(
        world: &mut NewtonWorld<T>,
        params: Params,
        shape_id: raw::c_int,
        debug: Option<&'static str>,
    ) -> Self {
        let world = world.as_raw();
        let world_udata = unsafe { super::world::userdata::<T>(world) };
        let world_lock = Weak::upgrade(&world_udata.world).unwrap();

        let collision_raw = unsafe {
            let offset = std::ptr::null();

            const FIELD_F32_TYPE: i32 = 0;
            const FIELD_U16_TYPE: i32 = 1;

            match &params {
                &Params::Box(dx, dy, dz) => {
                    ffi::NewtonCreateBox(world, dx, dy, dz, shape_id, offset)
                }
                &Params::Sphere(radius) => ffi::NewtonCreateSphere(world, radius, shape_id, offset),
                &Params::Cone(radius, height) => {
                    ffi::NewtonCreateCone(world, radius, height, shape_id, offset)
                }
                &Params::Cylinder(radius0, radius1, height) => {
                    ffi::NewtonCreateCylinder(world, radius0, radius1, height, shape_id, offset)
                }
                &Params::Capsule(radius0, radius1, height) => {
                    ffi::NewtonCreateCapsule(world, radius0, radius1, height, shape_id, offset)
                }
                &Params::Null => ffi::NewtonCreateNull(world),
                &Params::HeightFieldF32(ref h) => {
                    let (s_x, s_y, s_z) = h.scale();
                    ffi::NewtonCreateHeightFieldCollision(
                        world,
                        h.rows() as _,    // width
                        h.columns() as _, // height
                        mem::transmute(h.grid()), // gridsDiagonals
                        FIELD_F32_TYPE,   // elevationdatType
                        h.elevation().as_ptr() as *const raw::c_void,
                        h.ids().as_ptr(),
                        s_y, // verticalScale
                        s_x, // horizontalScale_x
                        s_z, // horizontalScale_z
                        shape_id,
                    )
                }
                &Params::HeightFieldU16(ref h) => {
                    let (s_x, s_y, s_z) = h.scale();
                    ffi::NewtonCreateHeightFieldCollision(
                        world,
                        h.rows() as _,    // width
                        h.columns() as _, // height
                        mem::transmute(h.grid()), // gridsDiagonals
                        FIELD_U16_TYPE,   // elevationdatType
                        h.elevation().as_ptr() as *const raw::c_void,
                        h.ids().as_ptr(),
                        s_y, // verticalScale
                        s_x, // horizontalScale_x
                        s_z, // horizontalScale_z
                        shape_id,
                    )
                }
            }
        };

        let collision = NewtonCollision {
            world: world_lock.clone(),
            collision: collision_raw,
            owned: true,
        };

        let collision_lock = Shared::new(Lock::new(collision));

        let userdata = Shared::new(CollisionData {
            collision: Shared::downgrade(&collision_lock),
            world: Shared::downgrade(&world_lock),
            debug,
            params,
        });

        unsafe {
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(userdata));
        }

        Collision(collision_lock, world_lock, collision_raw)
    }

    pub fn try_read(&self) -> Result<CollisionLocked<T>> {
        unimplemented!()
    }

    pub fn try_write(&self) -> Result<CollisionLockedMut<T>> {
        #[cfg(feature = "debug")]
        {
            let udata = unsafe { userdata::<T>(self.2) };
            let collision_ref = self.0.try_write(udata.debug)?;
            let world_ref = self.1.try_write(udata.debug)?;
            Ok(CollisionLockedMut(collision_ref, world_ref))
        }
        #[cfg(not(feature = "debug"))]
        {
            let collision_ref = self.0.try_write(None)?;
            let world_ref = self.1.try_write(None)?;
            Ok(CollisionLockedMut(collision_ref, world_ref))
        }
    }

    pub fn read(&self) -> CollisionLocked<T> {
        let collision_ref = self.0.read();
        CollisionLocked(collision_ref)
    }

    pub fn write(&self) -> CollisionLockedMut<T> {
        self.try_write().unwrap()
    }
}

impl<T: Types> NewtonCollision<T> {
    pub fn set_matrix(&mut self, matrix: &T::Matrix) {
        unsafe {
            ffi::NewtonCollisionSetMatrix(self.collision, mem::transmute(matrix));
        }
    }

    pub fn matrix(&self) -> T::Matrix {
        unsafe {
            let mut mat: T::Matrix = mem::zeroed();
            ffi::NewtonCollisionGetMatrix(self.collision, mem::transmute(&mut mat));
            mat
        }
    }
}

impl<T> NewtonCollision<T> {
    pub fn params(&self) -> &Params {
        unimplemented!()
    }

    pub fn as_raw(&self) -> *const ffi::NewtonCollision {
        self.collision as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonCollision {
        self.collision
    }
}

impl<'a, T> Deref for CollisionLocked<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> Deref for CollisionLockedMut<'a, T> {
    type Target = NewtonCollision<T>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, T> DerefMut for CollisionLockedMut<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

// TODO FIXME CollisionData is dropped but the collision may still be active in some body!!!
impl<T> Drop for NewtonCollision<T> {
    fn drop(&mut self) {
        if self.owned {
            let collision = self.collision;
            //let _ = self.world.write();
            unsafe {
                // TODO FIXME CollisionData is dropped but the collision may still be active in some body!!!
                let _: Shared<CollisionData<T>> =
                    mem::transmute(ffi::NewtonCollisionGetUserData(collision));
                ffi::NewtonDestroyCollision(collision)
            }
        }
    }
}

pub(crate) unsafe fn userdata<T>(col: *const ffi::NewtonCollision) -> Shared<CollisionData<T>> {
    let udata: Shared<CollisionData<T>> = mem::transmute(ffi::NewtonCollisionGetUserData(col));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}
