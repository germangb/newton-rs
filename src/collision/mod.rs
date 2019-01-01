pub mod params;

use ffi;

use self::params::Params;
use super::lock::{Lock, LockError, Locked, LockedMut, Shared, Weak};
use super::world::{NewtonWorld, WorldLockedMut};
use super::{Matrix, Quaternion, Result, Vector};

use self::params::HeightFieldParams;

use std::{
    mem,
    ops::{Deref, DerefMut},
    os::raw,
    ptr, slice,
};

/// Numeric ID type to identity collision faces in callbacks
pub type ShapeId = raw::c_int;

#[derive(Debug, Clone)]
pub struct Collision<B, C>(
    Shared<Lock<NewtonCollision<B, C>>>,
    Shared<Lock<NewtonWorld<B, C>>>,
    *const ffi::NewtonCollision,
);

unsafe impl<B, C> Send for Collision<B, C> {}
unsafe impl<B, C> Sync for Collision<B, C> {}

#[derive(Debug, Clone)]
pub struct WeakCollision<B, C>(
    Weak<Lock<NewtonCollision<B, C>>>,
    Weak<Lock<NewtonWorld<B, C>>>,
    *const ffi::NewtonCollision,
);

unsafe impl<B, C> Send for WeakCollision<B, C> {}
unsafe impl<B, C> Sync for WeakCollision<B, C> {}

impl<B, C> WeakCollision<B, C> {
    fn upgrade(weak: &WeakCollision<B, C>) -> Option<Collision<B, C>> {
        let coll = Weak::upgrade(&weak.0);
        if coll.is_some() {
            let world = Weak::upgrade(&weak.1);
            if world.is_some() {
                return Some(Collision(coll.unwrap(), world.unwrap(), weak.2));
            }
        }
        None
    }
}

/// Reference to a `NewtonCollision`.
#[derive(Debug)]
pub struct CollisionLocked<'a, B, C>(Locked<'a, NewtonCollision<B, C>>);

/// Mutable reference to a `NewtonCollision`.
#[derive(Debug)]
pub struct CollisionLockedMut<'a, B, C>(
    LockedMut<'a, NewtonCollision<B, C>>,
    LockedMut<'a, NewtonWorld<B, C>>,
);

#[derive(Debug)]
pub struct NewtonCollision<B, C> {
    // TODO remove pub(crate) visibility
    pub(crate) collision: *mut ffi::NewtonCollision,
    /// We need all collisions to `drop` before the `NewtonWorld` is.
    world: Shared<Lock<NewtonWorld<B, C>>>,
    /// If `owned` is set to true, the `NewtonCollision` will get destroyed
    owned: bool,
}

#[doc(hidden)]
#[derive(Debug)]
pub struct CollisionData<B, C> {
    /// Reference to the `NewtonWorld` that allocated this `NewtonCollision`
    world: Weak<Lock<NewtonWorld<B, C>>>,
    /// Reference to the `NewtonCollision` itself.
    collision: Weak<Lock<NewtonCollision<B, C>>>,
    /// Debug name
    debug: Option<&'static str>,
    /// collision parameters
    params: Params,
    /// Contained data
    contained: Option<C>,
}

impl<B, C> CollisionData<B, C> {
    /// Get collision data from a given collision
    ///
    /// Collision data is unique to the NewtonCollision object
    pub fn from(collision: &NewtonCollision<B, C>) -> Shared<Self> {
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

pub struct Builder<'a, B, C> {
    world: &'a mut NewtonWorld<B, C>,
    /// Collision shape params
    params: Params,
    // TODO docs
    shape_id: ShapeId,
    /// Collision offset transform
    offset: Option<Matrix>,
    /// A name given to the collision.
    debug: Option<&'static str>,
    /// Contained data
    contained: Option<C>,
}

impl<'a, B, C> Builder<'a, B, C> {
    pub fn new(world: &mut NewtonWorld<B, C>) -> Builder<B, C> {
        Builder {
            world,
            params: Params::Null,
            shape_id: 0,
            debug: None,
            offset: None,
            contained: None,
        }
    }

    /// Consumes the builder and returns a collision
    pub fn build(self) -> Collision<B, C> {
        Collision::new(
            self.world,
            self.params,
            self.shape_id,
            self.offset.as_ref(),
            self.debug,
            self.contained,
        )
    }

    pub fn data(mut self, data: C) -> Self {
        self.contained = Some(data);
        self
    }

    pub fn offset(mut self, offset: Matrix) -> Self {
        self.offset = Some(offset);
        self
    }

    /// Set a descriptive name
    pub fn debug(mut self, name: &'static str) -> Self {
        self.debug = Some(name);
        self
    }

    pub fn shape_id(mut self, shape_id: ShapeId) -> Self {
        self.shape_id = shape_id;
        self
    }

    pub fn params(mut self, params: Params) -> Self {
        self.params = params;
        self
    }

    pub fn capsule(mut self, radius0: f32, radius1: f32, height: f32) -> Self {
        self.params(Params::Capsule(radius0, radius1, height))
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

impl<B, C> Collision<B, C> {
    /// Clone ref-counted collision
    pub fn clone(rc: &Collision<B, C>) -> Collision<B, C> {
        Collision(Shared::clone(&rc.0), Shared::clone(&rc.1), rc.2)
    }

    /// Downgrade from a strong reference to a Weak one
    pub fn downgrade(rc: &Collision<B, C>) -> WeakCollision<B, C> {
        WeakCollision(Shared::downgrade(&rc.0), Shared::downgrade(&rc.1), rc.2)
    }

    pub unsafe fn from_raw_parts(raw: *mut ffi::NewtonCollision) -> Collision<B, C> {
        unimplemented!()
    }

    /// Creates a new collision.
    fn new(
        world: &mut NewtonWorld<B, C>,
        params: Params,
        shape_id: ShapeId,
        offset: Option<&Matrix>,
        debug: Option<&'static str>,
        data: Option<C>,
    ) -> Self {
        let world = world.as_raw();
        let world_udata = unsafe { super::world::userdata(world) };
        let world_lock = Weak::upgrade(&world_udata.world).unwrap();

        let collision_raw = unsafe {
            let offset = mem::transmute(offset);

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
                    let field = ffi::NewtonCreateHeightFieldCollision(
                        world,
                        h.rows() as _,            // width
                        h.columns() as _,         // height
                        mem::transmute(h.grid()), // gridsDiagonals
                        FIELD_F32_TYPE,           // elevationdatType
                        h.elevation().as_ptr() as *const raw::c_void,
                        h.ids().as_ptr(),
                        s_y, // verticalScale
                        s_x, // horizontalScale_x
                        s_z, // horizontalScale_z
                        shape_id,
                    );
                    ffi::NewtonCollisionSetMatrix(field, offset);
                    field
                }
                &Params::HeightFieldU16(ref h) => {
                    let (s_x, s_y, s_z) = h.scale();
                    let field = ffi::NewtonCreateHeightFieldCollision(
                        world,
                        h.rows() as _,            // width
                        h.columns() as _,         // height
                        mem::transmute(h.grid()), // gridsDiagonals
                        FIELD_U16_TYPE,           // elevationdatType
                        h.elevation().as_ptr() as *const raw::c_void,
                        h.ids().as_ptr(),
                        s_y, // verticalScale
                        s_x, // horizontalScale_x
                        s_z, // horizontalScale_z
                        shape_id,
                    );
                    ffi::NewtonCollisionSetMatrix(field, offset);
                    field
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
            contained: data,
        });

        unsafe {
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(userdata));
        }

        Collision(collision_lock, world_lock, collision_raw)
    }

    pub fn try_read(&self) -> Result<CollisionLocked<B, C>> {
        unimplemented!()
    }

    pub fn try_write(&self) -> Result<CollisionLockedMut<B, C>> {
        #[cfg(feature = "debug")]
        {
            let udata = unsafe { userdata::<B, C>(self.2) };
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

    pub fn read(&self) -> CollisionLocked<B, C> {
        let collision_ref = self.0.read();
        CollisionLocked(collision_ref)
    }

    pub fn write(&self) -> CollisionLockedMut<B, C> {
        self.try_write().unwrap()
    }
}

impl<B, C> NewtonCollision<B, C> {
    pub(crate) unsafe fn null(world: Shared<Lock<NewtonWorld<B, C>>>) -> Self {
        NewtonCollision {
            owned: false,
            collision: ptr::null_mut(),
            world,
        }
    }

    /// Wraps a raw `ffi::NewtonCollision` pointer
    pub(crate) unsafe fn new_not_owned(collision: *mut ffi::NewtonCollision) -> Self {
        let udata = userdata(collision);
        NewtonCollision {
            owned: false,
            collision,
            world: Weak::upgrade(&udata.world).unwrap(),
        }
    }

    /// Sets the offset transformation
    pub fn set_matrix(&mut self, matrix: &Matrix) {
        unsafe {
            ffi::NewtonCollisionSetMatrix(self.collision, mem::transmute(matrix));
        }
    }

    /// Offset transformation
    pub fn matrix(&self) -> Matrix {
        unsafe {
            let mut mat: Matrix = mem::zeroed();
            ffi::NewtonCollisionGetMatrix(self.collision, mem::transmute(&mut mat));
            mat
        }
    }

    /// Calls the given closure for each polygon in the collision shape.
    ///
    /// Use this method to display the geometry of a collision:
    pub fn polygons<F: FnMut(ShapeId, &[f32])>(&self, matrix: &Matrix, mut call: F) {
        unsafe {
            ffi::NewtonCollisionForEachPolygonDo(
                self.collision,
                mem::transmute(matrix),
                Some(callback::<F>),
                mem::transmute(&mut call),
            );
        }

        unsafe extern "C" fn callback<F: FnMut(ShapeId, &[f32])>(
            user_data: *const raw::c_void,
            vertex_count: raw::c_int,
            face_array: *const f32,
            face_id: raw::c_int,
        ) {
            let slice = slice::from_raw_parts(face_array, vertex_count as usize * 3);
            mem::transmute::<_, &mut F>(user_data)(face_id, slice);
        }
    }

    pub fn params(&self) -> &Params {
        unimplemented!()
    }

    pub fn as_raw(&self) -> *const ffi::NewtonCollision {
        self.collision as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonCollision {
        self.collision
    }

    /// Sets the collision scale
    pub fn set_scale(&mut self, x: f32, y: f32, z: f32) {
        unsafe { ffi::NewtonCollisionSetScale(self.collision, x, y, z) };
    }

    /// Collision scale
    pub fn scale(&self) -> (f32, f32, f32) {
        let mut scale = (0.0, 0.0, 0.0);
        unsafe {
            ffi::NewtonCollisionGetScale(self.collision, &mut scale.0, &mut scale.1, &mut scale.2)
        };
        scale
    }
}

impl<'a, B, C> Deref for CollisionLocked<'a, B, C> {
    type Target = NewtonCollision<B, C>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, B, C> Deref for CollisionLockedMut<'a, B, C> {
    type Target = NewtonCollision<B, C>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<'a, B, C> DerefMut for CollisionLockedMut<'a, B, C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

// TODO FIXME CollisionData is dropped but the collision may still be active in some body!!!
impl<B, C> Drop for NewtonCollision<B, C> {
    fn drop(&mut self) {
        if self.owned {
            let collision = self.collision;
            //let _ = self.world.write();
            unsafe {
                // TODO FIXME CollisionData is dropped but the collision may still be active in some body!!!
                let _: Shared<CollisionData<B, C>> =
                    mem::transmute(ffi::NewtonCollisionGetUserData(collision));
                ffi::NewtonDestroyCollision(collision)
            }
        }
    }
}

pub(crate) unsafe fn userdata<B, C>(
    col: *const ffi::NewtonCollision,
) -> Shared<CollisionData<B, C>> {
    let udata: Shared<CollisionData<B, C>> = mem::transmute(ffi::NewtonCollisionGetUserData(col));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}
