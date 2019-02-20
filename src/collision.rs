use std::mem;
use std::os::raw;

use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};
use super::Handle;
use std::marker::PhantomData;

macro_rules! collision {
    ($(
        $( #[ $($meta:meta)+ ] )*
        ($enum_var:ident , ffi::$id:ident ) => pub struct $collision:ident<'world>(...);
    )*) => {

    #[derive(Debug, Eq, PartialEq)]
    pub enum Collision<'world> {
        $($enum_var($collision<'world>)),*
    }

    impl<'world> Collision<'world> {
        pub unsafe fn from_raw(raw: *const ffi::NewtonCollision) -> Self {
            let col_type = ffi::NewtonCollisionGetType(raw);
            match col_type as _ {
                $(
                    ffi::$id => Collision::$enum_var($collision(raw, PhantomData)),
                )*
                _ => unimplemented!("Collision type ({}) not implemented", col_type),
            }
        }
    }

    impl<'world> CollisionTrait for Collision<'world> {
        fn as_raw(&self) -> *const ffi::NewtonCollision {
            match self {
                $(Collision::$enum_var(col) => col.0 ),*
            }
        }
    }

    $(
        $(#[$($meta)+])*
        pub struct $collision<'world>(*const ffi::NewtonCollision, PhantomData<&'world ()>);

        impl<'world> $collision<'world> {
            /// The returned collision owns itself, i.e. It will be freed when the instance
            /// is dropped.
            pub unsafe fn from_raw(newton: &'world Newton, raw: *const ffi::NewtonCollision) -> Self {
                $collision(raw, PhantomData)
            }

            pub(crate) unsafe fn set_owner(&mut self, owner: *const ()) {
                let mut udata = ffi::NewtonCollisionGetUserData(self.0);
                let mut udata: &mut Box<UserData> = mem::transmute(&mut udata);
                udata.owner = owner;
            }
        }

        impl<'world> Drop for $collision<'world> {
            fn drop(&mut self) {
                // if collision owns itself, then free the memory
                unsafe {
                    let udata = ffi::NewtonCollisionGetUserData(self.0);
                    let mut udata: Option<Box<UserData>> = mem::transmute(udata);
                    if let Some(inner) = udata.take() {
                        if inner.owner == self.0 as *const () {
                            ffi::NewtonDestroyCollision(self.0);
                        } else {
                            mem::forget(inner);
                        }
                    }
                }
            }
        }

        impl<'world> IntoCollision<'world> for $collision<'world> {
            fn into_collision(self) -> Collision<'world> {
                Collision::$enum_var(self)
            }
        }

        impl<'world> CollisionTrait for $collision<'world> {
            fn as_raw(&self) -> *const ffi::NewtonCollision {
                self.0
            }
        }

        impl<'a, 'world> CollisionTrait for &'a $collision<'world> {
            fn as_raw(&self) -> *const ffi::NewtonCollision {
                self.0
            }
        }
    )*}
}

pub(crate) struct UserData {
    /// A pointer to the structure that owns the Newton object.
    ///
    /// # Safety
    /// It must be ensured that this value always points to valid Newton memory,
    /// otherwise it will never be freed and could cause a memory leak!
    pub(crate) owner: *const (),

    /// An optional, human readable name given to the collision.
    name: Option<&'static str>,
}

collision! {
    /// Box collision object.
    #[derive(Debug, Eq, PartialEq)]
    (Box, ffi::SERIALIZE_ID_BOX) => pub struct BoxCollision<'world>(...);

    /// Sphere collision object.
    #[derive(Debug, Eq, PartialEq)]
    (Sphere, ffi::SERIALIZE_ID_SPHERE) => pub struct SphereCollision<'world>(...);

    /// Cylinder collision object.
    #[derive(Debug, Eq, PartialEq)]
    (Cylinder, ffi::SERIALIZE_ID_CYLINDER) => pub struct CylinderCollision<'world>(...);

    /// Capsule collision object.
    #[derive(Debug, Eq, PartialEq)]
    (Capsule, ffi::SERIALIZE_ID_CAPSULE) => pub struct CapsuleCollision<'world>(...);
}

impl<'world> BoxCollision<'world> {
    pub fn create(newton: &'world Newton,
                  x: f32,
                  y: f32,
                  z: f32,
                  offset: Option<[[f32; 4]; 4]>,
                  name: Option<&'static str>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateBox(newton.as_raw(), x, y, z, 0, offset);
            let udata = Box::new(UserData { owner: collision as _, name });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(newton, collision)
        }
    }
}

impl<'world> SphereCollision<'world> {
    pub fn create(newton: &'world Newton,
                  radius: f32,
                  offset: Option<[[f32; 4]; 4]>,
                  name: Option<&'static str>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateSphere(newton.as_raw(), radius, 0, offset);
            let udata = Box::new(UserData { owner: collision as _, name });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(newton, collision)
        }
    }
}

impl<'world> CylinderCollision<'world> {
    pub fn create(newton: &'world Newton,
                  radius0: f32,
                  radius1: f32,
                  height: f32,
                  offset: Option<[[f32; 4]; 4]>,
                  name: Option<&'static str>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateCylinder(newton.as_raw(), radius0, radius1, height, 0, offset);
            let udata = Box::new(UserData { owner: collision as _, name });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(newton, collision)
        }
    }
}

impl<'world> CapsuleCollision<'world> {
    pub fn create(newton: &'world Newton,
                  radius0: f32,
                  radius1: f32,
                  height: f32,
                  offset: Option<[[f32; 4]; 4]>,
                  name: Option<&'static str>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateCapsule(newton.as_raw(), radius0, radius1, height, 0, offset);
            let udata = Box::new(UserData { owner: collision as _, name });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(newton, collision)
        }
    }
}

pub trait CollisionTrait {
    fn as_raw(&self) -> *const ffi::NewtonCollision;

    fn into_handle<'w>(self, newton: &'w Newton) -> Handle
    where
        Self: Sized + IntoCollision<'w>,
    {
        newton.move_collision(self)
    }

    fn name(&self) -> Option<&'static str> {
        unsafe {
            let udata = ffi::NewtonCollisionGetUserData(self.as_raw());
            let udata: &Option<Box<UserData>> = mem::transmute(&udata);
            udata.as_ref().and_then(|d| d.name)
        }
    }

    fn for_each_polygon<F>(&self, matrix: [[f32; 4]; 4], mut callback: F)
    where
        F: FnMut(&[f32], raw::c_int),
    {
        unsafe {
            let udata = mem::transmute(&mut callback);
            let matrix = matrix[0].as_ptr();
            ffi::NewtonCollisionForEachPolygonDo(self.as_raw(), matrix, Some(iterator::<F>), udata);
        }

        unsafe extern "C" fn iterator<F>(
            udata: *const raw::c_void,
            vert_count: raw::c_int,
            face_array: *const f32,
            face_id: raw::c_int,
        )
        where
            F: FnMut(&[f32], raw::c_int),
        {
            let slice = std::slice::from_raw_parts(face_array, vert_count as usize * 3);
            mem::transmute::<_, &mut F>(udata)(slice, face_id);
        }
    }
}

pub trait IntoCollision<'world> {
    fn into_collision(self) -> Collision<'world>;
}

#[derive(Debug, Hash, Eq, PartialEq, Clone, Copy)]
pub struct HandleOld(pub(crate) *const ffi::NewtonCollision);

unsafe impl Send for HandleOld {}
unsafe impl Sync for HandleOld {}

#[derive(Debug)]
pub struct CollisionOld<'world> {
    pub(crate) newton: &'world Newton,
    /// Underlying NewtonCollision pointer
    ///
    /// This is the pointer passed to all calls to `NewtonCollision*`
    /// API functions.
    pub(crate) collision: *const ffi::NewtonCollision,
    /// If owned is set to true, the underlying NewtonCollision
    /// will be dropped along with this type.
    pub(crate) owned: bool,
}

unsafe impl<'world> Send for CollisionOld<'world> {}
unsafe impl<'world> Sync for CollisionOld<'world> {}

impl<'world> CollisionOld<'world> {
    fn from_ptr(newton: &'world Newton, collision: *mut ffi::NewtonCollision) -> Self {
        Self {
            newton,
            collision,
            owned: true,
        }
    }

    /// Returns the wrapped NewtonCollision.
    ///
    /// ```
    /// use newton::ffi;
    /// use newton::{Newton, Collision};
    ///
    /// let world = Newton::default();
    /// let collision = Collision::sphere(&world, 1.0, None);
    ///
    /// unsafe {
    ///     let collision = collision.as_ptr();
    /// }
    /// ```
    pub const fn as_raw(&self) -> *const ffi::NewtonCollision {
        self.collision
    }
}

/// FFI wrappers
impl<'world> CollisionOld<'world> {
    /// Inspects the geometry of the collision.
    ///
    /// The given closure gets called for every face of the collision geometry
    /// (transformed by some `matrix`).
    ///
    /// ## Performance
    /// According to Newton-dynamics docs, this method is really slow and should
    /// only be used to render debug information.
    ///
    /// ## Example
    /// ```
    /// use newton::{Newton, Collision};
    ///
    /// let newton = Newton::default();
    /// let collision = Collision::sphere(&world, 1.0, None);
    ///
    /// let ident = newton::math::identity();
    /// collision.polygons(&identity, |face, face_id| {
    ///     let normal = compute_normal(face);
    ///     render_face(face, &normal);
    /// })
    ///
    /// # fn render_face(face: &[f32], &normal) {}
    /// # fn compute_normal(face: &[f32]) {}
    /// ```
    pub fn polygons<F: FnMut(&[f32], raw::c_int)>(&self, matrix: &Matrix, mut callback: F) {
        unsafe {
            let udata = mem::transmute(&mut callback);
            let matrix = matrix.as_ptr() as _;
            ffi::NewtonCollisionForEachPolygonDo(self.as_raw(), matrix, Some(iterator::<F>), udata);
        }

        unsafe extern "C" fn iterator<F: FnMut(&[f32], raw::c_int)>(
            udata: *const raw::c_void,
            vert_count: raw::c_int,
            face_array: *const f32,
            face_id: raw::c_int,
        ) {
            let slice = std::slice::from_raw_parts(face_array, vert_count as usize * 3);
            mem::transmute::<_, &mut F>(udata)(slice, face_id);
        }
    }

    pub fn set_scale(&self, s: &Vector) {
        unsafe { ffi::NewtonCollisionSetScale(self.as_raw(), s.x, s.y, s.z) }
    }

    pub fn set_matrix(&self, mat: &Matrix) {
        unsafe { ffi::NewtonCollisionSetMatrix(self.as_raw(), mat.as_ptr()) }
    }

    pub fn matrix(&self) -> Matrix {
        unsafe {
            let mut matrix: Matrix = mem::zeroed();
            ffi::NewtonCollisionGetMatrix(self.as_raw(), matrix.as_mut_ptr() as *const f32);
            matrix
        }
    }

    pub fn scale(&self) -> Vector {
        let mut scale = Vector::zero();
        unsafe {
            #[rustfmt::skip]
            let &mut Vector { mut x, mut y, mut z, .. } = &mut scale;
            ffi::NewtonCollisionGetScale(self.as_raw(), &mut x, &mut y, &mut z);
        }
        scale
    }
}

impl<'world> Drop for CollisionOld<'world> {
    fn drop(&mut self) {
        if self.owned {
            unsafe { ffi::NewtonDestroyCollision(self.as_raw()) }
        }
    }
}
