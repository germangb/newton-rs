use std::marker::PhantomData;
use std::mem;
use std::os::raw;

use super::body::Body;
use super::ffi;
use super::newton::Newton;
use super::{AsHandle, IntoHandle};
use super::{Handle, HandleInner};

use self::builders::{CompoundBuilder, SceneBuilder, TreeBuilder};

use self::iters::{Collisions, Handles};

/// Types to build compounds, scenes, and tree collisions.
pub mod builders;
/// Collision & collision handles iterators
pub mod iters;

macro_rules! collision {
    ($(
        $( #[ $($meta:meta)+ ] )*
        ($enum_var:ident , ffi::$id:ident , $option:ident ) => pub struct $collision:ident<'a>(...);
    )*) => {

    #[derive(Debug, Eq, PartialEq)]
    pub enum Collision<'a> {
        $($enum_var($collision<'a>)),*
    }

    #[repr(i32)]
    #[derive(Clone, Copy, Hash, Debug, Eq, PartialEq, PartialOrd, Ord)]
    pub enum Type {
        $($enum_var = ffi::$id as i32),*
    }

    fn check_owned(coll: &Collision) {
        match coll {
            $(Collision::$enum_var(ref col) => if !col.owned { panic!() }),*
        }
    }

    impl<'a> Collision<'a> {
        pub unsafe fn from_raw(raw: *const ffi::NewtonCollision, owned: bool) -> Self {
            let col_type = ffi::NewtonCollisionGetType(raw);
            match col_type as _ {
                $(
                    ffi::$id => Collision::$enum_var($collision::from_raw(raw, owned)),
                )*
                _ => unimplemented!("Collision type ({}) not implemented", col_type),
            }
        }

        pub fn create_instance(col: &Self) -> Self {
            match col {
                $( Collision::$enum_var(ref col) => Collision::$enum_var($collision::create_instance(&col)) ),*
            }
        }

        $(
            pub fn $option(self) -> Option<$collision<'a>> {
                match self {
                    Collision::$enum_var(col) => Some(col),
                    _ => None,
                }
            }
        )*
    }

    impl<'a> NewtonCollision for Collision<'a> {
        fn as_raw(&self) -> *const ffi::NewtonCollision {
            match self {
                $(Collision::$enum_var(col) => col.raw ),*
            }
        }
    }

    $(
        $(#[$($meta)+])*
        pub struct $collision<'a> {
            raw: *const ffi::NewtonCollision,

            // If set to true, the memory is freed when the instance is dropped.
            owned: bool,
            _phantom: PhantomData<&'a ()>,
        }

        unsafe impl<'a> Send for $collision<'a> {}
        unsafe impl<'a> Sync for $collision<'a> {}

        impl<'a> From<$collision<'a>> for Collision<'a> {
            fn from(col: $collision<'a>) -> Self {
                Collision::$enum_var ( col )
            }
        }

        impl<'a> $collision<'a> {
            pub unsafe fn from_raw(raw: *const ffi::NewtonCollision, owned: bool) -> Self {
                $collision { raw, owned, _phantom: PhantomData }
            }

            pub fn create_instance(col: &Self) -> Self {
                unsafe {
                    let instance = ffi::NewtonCollisionCreateInstance(col.raw);
                    Self::from_raw(instance, true)
                }
            }
        }

        impl<'a> Drop for $collision<'a> {
            fn drop(&mut self) {
                // if collision owns itself, then free the memory
                if self.owned {
                    unsafe {
                        ffi::NewtonDestroyCollision(self.raw);
                    }
                }
            }
        }

        impl<'a> IntoHandle for $collision<'a> {
            fn into_handle(mut self, newton: &Newton) -> Handle {
                if !self.owned { panic!() }
                self.owned = false;
                newton.move_collision2(self.into_collision())
            }
        }

        impl<'a> AsHandle for $collision<'a> {
            fn as_handle(&self) -> Handle {
                Handle::from_ptr(self.raw as _)
            }
        }

        impl<'a> IntoCollision<'a> for $collision<'a> {
            fn into_collision(self) -> Collision<'a> {
                Collision::$enum_var(self)
            }
        }

        impl<'a> NewtonCollision for $collision<'a> {
            fn as_raw(&self) -> *const ffi::NewtonCollision {
                self.raw
            }
        }
    )*}
}

collision! {
    #[derive(Debug, Eq, PartialEq)]
    (Cuboid, ffi::SERIALIZE_ID_BOX, cuboid) => pub struct Cuboid<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Sphere, ffi::SERIALIZE_ID_SPHERE, sphere) => pub struct Sphere<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Cylinder, ffi::SERIALIZE_ID_CYLINDER, cylinder) => pub struct Cylinder<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Capsule, ffi::SERIALIZE_ID_CAPSULE, capsule) => pub struct Capsule<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Cone, ffi::SERIALIZE_ID_CONE, cone) => pub struct Cone<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Compound, ffi::SERIALIZE_ID_COMPOUND, compound) => pub struct Compound<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Tree, ffi::SERIALIZE_ID_TREE, tree) => pub struct Tree<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Scene, ffi::SERIALIZE_ID_SCENE, scene) => pub struct Scene<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (ConvexHull, ffi::SERIALIZE_ID_CONVEXHULL, convex_hull) => pub struct ConvexHull<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (HeightField, ffi::SERIALIZE_ID_HEIGHTFIELD, height_field) => pub struct HeightField<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Null, ffi::SERIALIZE_ID_NULL, null) => pub struct Null<'a>(...);
}

impl<'a> Compound<'a> {
    pub fn create(newton: &'a Newton) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateCompoundCollision(newton.as_raw(), 0);
            Self::from_raw(collision, true)
        }
    }

    pub fn handles(&self) -> Handles {
        let next = unsafe { ffi::NewtonCompoundCollisionGetFirstNode(self.raw) };
        Handles {
            collision: self.raw,
            get_next: Box::new(|c, n| unsafe { ffi::NewtonCompoundCollisionGetNextNode(c, n) }),
            next,
            _phantom: PhantomData,
        }
    }

    pub fn collisions(&self) -> Collisions {
        Collisions {
            handles: self.handles(),
            get_col: Box::new(|c, n| unsafe {
                ffi::NewtonCompoundCollisionGetCollisionFromNode(c, n)
            }),
        }
    }

    pub fn get(&self, handle: Handle) -> Option<Collision> {
        match handle.inner() {
            HandleInner::Pointer(ptr) => unsafe {
                let raw = ffi::NewtonCompoundCollisionGetCollisionFromNode(self.raw, ptr as _);
                if raw.is_null() {
                    None
                } else {
                    Some(Collision::from_raw(raw, false))
                }
            },
            HandleInner::Index(idx) => unsafe {
                let ptr = ffi::NewtonCompoundCollisionGetNodeByIndex(self.raw, idx as _);
                if ptr.is_null() {
                    None
                } else {
                    self.get(Handle::from_ptr(ptr as _))
                }
            },
        }
    }

    pub fn begin(&mut self) -> CompoundBuilder {
        unsafe {
            ffi::NewtonCompoundCollisionBeginAddRemove(self.raw);
        }
        CompoundBuilder { compound: self }
    }
}

impl<'a> Scene<'a> {
    pub fn create(newton: &'a Newton) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateSceneCollision(newton.as_raw(), 0);
            Self::from_raw(collision, true)
        }
    }

    pub fn handles(&self) -> Handles {
        let next = unsafe { ffi::NewtonSceneCollisionGetFirstNode(self.raw) };
        Handles {
            collision: self.raw,
            get_next: Box::new(|c, n| unsafe { ffi::NewtonSceneCollisionGetNextNode(c, n) }),
            next,
            _phantom: PhantomData,
        }
    }

    pub fn collisions(&self) -> Collisions {
        Collisions {
            handles: self.handles(),
            get_col: Box::new(|c, n| unsafe {
                ffi::NewtonSceneCollisionGetCollisionFromNode(c, n)
            }),
        }
    }

    pub fn get(&self, handle: Handle) -> Option<Collision> {
        match handle.inner() {
            HandleInner::Pointer(ptr) => unsafe {
                let raw = ffi::NewtonSceneCollisionGetCollisionFromNode(self.raw, ptr as _);
                if raw.is_null() {
                    None
                } else {
                    Some(Collision::from_raw(raw, false))
                }
            },
            HandleInner::Index(idx) => unsafe {
                let ptr = ffi::NewtonSceneCollisionGetNodeByIndex(self.raw, idx as _);
                if ptr.is_null() {
                    None
                } else {
                    self.get(Handle::from_ptr(ptr as _))
                }
            },
        }
    }

    pub fn begin(&mut self) -> SceneBuilder {
        unsafe {
            ffi::NewtonSceneCollisionBeginAddRemove(self.raw);
        }
        SceneBuilder { scene: self }
    }
}

impl<'a> Tree<'a> {
    pub fn create(newton: &'a Newton) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateTreeCollision(newton.as_raw(), 0);
            Self::from_raw(collision, true)
        }
    }

    pub fn begin(&mut self) -> TreeBuilder {
        unsafe { ffi::NewtonTreeCollisionBeginBuild(self.raw) }
        TreeBuilder {
            tree: self,
            optimize: false,
        }
    }
}

impl<'a> Null<'a> {
    pub fn create(newton: &'a Newton) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateNull(newton.as_raw());
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Cuboid<'a> {
    pub fn create(
        newton: &'a Newton,
        x: f32,
        y: f32,
        z: f32,
        offset: Option<[[f32; 4]; 4]>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateBox(newton.as_raw(), x, y, z, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Cone<'a> {
    pub fn create(
        newton: &'a Newton,
        radius: f32,
        height: f32,
        offset: Option<[[f32; 4]; 4]>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateCone(newton.as_raw(), radius, height, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Sphere<'a> {
    pub fn create(newton: &'a Newton, radius: f32, offset: Option<[[f32; 4]; 4]>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateSphere(newton.as_raw(), radius, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Cylinder<'a> {
    pub fn create(
        newton: &'a Newton,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<[[f32; 4]; 4]>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision =
                ffi::NewtonCreateCylinder(newton.as_raw(), radius0, radius1, height, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Capsule<'a> {
    pub fn create(
        newton: &'a Newton,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<[[f32; 4]; 4]>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision =
                ffi::NewtonCreateCapsule(newton.as_raw(), radius0, radius1, height, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

trait IntoCollision<'a> {
    fn into_collision(self) -> Collision<'a>;
}

/// A marker trait for static collisions (tree and heightfield).
pub trait StaticShape: NewtonCollision {}

/// A marker trait for collision shapes with convex geometry.
pub trait ConvexShape: NewtonCollision {}

/// Marker traits for collisions with geometry that can be iterated.
///
/// Applies to all collision shapes, except for user defined meshes.
pub trait RenderableShape: NewtonCollision {}

macro_rules! statik {
    ( $( $collision:ident ),* ) => {$(impl<'a> StaticShape for $collision<'a> {})*}
}

macro_rules! convex {
    ( $( $collision:ident ),* ) => {$(impl<'a> ConvexShape for $collision<'a> {})*}
}

//macro_rules! polygon {
//    ( $( $collision:ident ),* ) => {$(impl<'a> RenderableShape for $collision<'a> {})*}
//}

statik! {
    HeightField, Tree, Scene
}

convex! {
    Cuboid, Sphere, Cylinder, Capsule, Cone, ConvexHull, Null
}

//polygon! {
//    Cuboid, Sphere, Cylinder, Capsule, Cone, ConvexHull, Scene, Compound, Tree, HeightField, Null
//}

// TODO convex?
/// Tests whether two transformed collisions intersect.
pub fn intersection_test<A, B>(
    col_a: &A,
    col_b: &B,
    mat_a: [[f32; 4]; 4],
    mat_b: [[f32; 4]; 4],
    thread_idx: usize,
) -> bool
where
    A: NewtonCollision,
    B: NewtonCollision,
{
    unimplemented!()
}

/// Returns the closest point between two convex collisions, or None if they intersect.
///
/// Returns the pair of points from each body that are closest to each other.
pub fn closest_point<A, B>(
    col_a: &A,
    col_b: &B,
    mat_a: [[f32; 4]; 4],
    mat_b: [[f32; 4]; 4],
    thread_idx: usize,
) -> Option<([f32; 3], [f32; 3])>
where
    A: ConvexShape,
    B: ConvexShape,
{
    unimplemented!()
}

/// NewtonCollision functions.
pub trait NewtonCollision {
    fn as_raw(&self) -> *const ffi::NewtonCollision;

    fn for_each_polygon<F: FnMut(&[f32], raw::c_int)>(
        &self,
        matrix: [[f32; 4]; 4],
        mut callback: F,
    ) {
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
        ) where
            F: FnMut(&[f32], raw::c_int),
        {
            let slice = std::slice::from_raw_parts(face_array, vert_count as usize * 3);
            mem::transmute::<_, &mut F>(udata)(slice, face_id);
        }
    }

    fn is_static(&self) -> bool {
        unsafe {
            let is = ffi::NewtonCollisionIsStaticShape(self.as_raw());
            is == 1
        }
    }

    fn is_convex(&self) -> bool {
        unsafe {
            let is = ffi::NewtonCollisionIsConvexShape(self.as_raw());
            is == 1
        }
    }

    fn into_raw(self) -> *const ffi::NewtonCollision
    where
        Self: Sized,
    {
        self.as_raw()
    }

    fn collision_type(&self) -> Type {
        unsafe { mem::transmute(ffi::NewtonCollisionGetType(self.as_raw())) }
    }

    fn matrix(&self) -> [[f32; 4]; 4] {
        let mut mat: [[f32; 4]; 4] = Default::default();
        unsafe { ffi::NewtonCollisionGetMatrix(self.as_raw(), mat[0].as_mut_ptr()) }
        mat
    }

    fn set_matrix(&self, matrix: [[f32; 4]; 4]) {
        unsafe { ffi::NewtonCollisionSetMatrix(self.as_raw(), matrix[0].as_ptr()) }
    }

    fn set_user_id(&self, id: u32) {
        unsafe { ffi::NewtonCollisionSetUserID(self.as_raw(), id) }
    }

    fn user_id(&self) -> u32 {
        unsafe { ffi::NewtonCollisionGetUserID(self.as_raw()) }
    }
}
