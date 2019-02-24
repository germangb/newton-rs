use std::marker::PhantomData;
use std::mem;
use std::os::raw;

use super::body::Body;
use super::ffi;
use super::newton::Newton;
use super::IntoHandle;
use super::{AsHandle, FromHandle, Handle, HandleInner, Mat4, Vec3};

use self::builder::{CompoundBuilder, SceneBuilder, TreeBuilder};

use self::iter::{Collisions, Handles};

/// Types to build compounds, scenes, and tree collisions.
pub mod builder;
/// Collision & collision handles iterators
pub mod iter;

collision! {
    {
        enum Cuboid
        fn cuboid
        #[derive(Debug, Eq, PartialEq)]
        struct Cuboid
        const ffi::SERIALIZE_ID_BOX
        params Cuboid {
            dx: f32,
            dy: f32,
            dz: f32
        }
    }

    {
        enum Sphere
        fn sphere
        #[derive(Debug, Eq, PartialEq)]
        struct Sphere
        const ffi::SERIALIZE_ID_SPHERE
        params Sphere {
            radius: f32,
        }
    }

    {
        enum Cylinder
        fn cylinder
        #[derive(Debug, Eq, PartialEq)]
        struct Cylinder
        const ffi::SERIALIZE_ID_CYLINDER
        params Cylinder {
            radius0: f32,
            radius1: f32,
            height: f32,
        }
    }

    {
        enum ChamferCylinder
        fn chamfer_cylinder
        #[derive(Debug, Eq, PartialEq)]
        struct ChamferCylinder
        const ffi::SERIALIZE_ID_CHAMFERCYLINDER
        params ChamferCylinder {
            radius: f32,
            height: f32,
        }
    }

    {
        enum Capsule
        fn capsule
        #[derive(Debug, Eq, PartialEq)]
        struct Capsule
        const ffi::SERIALIZE_ID_CAPSULE
        params Capsule {
            radius0: f32,
            radius1: f32,
            height: f32,
        }
    }

    {
        enum Cone
        fn cone
        #[derive(Debug, Eq, PartialEq)]
        struct Cone
        const ffi::SERIALIZE_ID_CONE
        params Cone {
            radius: f32,
            height: f32,
        }
    }

    {
        enum Compound
        fn compound
        #[derive(Debug, Eq, PartialEq)]
        struct Compound
        const ffi::SERIALIZE_ID_COMPOUND
        params Compound {
            children_count: usize,
        }
    }

    {
        enum FracturedCompound
        fn fractured_compound
        #[derive(Debug, Eq, PartialEq)]
        struct FracturedCompound
        const ffi::SERIALIZE_ID_FRACTURED_COMPOUND
        params FracturedCompound { }
    }

    {
        enum Tree
        fn tree
        #[derive(Debug, Eq, PartialEq)]
        struct Tree
        const ffi::SERIALIZE_ID_TREE
        params Tree {
            vertex_count: usize,
            index_count: usize,
        }
    }

    {
        enum Scene
        fn scene
        #[derive(Debug, Eq, PartialEq)]
        struct Scene
        const ffi::SERIALIZE_ID_SCENE
        params Scene {
            children_proxy_count: usize,
        }
    }

    {
        enum ConvexHull
        fn convex_hull
        #[derive(Debug, Eq, PartialEq)]
        struct ConvexHull
        const ffi::SERIALIZE_ID_CONVEXHULL
        params ConvexHull { }
    }

    {
        enum HeightField
        fn height_field
        #[derive(Debug, Eq, PartialEq)]
        struct HeightField
        const ffi::SERIALIZE_ID_HEIGHTFIELD
        params HeightField {
            width: usize,
            height: usize,
            vertical_elevation: &'a [T],
            vertical_scale: f32,
            horizontal_scale_x: f32,
            horizontal_scale_z: f32,
            horizontal_displacement_scale_x: f32,
            horizontal_displacement_scale_z: f32,
            horizontal_displacement: &'a [i16],
            attributes: &'a [i8],

        }
    }

    {
        enum Null
        fn null
        #[derive(Debug, Eq, PartialEq)]
        struct Null
        const ffi::SERIALIZE_ID_NULL
        params Null { }
    }

    {
        enum DeformableSolid
        fn deformable_solid
        #[derive(Debug, Eq, PartialEq)]
        struct DeformableSolid
        const ffi::SERIALIZE_ID_DEFORMABLE_SOLID
        params DeformableSolid { }
    }

    {
        enum MassSpringDamperSystem
        fn mass_spring_damper_system
        #[derive(Debug, Eq, PartialEq)]
        struct MassSpringDamperSystem
        // TODO is the constant the correct one?
        const ffi::SERIALIZE_ID_CLOTH_PATCH
        params MassSpringDamperSystem { }
    }
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
        Handles { collision: self.raw,
                  get_next: Box::new(|c, n| unsafe {
                      ffi::NewtonCompoundCollisionGetNextNode(c, n)
                  }),
                  next,
                  _phantom: PhantomData }
    }

    pub fn collisions(&self) -> Collisions {
        Collisions { handles: self.handles(),
                     get_col: Box::new(|c, n| unsafe {
                         ffi::NewtonCompoundCollisionGetCollisionFromNode(c, n)
                     }) }
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
        Handles { collision: self.raw,
                  get_next: Box::new(|c, n| unsafe {
                      ffi::NewtonSceneCollisionGetNextNode(c, n)
                  }),
                  next,
                  _phantom: PhantomData }
    }

    pub fn collisions(&self) -> Collisions {
        Collisions { handles: self.handles(),
                     get_col: Box::new(|c, n| unsafe {
                         ffi::NewtonSceneCollisionGetCollisionFromNode(c, n)
                     }) }
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
        TreeBuilder { tree: self, optimize: false }
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
    pub fn create(newton: &'a Newton, x: f32, y: f32, z: f32, offset: Option<Mat4>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateBox(newton.as_raw(), x, y, z, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Cone<'a> {
    pub fn create(newton: &'a Newton, radius: f32, height: f32, offset: Option<Mat4>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateCone(newton.as_raw(), radius, height, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Sphere<'a> {
    pub fn create(newton: &'a Newton, radius: f32, offset: Option<Mat4>) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateSphere(newton.as_raw(), radius, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Cylinder<'a> {
    pub fn create(newton: &'a Newton,
                  radius0: f32,
                  radius1: f32,
                  height: f32,
                  offset: Option<Mat4>)
                  -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision =
                ffi::NewtonCreateCylinder(newton.as_raw(), radius0, radius1, height, 0, offset);
            Self::from_raw(collision, true)
        }
    }
}

impl<'a> Capsule<'a> {
    pub fn create(newton: &'a Newton,
                  radius0: f32,
                  radius1: f32,
                  height: f32,
                  offset: Option<Mat4>)
                  -> Self {
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

/*
/// Marker traits for collisions with geometry that can be iterated.
///
/// Applies to all collision shapes, except for user defined meshes.
pub trait RenderableShape: NewtonCollision {}
*/

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
    Cuboid, Sphere, Cylinder, Capsule, Cone, ConvexHull, Null, ChamferCylinder
}

//polygon! {
//    Cuboid, Sphere, Cylinder, Capsule, Cone, ConvexHull, Scene, Compound, Tree, HeightField, Null
//}

/// Calculates acceleration that satisfies a given damper system.
///
/// http://newtondynamics.com/wiki/index.php5?title=NewtonCalculateSpringDamperAcceleration
pub fn calculate_spring_damper_acceleration(dt: f32, ks: f32, x: f32, kd: f32, s: f32) -> f32 {
    unsafe { ffi::NewtonCalculateSpringDamperAcceleration(dt, ks, x, kd, s) }
}

// TODO convex?
/// Tests whether two transformed collisions intersect.
pub fn intersection_test<A, B>(col_a: &A,
                               col_b: &B,
                               mat_a: Mat4,
                               mat_b: Mat4,
                               thread_idx: usize)
                               -> bool
    where A: NewtonCollision,
          B: NewtonCollision
{
    unimplemented!()
}

/// Returns the closest point between two convex collisions, or None if they intersect.
///
/// Returns the pair of points from each body that are closest to each other.
pub fn closest_point<A, B>(col_a: &A,
                           col_b: &B,
                           mat_a: Mat4,
                           mat_b: Mat4,
                           thread_idx: usize)
                           -> Option<(Vec3, Vec3)>
    where A: ConvexShape,
          B: ConvexShape
{
    unimplemented!()
}

/// NewtonCollision functions.
pub trait NewtonCollision {
    fn as_raw(&self) -> *const ffi::NewtonCollision;

    fn set_scale(&self, scale: Vec3) {
        unsafe {
            let [x, y, z] = scale;
            ffi::NewtonCollisionSetScale(self.as_raw(), x, y, z);
        }
    }

    fn scale(&self) -> Vec3 {
        unsafe {
            let mut scale = [0.0, 0.0, 0.0];
            ffi::NewtonCollisionGetScale(self.as_raw(),
                                         &mut scale[0],
                                         &mut scale[1],
                                         &mut scale[2]);
            scale
        }
    }

    fn for_each_polygon<F: FnMut(&[f32], raw::c_int)>(&self, matrix: Mat4, mut callback: F) {
        unsafe {
            let udata = mem::transmute(&mut callback);
            let matrix = matrix[0].as_ptr();
            ffi::NewtonCollisionForEachPolygonDo(self.as_raw(), matrix, Some(iterator::<F>), udata);
        }

        unsafe extern "C" fn iterator<F>(udata: *const raw::c_void,
                                         vert_count: raw::c_int,
                                         face_array: *const f32,
                                         face_id: raw::c_int)
            where F: FnMut(&[f32], raw::c_int)
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
        where Self: Sized
    {
        self.as_raw()
    }

    fn collision_type(&self) -> Type {
        unsafe { mem::transmute(ffi::NewtonCollisionGetType(self.as_raw())) }
    }

    fn matrix(&self) -> Mat4 {
        let mut mat: Mat4 = Default::default();
        unsafe { ffi::NewtonCollisionGetMatrix(self.as_raw(), mat[0].as_mut_ptr()) }
        mat
    }

    fn set_matrix(&self, matrix: Mat4) {
        unsafe { ffi::NewtonCollisionSetMatrix(self.as_raw(), matrix[0].as_ptr()) }
    }

    fn set_user_id(&self, id: u32) {
        unsafe { ffi::NewtonCollisionSetUserID(self.as_raw(), id) }
    }

    fn user_id(&self) -> u32 {
        unsafe { ffi::NewtonCollisionGetUserID(self.as_raw()) }
    }
}
