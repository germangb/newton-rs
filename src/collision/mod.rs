use std::marker::PhantomData;
use std::mem;
use std::os::raw;

use crate::body::Body;
use crate::ffi;
use crate::handle::{AsHandle, FromHandle, Handle, HandleInner, IntoHandle};
use crate::math::{Mat4, Vec3};
use crate::newton::Newton;

use builder::{CompoundBuilder, SceneBuilder, TreeBuilder};
use iter::{Collisions, Handles};

/// Types to build compounds, scenes, and tree collisions.
pub mod builder;
/// Collision & collision handles iterators
pub mod iter;

collision! {
    {
        enum Cuboid
        fn cuboid, is_cuboid
        #[derive(Debug, Eq, PartialEq)]
        struct Cuboid<'a>
        const ffi::SERIALIZE_ID_BOX
        params Cuboid {
            dx: f32,
            dy: f32,
            dz: f32
        }
    }

    {
        enum UserMesh
        fn user_mesh, is_user_mesh
        #[derive(Debug, Eq, PartialEq)]
        struct UserMesh<'a>
        const ffi::SERIALIZE_ID_USERMESH
        params UserMesh {}
    }

    {
        enum Sphere
        fn sphere, is_sphere
        #[derive(Debug, Eq, PartialEq)]
        struct Sphere<'a>
        const ffi::SERIALIZE_ID_SPHERE
        params Sphere {
            radius: f32,
        }
    }

    {
        enum Cylinder
        fn cylinder, is_cylinder
        #[derive(Debug, Eq, PartialEq)]
        struct Cylinder<'a>
        const ffi::SERIALIZE_ID_CYLINDER
        params Cylinder {
            radius0: f32,
            radius1: f32,
            height: f32,
        }
    }

    {
        enum ChamferCylinder
        fn chamfer_cylinder, is_chamfer_cylinder
        #[derive(Debug, Eq, PartialEq)]
        struct ChamferCylinder<'a>
        const ffi::SERIALIZE_ID_CHAMFERCYLINDER
        params ChamferCylinder {
            radius: f32,
            height: f32,
        }
    }

    {
        enum Capsule
        fn capsule, is_capsule
        #[derive(Debug, Eq, PartialEq)]
        struct Capsule<'a>
        const ffi::SERIALIZE_ID_CAPSULE
        params Capsule {
            radius0: f32,
            radius1: f32,
            height: f32,
        }
    }

    {
        enum Cone
        fn cone, is_cone
        #[derive(Debug, Eq, PartialEq)]
        struct Cone<'a>
        const ffi::SERIALIZE_ID_CONE
        params Cone {
            radius: f32,
            height: f32,
        }
    }

    {
        enum Compound
        fn compound, is_compound
        #[derive(Debug, Eq, PartialEq)]
        struct Compound<'a>
        const ffi::SERIALIZE_ID_COMPOUND
        params Compound {
            children_count: usize,
        }
    }

    {
        enum FracturedCompound
        fn fractured_compound, is_fractured_compound
        #[derive(Debug, Eq, PartialEq)]
        struct FracturedCompound<'a>
        const ffi::SERIALIZE_ID_FRACTURED_COMPOUND
        params FracturedCompound { }
    }

    {
        enum Tree
        fn tree, is_tree
        #[derive(Debug, Eq, PartialEq)]
        struct Tree<'a>
        const ffi::SERIALIZE_ID_TREE
        params Tree {
            vertex_count: usize,
            index_count: usize,
        }
    }

    {
        enum Scene
        fn scene, is_scene
        #[derive(Debug, Eq, PartialEq)]
        struct Scene<'a>
        const ffi::SERIALIZE_ID_SCENE
        params Scene {
            children_proxy_count: usize,
        }
    }

    {
        enum ConvexHull
        fn convex_hull, is_convex_hull
        #[derive(Debug, Eq, PartialEq)]
        struct ConvexHull<'a>
        const ffi::SERIALIZE_ID_CONVEXHULL
        params ConvexHull { }
    }

/*
    {
        enum HeightField
        fn height_field
        #[derive(Debug, Eq, PartialEq)]
        struct HeightField<'a>
        const ffi::SERIALIZE_ID_HEIGHTFIELD
        params HeightFieldF32 (HeightFieldParams<'a, f32>)
    }
*/

    {
        enum Null
        fn null, is_null
        #[derive(Debug, Eq, PartialEq)]
        struct Null<'a>
        const ffi::SERIALIZE_ID_NULL
        params Null { }
    }

    {
        enum DeformableSolid
        fn deformable_solid, is_deformable_solid
        #[derive(Debug, Eq, PartialEq)]
        struct DeformableSolid<'a>
        const ffi::SERIALIZE_ID_DEFORMABLE_SOLID
        params DeformableSolid { }
    }

    {
        enum MassSpringDamperSystem
        fn mass_spring_damper_system, is_mass_spring_damper_system
        #[derive(Debug, Eq, PartialEq)]
        struct MassSpringDamperSystem<'a>
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

    pub fn begin_build(&mut self) -> CompoundBuilder {
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

    pub fn begin_build(&mut self) -> SceneBuilder {
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

    pub fn begin_build(&mut self) -> TreeBuilder {
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

// m_normalDiagonals = 0,
// m_invertedDiagonals,
// m_alternateOddRowsDiagonals,
// m_alternateEvenRowsDiagonals,
// m_alternateOddColumsDiagonals,
// m_alternateEvenColumsDiagonals,
// m_starDiagonals,
// m_starInvertexDiagonals,
/// HeightField grid construction pattern.
#[repr(i32)]
#[derive(Clone, Copy, Eq, PartialEq, Hash)]
pub enum HeightFieldGrid {
    NormalDiagonals = 0,
    InvertedDiagonals = 1,
    AlternateOddRowsDiagonals = 2,
    AlternateEvenRowsDiagonals = 3,
    AlternateOddColumsDiagonals = 4,
    AlternateEvenColumsDiagonals = 5,
    StarDiagonals = 6,
    StarInvertexDiagonals = 7,
}

impl<'a, T: Elevation> HeightField<'a, T> {
    /// A more convenient way to build a HeightField collision with default values.
    pub fn builder() -> ! {
        unimplemented!()
    }

    pub fn create(newton: &'a Newton,
                  width: usize,
                  height: usize,
                  grid: HeightFieldGrid,
                  elevation: &[T],
                  attrs: &[i8],
                  vert_scale: f32,
                  hor_x_scale: f32,
                  hor_z_scale: f32)
                  -> Self {
        let data_type = T::newton_enum();
        unsafe {
            let raw = ffi::NewtonCreateHeightFieldCollision(newton.as_raw(),
                                                            width as _,
                                                            height as _,
                                                            mem::transmute(grid),
                                                            data_type,
                                                            elevation.as_ptr() as _,
                                                            attrs.as_ptr(),
                                                            vert_scale,
                                                            hor_x_scale,
                                                            hor_z_scale,
                                                            0);
            Self::from_raw(raw, true)
        }
    }

    pub fn set_horizontal_displacement(&self, map: &[u16], scale: f32) {
        unsafe { ffi::NewtonHeightFieldSetHorizontalDisplacement(self.raw, map.as_ptr(), scale) }
    }
}

trait IntoCollision<'a> {
    fn into_collision(self) -> Collision<'a>;
}

/// A marker trait for static collisions (Tree, Scene & HeightField).
///
/// If a body has a static collision assigned, the body becomes static.
pub trait StaticShape: NewtonCollision {}

/// A marker trait for collision shapes with convex geometry.
pub trait ConvexShape: NewtonCollision {}

/// HeightField collision elevation data types
// https://github.com/MADEAPPS/newton-dynamics/blob/69d773bd41f6aa8b6a955b7fa21515e22040e05a/sdk/dgPhysics/dgCollisionHeightField.h#L36
// https://github.com/MADEAPPS/newton-dynamics/blob/69d773bd41f6aa8b6a955b7fa21515e22040e05a/sdk/dgPhysics/dgCollisionHeightField.h#L35
pub trait Elevation {
    fn newton_enum() -> i32;
}

impl Elevation for f32 {
    fn newton_enum() -> i32 {
        0
    }
}
impl Elevation for u16 {
    fn newton_enum() -> i32 {
        1
    }
}

macro_rules! statik {
    ( $( $collision:ident ),* ) => {$(impl<'a> StaticShape for $collision<'a> {})*}
}

macro_rules! convex {
    ( $( $collision:ident ),* ) => {$(impl<'a> ConvexShape for $collision<'a> {})*}
}

statik! {
    Tree, Scene
}

impl<'a, T: Elevation> StaticShape for HeightField<'a, T> {}

convex! {
    Cuboid, Sphere, Cylinder, Capsule, Cone, ConvexHull, Null, ChamferCylinder
}

/// Calculates acceleration that satisfies a given damper system.
///
/// http://newtondynamics.com/wiki/index.php5?title=NewtonCalculateSpringDamperAcceleration
pub fn calculate_spring_damper_acceleration(dt: f32, ks: f32, x: f32, kd: f32, s: f32) -> f32 {
    unsafe { ffi::NewtonCalculateSpringDamperAcceleration(dt, ks, x, kd, s) }
}

// TODO convex?
/// Tests whether two transformed collisions intersect.
pub fn intersection_test<A, B>(newton: &Newton,
                               col_a: &A,
                               mat_a: Mat4,
                               col_b: &B,
                               mat_b: Mat4,
                               thread_idx: usize)
                               -> bool
    where A: NewtonCollision,
          B: NewtonCollision
{
    unsafe {
        ffi::NewtonCollisionIntersectionTest(newton.as_raw(),
                                             col_a.as_raw(),
                                             mat_a[0].as_ptr(),
                                             col_b.as_raw(),
                                             mat_b[0].as_ptr(),
                                             thread_idx as _)
        == 1
    }
}

/// Closest point between two collisions.
/// Type returned by the `closest_point` function.
#[derive(Debug, Clone, Copy)]
pub struct ClosestPoint {
    pub p0: Vec3,
    pub mat0: Mat4,
    pub p1: Vec3,
    pub mat1: Mat4,
    pub normal: Vec3,
}

/// Returns the closest point between two convex collisions.
pub fn closest_point<A, B>(newton: &Newton,
                           col_a: &A,
                           mat_a: Mat4,
                           col_b: &B,
                           mat_b: Mat4,
                           thread_idx: usize)
                           -> Option<ClosestPoint>
    where A: ConvexShape,
          B: ConvexShape
{
    unsafe {
        let mut p0 = [0.0, 0.0, 0.0];
        let mut p1 = [0.0, 0.0, 0.0];
        let mut normal = [0.0, 0.0, 0.0];
        if ffi::NewtonCollisionClosestPoint(newton.as_raw(),
                                            col_a.as_raw(),
                                            mat_a[0].as_ptr(),
                                            col_b.as_raw(),
                                            mat_b[0].as_ptr(),
                                            p0.as_mut_ptr(),
                                            p1.as_mut_ptr(),
                                            normal.as_mut_ptr(),
                                            thread_idx as _)
           == 1
        {
            Some(ClosestPoint { p0, p1, normal, mat0: mat_a, mat1: mat_b })
        } else {
            None
        }
    }
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

    fn params(&self) -> Params {
        unimplemented!()
    }

    fn for_each_polygon<F>(&self, matrix: Mat4, mut callback: F)
        where F: FnMut(&[f32], raw::c_int)
    {
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
