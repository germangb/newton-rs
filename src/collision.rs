use std::marker::PhantomData;
use std::mem;
use std::os::raw;

use super::body::Body;
use super::ffi;
use super::world::Newton;
use super::Handle;
use super::IntoHandle;

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
    pub enum CollisionType {
        $($enum_var = ffi::$id as i32),*
    }

    fn make_not_owned(coll: &mut Collision) {
        match coll {
            $(Collision::$enum_var(ref mut col) => col.owned = false),*
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

        $(
        fn $option(self) -> Option<$collision<'a>> {
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

        impl<'a> $collision<'a> {
            /// The returned collision owns itself, i.e. It will be freed when the instance
            /// is dropped.
            pub unsafe fn from_raw(raw: *const ffi::NewtonCollision, owned: bool) -> Self {
                $collision { raw, owned, _phantom: PhantomData }
            }
        }

        impl<'a> Drop for $collision<'a> {
            fn drop(&mut self) {
                // if collision owns itself, then free the memory
                unsafe {
                    if self.owned {
                        let udata = ffi::NewtonCollisionGetUserData(self.raw);
                        let _: Option<Box<UserData>> = mem::transmute(udata);
                        ffi::NewtonDestroyCollision(self.raw);
                    }
                }
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
    #[derive(Debug, Eq, PartialEq)]
    (Box, ffi::SERIALIZE_ID_BOX, box2) => pub struct BoxCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Sphere, ffi::SERIALIZE_ID_SPHERE, sphere) => pub struct SphereCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Cylinder, ffi::SERIALIZE_ID_CYLINDER, cylinder) => pub struct CylinderCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Capsule, ffi::SERIALIZE_ID_CAPSULE, capsule) => pub struct CapsuleCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Cone, ffi::SERIALIZE_ID_CONE, cone) => pub struct ConeCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Compound, ffi::SERIALIZE_ID_COMPOUND, compound) => pub struct CompoundCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Tree, ffi::SERIALIZE_ID_TREE, tree) => pub struct TreeCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Scene, ffi::SERIALIZE_ID_SCENE, scene) => pub struct SceneCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (ConvexHull, ffi::SERIALIZE_ID_CONVEXHULL, convex_hull) => pub struct ConvexHullCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (HeightField, ffi::SERIALIZE_ID_HEIGHTFIELD, height_field) => pub struct HeightFieldCollision<'a>(...);

    #[derive(Debug, Eq, PartialEq)]
    (Null, ffi::SERIALIZE_ID_NULL, null) => pub struct NullCollision<'a>(...);
}

impl<'world> CompoundCollision<'world> {
    pub fn create(newton: &'world Newton, nodes: &[Collision], name: Option<&'static str>) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateCompoundCollision(newton.as_raw(), 0);

            ffi::NewtonCompoundCollisionBeginAddRemove(collision);
            for node in nodes {
                ffi::NewtonCompoundCollisionAddSubCollision(collision, node.as_raw());
            }
            ffi::NewtonCompoundCollisionEndAddRemove(collision);

            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

impl<'world> SceneCollision<'world> {
    pub fn create(newton: &'world Newton, nodes: &[Collision], name: Option<&'static str>) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateSceneCollision(newton.as_raw(), 0);

            ffi::NewtonSceneCollisionBeginAddRemove(collision);
            for node in nodes {
                ffi::NewtonSceneCollisionAddSubCollision(collision, node.as_raw());
            }
            ffi::NewtonSceneCollisionEndAddRemove(collision);

            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

/*
impl<'world> TreeCollision<'world> {
    pub fn create(newton: &'world Newton, name: Option<&'static str>) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateTreeCollision(newton.as_raw(), 0);
            let udata = Box::new(UserData { owner: collision as _, name });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}
*/

impl<'world> NullCollision<'world> {
    pub fn create(newton: &'world Newton, name: Option<&'static str>) -> Self {
        unsafe {
            let collision = ffi::NewtonCreateNull(newton.as_raw());
            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

impl<'world> BoxCollision<'world> {
    pub fn create(
        newton: &'world Newton,
        x: f32,
        y: f32,
        z: f32,
        offset: Option<[[f32; 4]; 4]>,
        name: Option<&'static str>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateBox(newton.as_raw(), x, y, z, 0, offset);
            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

impl<'world> ConeCollision<'world> {
    pub fn create(
        newton: &'world Newton,
        radius: f32,
        height: f32,
        offset: Option<[[f32; 4]; 4]>,
        name: Option<&'static str>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateCone(newton.as_raw(), radius, height, 0, offset);
            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

impl<'world> SphereCollision<'world> {
    pub fn create(
        newton: &'world Newton,
        radius: f32,
        offset: Option<[[f32; 4]; 4]>,
        name: Option<&'static str>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision = ffi::NewtonCreateSphere(newton.as_raw(), radius, 0, offset);
            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

impl<'world> CylinderCollision<'world> {
    pub fn create(
        newton: &'world Newton,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<[[f32; 4]; 4]>,
        name: Option<&'static str>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision =
                ffi::NewtonCreateCylinder(newton.as_raw(), radius0, radius1, height, 0, offset);
            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

impl<'world> CapsuleCollision<'world> {
    pub fn create(
        newton: &'world Newton,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<[[f32; 4]; 4]>,
        name: Option<&'static str>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset.as_ref());
            let collision =
                ffi::NewtonCreateCapsule(newton.as_raw(), radius0, radius1, height, 0, offset);
            let udata = Box::new(UserData {
                owner: collision as _,
                name,
            });
            ffi::NewtonCollisionSetUserData(collision, mem::transmute(Some(udata)));
            Self::from_raw(collision, true)
        }
    }
}

pub trait IntoCollision<'world> {
    fn into_collision(self) -> Collision<'world>;
}

impl<'a, T> IntoHandle<Collision<'a>> for T
where
    T: NewtonCollision + IntoCollision<'a>,
{
    fn into_handle(self, newton: &Newton) -> Handle {
        let mut collision = self.into_collision();
        make_not_owned(&mut collision);
        newton.move_collision2(collision)
    }

    fn as_handle(&self) -> Handle {
        Handle(self.as_raw() as _)
    }
}

pub trait NewtonCollision {
    fn as_raw(&self) -> *const ffi::NewtonCollision;

    fn into_raw(self) -> *const ffi::NewtonCollision
    where
        Self: Sized,
    {
        self.as_raw()
    }

    fn collision_type(&self) -> CollisionType {
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
        ) where
            F: FnMut(&[f32], raw::c_int),
        {
            let slice = std::slice::from_raw_parts(face_array, vert_count as usize * 3);
            mem::transmute::<_, &mut F>(udata)(slice, face_id);
        }
    }
}
