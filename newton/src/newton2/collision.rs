use ffi;

use super::world::{NewtonWorld, WorldRefMut};
use super::{Application, Types};

use std::cell::{Ref, RefCell, RefMut};
use std::mem;
use std::ops::{Deref, DerefMut};
use std::os::raw;
use std::rc::{Rc, Weak};

pub type ShapeId = raw::c_int;

macro_rules! collisions {
    ($(
        $(#[$($meta:meta)+])*
        pub struct $name:ident<App>( .. );
        {
            params => ( $($param:ident),* )
            function => ffi::$ffi:ident
        }
    )+) => {$(
        $(#[$($meta)+])*
        pub struct $name<App>(pub(crate) Rc<RefCell<NewtonWorld<App>>>, pub(crate) Rc<RefCell<NewtonCollision<App>>>, *mut ffi::NewtonCollision);

        impl<F: Types, App: Application<Types = F>> $name<App> {
            pub fn new(
                world: WorldRefMut<App>,
                $($param : f32 ,)+
                shape_id: ShapeId,
                offset: Option<&F::Matrix>,
            ) -> Self {
                let world_rc = unsafe {
                    let world_ref: Weak<RefCell<NewtonWorld<App>>> =
                        mem::transmute(ffi::NewtonWorldGetUserData(world.as_raw()));
                    let world_ref_rc = Weak::upgrade(&world_ref).unwrap();
                    mem::forget(world_ref);
                    world_ref_rc
                };

                let collision_raw = unsafe {
                    ffi::$ffi(world.as_raw(), $( $param ,)+ shape_id, mem::transmute(offset))
                };

                let collision = NewtonCollision(world_rc.clone(), collision_raw);
                let collision_rc = Rc::new(RefCell::new(collision));

                // TODO FIXME WARNING hack
                let world_weak = Rc::downgrade(&world_rc);
                let collision_weak = Rc::downgrade(&collision_rc);

                let collision = $name(world_rc, collision_rc, collision_raw);

                // TODO FIXME WARNING hack
                unsafe {
                    ffi::NewtonCollisionSetUserData(
                        collision_raw,
                        mem::transmute(Box::new((world_weak, collision_weak))),
                    );
                }

                collision
            }

            pub fn borrow(&self) -> CollisionRef<App> {
                let world_ref = self.0.borrow();
                let collision_ref = self.1.borrow();
                let world_ptr = world_ref.as_raw();
                CollisionRef(world_ref, collision_ref, world_ptr, self.2)
            }

            pub fn borrow_mut(&self) -> CollisionRefMut<App> {
                let world_ref = self.0.borrow_mut();
                let collision_ref = self.1.borrow_mut();
                let world_ptr = world_ref.as_raw();
                CollisionRefMut(world_ref, collision_ref, world_ptr, self.2)
            }
        }

    )+};
}

collisions! {
    #[derive(Debug, Clone)]
    pub struct BoxCollision<App>( .. );
    {
        params => (dx, dy, dz)
        function => ffi::NewtonCreateBox
    }

    #[derive(Debug, Clone)]
    pub struct SphereCollision<App>( .. );
    {
        params => (radius)
        function => ffi::NewtonCreateSphere
    }

    #[derive(Debug, Clone)]
    pub struct CylinderCollision<App>( .. );
    {
        params => (radius0, radius1, height)
        function => ffi::NewtonCreateCylinder
    }

    #[derive(Debug, Clone)]
    pub struct CapsuleCollision<App>( .. );
    {
        params => (radius0, radius1, height)
        function => ffi::NewtonCreateCapsule
    }

    #[derive(Debug, Clone)]
    pub struct ConeCollision<App>( .. );
    {
        params => (radius, height)
        function => ffi::NewtonCreateCone
    }

    //#[derive(Debug, Clone)]
    //pub struct NullCollision<App>( .. );
    //{
    //    params => ()
    //    function => ffi::NewtonCreateNull
    //}
}

#[derive(Debug)]
pub struct NewtonCollision<App>(Rc<RefCell<NewtonWorld<App>>>, *mut ffi::NewtonCollision);

#[derive(Debug)]
pub struct CollisionRef<'w, 'c, App>(
    pub(crate) Ref<'w, NewtonWorld<App>>,
    pub(crate) Ref<'c, NewtonCollision<App>>,
    pub(crate) *const ffi::NewtonWorld,
    pub(crate) *mut ffi::NewtonCollision,
);

#[derive(Debug)]
pub struct CollisionRefMut<'w, 'c, App>(
    pub(crate) RefMut<'w, NewtonWorld<App>>,
    pub(crate) RefMut<'c, NewtonCollision<App>>,
    pub(crate) *const ffi::NewtonWorld,
    pub(crate) *mut ffi::NewtonCollision,
);

impl<App> NewtonCollision<App> {
    pub fn as_raw(&self) -> *const ffi::NewtonCollision {
        self.1 as *const _
    }

    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonCollision {
        self.1
    }
}

impl<'w, 'c, App> Deref for CollisionRef<'w, 'c, App> {
    type Target = NewtonCollision<App>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'w, 'c, App> Deref for CollisionRefMut<'w, 'c, App> {
    type Target = NewtonCollision<App>;

    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'w, 'c, App> DerefMut for CollisionRefMut<'w, 'c, App> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<App> Drop for NewtonCollision<App> {
    fn drop(&mut self) {
        let collision = self.1;
        let _ = self.0.borrow_mut();
        unsafe {
            let _: Box<(Weak<RefCell<NewtonWorld<App>>>, Weak<RefCell<Self>>)> =
                mem::transmute(ffi::NewtonCollisionGetUserData(collision));
            ffi::NewtonDestroyCollision(collision)
        }
    }
}
