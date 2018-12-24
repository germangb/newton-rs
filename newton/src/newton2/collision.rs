use ffi;

use super::{Application, Types};
use super::world::{NewtonWorld, WorldRefMut};

use std::mem;
use std::rc::{Rc, Weak};
use std::cell::{RefCell, Ref, RefMut};
use std::os::raw;
use std::ops::{Deref, DerefMut};

pub type ShapeId = raw::c_int;

/// Ref-counted NewtonCollision Box
#[derive(Debug)]
pub struct BoxCollision<App>(pub(crate) Rc<RefCell<NewtonWorld<App>>>, pub(crate) Rc<RefCell<NewtonCollision<App>>>, *mut ffi::NewtonCollision);

#[doc(hidden)]
#[derive(Debug)]
pub struct NewtonCollision<App>(Rc<RefCell<NewtonWorld<App>>>, *mut ffi::NewtonCollision);

/// Immutable reference to a NewtonCollision
#[derive(Debug)]
pub struct CollisionRef<'w, 'c, App>(pub(crate) Ref<'w, NewtonWorld<App>>, pub(crate) Ref<'c, NewtonCollision<App>>, pub(crate) *const ffi::NewtonWorld, pub(crate) *mut ffi::NewtonCollision);

/// Mutable reference to a NewtonCollision
#[derive(Debug)]
pub struct CollisionRefMut<'w, 'c, App>(pub(crate) RefMut<'w, NewtonWorld<App>>, pub(crate) RefMut<'c, NewtonCollision<App>>, pub(crate) *const ffi::NewtonWorld, pub(crate) *mut ffi::NewtonCollision);

impl<F: Types, App: Application<Types=F>> BoxCollision<App> {
    pub fn new(world: WorldRefMut<App>, dx: f32, dy: f32, dz: f32, shape_id: ShapeId, offset: Option<&F::Matrix>) -> Self {
        let world_rc = unsafe {
            let world_ref: Weak<RefCell<NewtonWorld<App>>> = mem::transmute(ffi::NewtonWorldGetUserData(world.as_raw()));
            let world_ref_rc = Weak::upgrade(&world_ref).unwrap();
            mem::forget(world_ref);
            world_ref_rc
        };

        let collision_raw = unsafe {
            ffi::NewtonCreateBox(world.as_raw(), dx, dy, dz, shape_id, mem::transmute(offset))
        };

        let collision = NewtonCollision(world_rc.clone(), collision_raw);
        let collision_rc = Rc::new(RefCell::new(collision));

        // TODO FIXME WARNING hack
        let world_weak = Rc::downgrade(&world_rc);
        let collision_weak = Rc::downgrade(&collision_rc);

        let collision = BoxCollision(world_rc, collision_rc, collision_raw);

        // TODO FIXME WARNING hack
        unsafe {
            ffi::NewtonCollisionSetUserData(collision_raw, mem::transmute(Box::new( (world_weak, collision_weak) )));
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
        unsafe {
            eprintln!("drop collision");
            let _: Box<(Weak<RefCell<NewtonWorld<App>>>, Weak<RefCell<Self>>)> = mem::transmute(ffi::NewtonCollisionGetUserData(collision));
            ffi::NewtonDestroyCollision(collision)
        }
    }
}

