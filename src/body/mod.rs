use ffi;

use super::collision::NewtonCollision;
use super::joint::{Contacts, Joints};
use super::lock::{Lock, LockError, Locked, LockedMut};
use super::material::GroupId;
use super::world::{Command, NewtonWorld, WorldLockedMut};
use super::{Matrix, Quaternion, Result, Shared, Tx, Vector, Weak};

use std::{
    marker::PhantomData,
    mem,
    ops::{Deref, DerefMut},
    os::raw,
    ptr,
    sync::mpsc,
    time::Duration,
};

/// Ref-counted body
#[derive(Debug, Clone)]
pub struct Body<B, C>(
    Shared<Lock<NewtonWorld<B, C>>>,
    Shared<Lock<NewtonBody<B, C>>>,
    *const ffi::NewtonBody,
);

unsafe impl<B, C> Send for Body<B, C> {}
unsafe impl<B, C> Sync for Body<B, C> {}

/// Weak version of Body
#[derive(Debug, Clone)]
pub struct WeakBody<B, C>(
    Weak<Lock<NewtonWorld<B, C>>>,
    Weak<Lock<NewtonBody<B, C>>>,
    *const ffi::NewtonBody,
);

unsafe impl<B, C> Send for WeakBody<B, C> {}
unsafe impl<B, C> Sync for WeakBody<B, C> {}

impl<B, C> WeakBody<B, C> {
    pub fn upgrade(weak: &WeakBody<B, C>) -> Option<Body<B, C>> {
        let world = Weak::upgrade(&weak.0);
        if world.is_some() {
            let body = Weak::upgrade(&weak.1);
            if body.is_some() {
                return Some(Body(world.unwrap(), body.unwrap(), weak.2));
            }
        }
        None
    }
}

#[derive(Debug)]
pub struct NewtonBody<B, C> {
    // TODO remove pub(crate). It is used by the convex cast
    pub(crate) body: *mut ffi::NewtonBody,
    /// Bodies must be dropped before the world is.
    world: Shared<Lock<NewtonWorld<B, C>>>,
    // TODO remove pub(crate). It is used by the convex cast
    /// A non-owned NewtonCollision
    pub(crate) collision: NewtonCollision<B, C>,
    /// Owned NewtonBody s, when dropped, are destroyed and therefore removed from the simulation
    owned: bool,
    /// Destroyed flag
    destroyed: bool,
    /// The Tx end of the command channel. Only for owned variants,
    tx: Option<Tx<Command>>,
}

//#[doc(hidden)]
pub struct NewtonBodyData<B, C> {
    /// A reference to the World context so it can be referenced in callbacks and so on
    world: Weak<Lock<NewtonWorld<B, C>>>,
    /// A reference to itself
    body: Weak<Lock<NewtonBody<B, C>>>,
    /// Debug name
    debug: Option<&'static str>,
    /// Force and torque callback. If None, the global one will be used
    pub(crate) force_torque: Option<Box<dyn Fn(&mut NewtonBody<B, C>, Duration, raw::c_int)>>,
    /// Contained value
    contained: Option<B>,
}

impl<B, C> Drop for NewtonBodyData<B, C> {
    fn drop(&mut self) {
        println!("DROP NewtonBodyData");
    }
}

impl<B, C> NewtonBodyData<B, C> {
    /// Gets body data
    pub fn from(body: &NewtonBody<B, C>) -> Shared<Self> {
        unsafe { userdata(body.as_raw()) }
    }

    /// Gets the actual data
    pub fn get(&self) -> Option<&B> {
        self.contained.as_ref()
    }
}

pub struct BodyBuilder<'a, 'b, B, C> {
    world: &'a mut NewtonWorld<B, C>,
    collision: &'b NewtonCollision<B, C>,
    /// Force and torque callback
    force_torque: Option<Box<dyn Fn(&mut NewtonBody<B, C>, Duration, raw::c_int)>>,
    /// Type of the body (Dynamic or Kinematic)
    type_: Type,
    /// A name given to the collision.
    debug: Option<&'static str>,
    /// Initial transformation
    transform: Matrix,
    /// Material group ID
    material_group: GroupId,
    /// Continuous collision enabled/disabled
    continuous: bool,
    /// Mass
    mass: f32,
    /// Userdata value
    contained: Option<B>,
}

impl<'a, 'b, B, C> BodyBuilder<'a, 'b, B, C> {
    pub fn new(world: &'a mut NewtonWorld<B, C>, collision: &'b NewtonCollision<B, C>) -> Self {
        let world_ptr = world.as_raw();
        BodyBuilder {
            world,
            collision,
            force_torque: None,
            type_: Type::Dynamic,
            debug: None,
            transform: super::IDENTITY,
            material_group: unsafe { GroupId(ffi::NewtonMaterialGetDefaultGroupID(world_ptr)) },
            continuous: false,
            mass: 0.0,
            contained: None,
        }
    }

    pub fn data(mut self, data: B) -> Self {
        self.contained = Some(data);
        self
    }

    /// Enable continuous collision mode
    pub fn continuous(mut self) -> Self {
        self.continuous = true;
        self
    }

    /// Sets material GroupID
    pub fn material(mut self, group: GroupId) -> Self {
        self.material_group = group;
        self
    }

    /// Set the force and torque callback
    /// A None means the callback will not be called
    pub fn force_torque_callback<Callback>(mut self, callback: Callback) -> Self
    where
        Callback: Fn(&mut NewtonBody<B, C>, Duration, raw::c_int) + 'static,
    {
        self.force_torque = Some(Box::new(callback));
        self
    }

    /// Consumes the builder and returns a body
    pub fn build(self) -> Body<B, C> {
        Body::new(
            self.world,
            self.collision,
            self.type_,
            &self.transform,
            self.debug,
            self.force_torque,
            self.material_group,
            self.continuous,
            self.mass,
            self.contained,
        )
    }

    pub fn mass(mut self, mass: f32) -> Self {
        self.mass = mass;
        self
    }

    pub fn transform(mut self, transform: Matrix) -> Self {
        self.transform = transform;
        self
    }

    pub fn debug(mut self, name: &'static str) -> Self {
        self.debug = Some(name);
        self
    }

    pub fn dynamic(mut self) -> Self {
        self.type_ = Type::Dynamic;
        self
    }

    pub fn kinematic(mut self) -> Self {
        self.type_ = Type::Kinematic;
        self
    }
}

pub(crate) unsafe fn drop_body<B, C>(body: *const ffi::NewtonBody) {
    eprintln!("DROP body");
    let collision = ffi::NewtonBodyGetCollision(body);

    let _: Shared<NewtonBodyData<B, C>> = mem::transmute(ffi::NewtonBodyGetUserData(body));

    // decrement ref count of the collision
    let _: Shared<super::collision::NewtonCollisionData<B, C>> =
        mem::transmute(ffi::NewtonCollisionGetUserData(collision));

    ffi::NewtonDestroyBody(body);
}

pub(crate) unsafe fn userdata<B, C>(body: *const ffi::NewtonBody) -> Shared<NewtonBodyData<B, C>> {
    let udata: Shared<NewtonBodyData<B, C>> = mem::transmute(ffi::NewtonBodyGetUserData(body));
    let udata_cloned = udata.clone();
    mem::forget(udata);
    udata_cloned
}

macro_rules! body_iterator {
    (
        $struct_name:ident < 'a, B, C > ,
        $item:ty
    ) => {
        // TODO remove pub visibility
        /// Iterator that yields NewtonBodys
        #[derive(Debug)]
        pub struct $struct_name<'a, B, C> {
            pub(crate) world: *mut ffi::NewtonWorld,
            /// A raw pointer to the next body to be returned by the iterator
            pub(crate) next: *mut ffi::NewtonBody,
            /// The NewtonBody reference that gets returned by the iterator. This data is stored on the
            /// heap so it may have a slight effect in performance.
            pub(crate) body: *mut NewtonBody<B, C>,
            pub(crate) _phantom: PhantomData<&'a ()>,
        }
        impl<'a, B: 'a, C: 'a> Iterator for $struct_name<'a, B, C> {
            type Item = $item;

            fn next(&mut self) -> Option<Self::Item> {
                if self.next.is_null() {
                    None
                } else {
                    unsafe {
                        let mut boxed = unsafe { Box::from_raw(self.body) };
                        boxed.body = self.next;
                        boxed.collision.collision = ffi::NewtonBodyGetCollision(self.next);

                        self.next = ffi::NewtonWorldGetNextBody(self.world, boxed.body);
                        Some(mem::transmute(Box::into_raw(boxed)))
                    }
                }
            }
        }
        impl<'a, B, C> Drop for $struct_name<'a, B, C> {
            fn drop(&mut self) {
                unsafe {
                    let _: Box<NewtonBody<B, C>> = Box::from_raw(self.body);
                }
            }
        }
    };
}

body_iterator! { NewtonBodies<'a, B, C>, &'a NewtonBody<B, C> }
body_iterator! { NewtonBodiesMut<'a, B, C>, &'a mut NewtonBody<B, C> }

/// Reference to a NewtonWorld
#[derive(Debug)]
pub struct BodyLocked<'a, B, C>(Locked<'a, NewtonWorld<B, C>>, Locked<'a, NewtonBody<B, C>>);

/// Mutable reference to a NewtonWorld
#[derive(Debug)]
pub struct BodyLockedMut<'a, B, C>(
    LockedMut<'a, NewtonWorld<B, C>>,
    LockedMut<'a, NewtonBody<B, C>>,
);

impl<'a, B, C> BodyLockedMut<'a, B, C> {
    /// Destroys the body
    pub fn destroy(mut self) {
        if self.destroyed {
            return;
        }

        self.destroyed = true;
        unsafe { drop_body::<B, C>(self.body) };
    }
}

/// Dynamic / Kinematic type
#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum Type {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY as _,
    /// Bodies that only have an effect at collision level.
    Kinematic = ffi::NEWTON_KINEMATIC_BODY as _,
}

impl<B, C> Body<B, C> {
    /// Clones the ref-counted body
    pub fn clone(body: &Body<B, C>) -> Self {
        Body(Shared::clone(&body.0), Shared::clone(&body.1), body.2)
    }

    /// Downgrade ref-counted body
    pub fn downgrade(rc: &Body<B, C>) -> WeakBody<B, C> {
        WeakBody(Shared::downgrade(&rc.0), Shared::downgrade(&rc.1), rc.2)
    }

    /// Destroys the inner body. panic! if it can't
    pub fn destroy(self) {
        self.try_destroy().unwrap()
    }

    /// Destroys the inner `NewtonBody`
    pub fn try_destroy(self) -> Result<()> {
        self.try_write()?.destroy();
        Ok(())
    }

    pub unsafe fn from_raw_parts(body: *mut ffi::NewtonBody) -> Self {
        let udata = userdata(body);
        let body_rc = Weak::upgrade(&udata.body).unwrap();
        let world_rc = Weak::upgrade(&udata.world).unwrap();
        Body(world_rc, body_rc, body as *const _)
    }

    pub fn try_read(&self) -> Result<BodyLocked<B, C>> {
        let world = self.0.try_read()?;
        let body = self.1.try_read()?;
        if body.destroyed {
            Err(super::lock::LockError::Destroyed.into())
        } else {
            Ok(BodyLocked(world, body))
        }
    }

    pub fn try_write(&self) -> Result<BodyLockedMut<B, C>> {
        #[cfg(feature = "debug")]
        let name = unsafe { userdata::<B, C>(self.2).debug };
        #[cfg(not(feature = "debug"))]
        let name = None;

        let world = self.0.try_write(name)?;
        let body = self.1.try_write(name)?;

        if body.destroyed {
            Err(super::lock::LockError::Destroyed.into())
        } else {
            Ok(BodyLockedMut(world, body))
        }
    }

    pub fn read(&self) -> BodyLocked<B, C> {
        let world = self.0.read();
        let body = self.1.read();
        BodyLocked(world, body)
    }

    pub fn write(&self) -> BodyLockedMut<B, C> {
        #[cfg(feature = "debug")]
        let name = unsafe { userdata::<B, C>(self.2).debug };
        #[cfg(not(feature = "debug"))]
        let name = None;

        let world = self.0.write(name);
        let body = self.1.write(name);
        BodyLockedMut(world, body)
    }

    pub fn builder<'a, 'b>(
        world: &'a mut NewtonWorld<B, C>,
        collision: &'b NewtonCollision<B, C>,
    ) -> BodyBuilder<'a, 'b, B, C> {
        BodyBuilder::new(world, collision)
    }

    fn new(
        world: &mut NewtonWorld<B, C>,
        collision: &NewtonCollision<B, C>,
        type_: Type,
        matrix: &Matrix,
        debug: Option<&'static str>,
        force_torque: Option<Box<dyn Fn(&mut NewtonBody<B, C>, Duration, raw::c_int)>>,
        material: GroupId,
        continuous: bool,
        mass: f32,
        contained: Option<B>,
    ) -> Self {
        let collision = collision.as_raw();
        let world_ptr = world.as_raw_mut();

        unsafe {
            let transform = mem::transmute(matrix);
            let body_ptr = match type_ {
                Type::Dynamic => ffi::NewtonCreateDynamicBody(world_ptr, collision, transform),
                Type::Kinematic => ffi::NewtonCreateKinematicBody(world_ptr, collision, transform),
            };

            let world_udata = super::world::userdata(world_ptr);

            let world_lock = Weak::upgrade(&world_udata.world).unwrap();
            let body_lock = Shared::new(Lock::new(NewtonBody {
                body: body_ptr,
                world: world_lock.clone(),
                tx: Some(world.tx.clone()),
                collision: NewtonCollision::new_not_owned(collision as _),
                owned: true,
                destroyed: false,
            }));

            match &force_torque {
                &Some(_) => unsafe {
                    ffi::NewtonBodySetForceAndTorqueCallback(
                        body_ptr,
                        Some(super::callbacks::force_and_torque_callback::<B, C>),
                    );
                },
                _ => {}
            }

            ffi::NewtonBodySetMaterialGroupID(body_ptr, material.0);
            ffi::NewtonBodySetContinuousCollisionMode(body_ptr, if continuous { 1 } else { 0 });
            ffi::NewtonBodySetMassProperties(body_ptr, mass, collision);

            let userdata = Shared::new(NewtonBodyData {
                body: Shared::downgrade(&body_lock),
                world: Shared::downgrade(&world_lock),
                force_torque,
                debug,
                contained,
            });

            ffi::NewtonBodySetUserData(body_ptr, mem::transmute(userdata));
            mem::forget(super::collision::userdata::<B, C>(collision));

            Body(world_lock, body_lock, body_ptr)
        }
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Awake = 0,
    /// Dynamic body equilibrium state
    Sleeping = 1,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum FreezeState {
    Freeze = 1,
    Unfreeze = 0,
}

impl<B, C> NewtonBody<B, C> {
    pub(crate) unsafe fn null(world: Shared<Lock<NewtonWorld<B, C>>>) -> Self {
        NewtonBody {
            owned: false,
            destroyed: false,
            body: ptr::null_mut(),
            collision: NewtonCollision::null(world.clone()),
            world,
            tx: None,
        }
    }

    pub fn set_material_id(&mut self, id: GroupId) {
        unsafe { ffi::NewtonBodySetMaterialGroupID(self.body, id.0) }
    }

    pub fn material_id(&self) -> GroupId {
        unsafe { GroupId(ffi::NewtonBodyGetMaterialGroupID(self.body)) }
    }

    #[inline]
    pub fn collision(&self) -> &NewtonCollision<B, C> {
        &self.collision
    }

    pub fn contact_joints(&self) -> Contacts {
        unimplemented!()
    }

    pub fn joints(&self) -> Joints {
        unimplemented!()
    }

    #[inline]
    pub fn body_type(&self) -> Type {
        unsafe { mem::transmute(ffi::NewtonBodyGetType(self.body)) }
    }

    #[inline]
    pub fn is_dynamic(&self) -> bool {
        self.body_type() == Type::Dynamic
    }

    #[inline]
    pub fn is_kinematic(&self) -> bool {
        self.body_type() == Type::Kinematic
    }

    #[inline]
    pub fn as_raw(&self) -> *const ffi::NewtonBody {
        self.body
    }

    #[inline]
    pub fn as_raw_mut(&mut self) -> *mut ffi::NewtonBody {
        self.body
    }

    #[inline]
    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.body)) }
    }

    #[inline]
    pub fn set_sleep_state(&mut self, state: SleepState) {
        unsafe { ffi::NewtonBodySetSleepState(self.body, mem::transmute(state)) }
    }

    #[inline]
    pub fn set_freeze_state(&mut self, state: FreezeState) {
        unsafe { ffi::NewtonBodySetFreezeState(self.body, mem::transmute(state)) }
    }

    #[inline]
    pub fn awake(&mut self) {
        self.set_sleep_state(SleepState::Awake)
    }

    #[inline]
    pub fn id(&self) -> raw::c_int {
        unsafe { ffi::NewtonBodyGetID(self.body) }
    }

    #[inline]
    pub fn set_collidable(&mut self, collidable: bool) {
        unsafe { ffi::NewtonBodySetCollidable(self.body, collidable as _) };
    }

    #[inline]
    pub fn set_collision(&mut self, collision: &NewtonCollision<B, C>) {
        unsafe {
            let current_collision = ffi::NewtonBodyGetCollision(self.body);
            let new_collision = collision.as_raw();

            // decrement ref count of the collision
            let _: Shared<super::collision::NewtonCollisionData<B, C>> =
                mem::transmute(ffi::NewtonCollisionGetUserData(current_collision));

            ffi::NewtonBodySetCollision(self.body, new_collision);
            self.collision.collision = collision.as_raw() as _;

            mem::forget(super::collision::userdata::<B, C>(new_collision));
        }
    }

    /// Wraps a raw NewtonBody pointer
    pub(crate) unsafe fn new_not_owned(body: *mut ffi::NewtonBody) -> Self {
        let udata = userdata(body);
        let collision = ffi::NewtonBodyGetCollision(body);
        NewtonBody {
            owned: false,
            destroyed: false,
            body,
            world: Weak::upgrade(&udata.world).unwrap(),
            collision: NewtonCollision::new_not_owned(collision),
            tx: None,
        }
    }

    pub fn add_impulse(&mut self, vel: &Vector, point: &Vector, step: Duration) {
        unsafe {
            ffi::NewtonBodyAddImpulse(
                self.body,
                mem::transmute(vel),
                mem::transmute(point),
                as_seconds(step),
            )
        }

        fn as_seconds(step: Duration) -> f32 {
            let nanos = step.as_secs() as f32 * 1_000_000_000.0 + step.subsec_nanos() as f32;
            nanos / 1_000_000_000.0
        }
    }

    /// Returns the AABB in local space.
    #[inline]
    pub fn aabb(&self) -> (Vector, Vector) {
        unsafe {
            let (mut min, mut max) = mem::zeroed();
            ffi::NewtonBodyGetAABB(
                self.body,
                mem::transmute(&mut min),
                mem::transmute(&mut max),
            );
            (min, max)
        }
    }

    /// Returns the AABB in world space
    pub fn aabb_world(&self) -> (Vector, Vector) {
        unimplemented!()
    }

    /*
    #[inline]
    pub fn set_mass(&mut self, mass: f32) {
        unsafe {
            let collision = ffi::NewtonBodyGetCollision(self.body);
            ffi::NewtonBodySetMassProperties(self.body, mass, collision);
        }
    }
    */

    #[inline]
    pub fn set_linear_damping(&mut self, damping: f32) {
        unsafe { ffi::NewtonBodySetLinearDamping(self.body, damping) };
    }

    #[inline]
    pub fn set_angular_damping(&mut self, damping: &Vector) {
        unsafe { ffi::NewtonBodySetAngularDamping(self.body, mem::transmute(damping)) };
    }

    #[inline]
    pub fn set_matrix(&mut self, matrix: &Matrix) {
        unsafe { ffi::NewtonBodySetMatrix(self.body, mem::transmute(matrix)) };
    }

    #[inline]
    pub fn set_velocity(&mut self, matrix: &Vector) {
        unsafe { ffi::NewtonBodySetVelocity(self.body, mem::transmute(matrix)) };
    }

    #[inline]
    pub fn set_force(&mut self, force: &Vector) {
        unsafe { ffi::NewtonBodySetForce(self.body, mem::transmute(force)) };
    }

    #[inline]
    pub fn set_torque(&mut self, torque: &Vector) {
        unsafe { ffi::NewtonBodySetTorque(self.body, mem::transmute(torque)) };
    }

    #[inline]
    pub fn set_matrix_no_sleep(&mut self, matrix: &Matrix) {
        unsafe { ffi::NewtonBodySetMatrixNoSleep(self.body, mem::transmute(matrix)) };
    }

    #[inline]
    pub fn matrix(&self) -> Matrix {
        unsafe {
            let mut mat: Matrix = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.body, mem::transmute(&mut mat));
            mat
        }
    }

    #[inline]
    pub fn position(&self) -> Vector {
        unsafe {
            let mut pos: Vector = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.body, mem::transmute(&mut pos));
            pos
        }
    }

    /*
    pub fn apply_force_and_torque(&mut self) {
        unsafe {
            ffi::NewtonBodySetForceAndTorqueCallback(
                self.body,
                Some(force_and_torque_callback::<T>),
            );
        }
    }
    */
}

impl<'a, B, C> Deref for BodyLocked<'a, B, C> {
    type Target = NewtonBody<B, C>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, B, C> Deref for BodyLockedMut<'a, B, C> {
    type Target = NewtonBody<B, C>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.1.deref()
    }
}

impl<'a, B, C> DerefMut for BodyLockedMut<'a, B, C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1.deref_mut()
    }
}

impl<B, C> Drop for NewtonBody<B, C> {
    fn drop(&mut self) {
        if self.destroyed || !self.owned {
            return;
        }

        unsafe {
            if let Ok(b) = self.world.try_write(None) {
                drop_body::<B, C>(self.body)
            } else {
                self.tx
                    .as_ref()
                    .unwrap()
                    .send(Command::DestroyBody(self.body))
                    .unwrap();
            }
        }
    }
}
