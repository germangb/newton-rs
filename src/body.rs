pub mod iter;

use ffi;

use super::collision::NewtonCollision;
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

#[derive(Debug)]
pub struct Body<B, C> {
    world: Shared<Lock<NewtonWorld<B, C>>>,
    body: Lock<NewtonBody<B, C>>,
    /// An optional name used in Error reporting
    debug: Option<&'static str>,
}

#[derive(Debug)]
pub struct NewtonBody<B, C> {
    // TODO remove pub(crate). It is used by the convex cast
    pub(crate) body: *mut ffi::NewtonBody,
    /// Bodies must be dropped before the world is.
    ///
    /// This field is optional because it is only needed when the struct is owned
    world: Option<Shared<Lock<NewtonWorld<B, C>>>>,
    /// Owned NewtonBody s, when dropped, are destroyed and therefore removed from the simulation
    owned: bool,
    /// The Tx end of the command channel. Only for owned variants,
    ///
    /// Like world , this field is optional because it is only needed when the struct is owned
    tx: Option<Tx<Command>>,
    /// Non-owned collision
    collision: NewtonCollision<B, C>,
}

// Compiler complains about the raw pointer
unsafe impl<B, C> Send for NewtonBody<B, C> {}
unsafe impl<B, C> Sync for NewtonBody<B, C> {}

//#[doc(hidden)]
pub struct NewtonBodyData<B, C> {
    /// Force and torque callback. If None, the global one will be used
    pub(crate) force_torque: Option<Shared<dyn Fn(&mut NewtonBody<B, C>, Duration, raw::c_int)>>,
    /// Contained value
    contained: Option<B>,
    _phantom: PhantomData<C>,
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
    force_torque: Option<Shared<dyn Fn(&mut NewtonBody<B, C>, Duration, raw::c_int)>>,
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

impl<'a, 'b, B: Clone, C> BodyBuilder<'a, 'b, B, C> {
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

    pub fn data(&mut self, data: B) -> &mut Self {
        self.contained = Some(data);
        self
    }

    /// Enable continuous collision mode
    pub fn continuous(&mut self) -> &mut Self {
        self.continuous = true;
        self
    }

    /// Sets material GroupID
    pub fn material(&mut self, group: GroupId) -> &mut Self {
        self.material_group = group;
        self
    }

    /// Set the force and torque callback
    /// A None means the callback will not be called
    pub fn force_torque_callback<Callback>(&mut self, callback: Callback) -> &mut Self
    where
        Callback: Fn(&mut NewtonBody<B, C>, Duration, raw::c_int) + 'static,
    {
        self.force_torque = Some(Shared::new(callback));
        self
    }

    pub fn mass(&mut self, mass: f32) -> &mut Self {
        self.mass = mass;
        self
    }

    pub fn transform(&mut self, transform: Matrix) -> &mut Self {
        self.transform = transform;
        self
    }

    pub fn debug(&mut self, name: &'static str) -> &mut Self {
        self.debug = Some(name);
        self
    }

    pub fn dynamic(&mut self, transform: &Matrix) -> Body<B, C> {
        self.build(Type::Dynamic, transform)
    }

    pub fn kinematic(&mut self, transform: &Matrix) -> Body<B, C> {
        self.build(Type::Kinematic, transform)
    }

    fn build(&mut self, type_: Type, transform: &Matrix) -> Body<B, C> {
        Body::new(
            self.world,
            self.collision,
            type_,
            transform,
            self.debug,
            self.force_torque.clone(),
            self.material_group,
            self.continuous,
            self.mass,
            self.contained.clone(),
        )
    }
}

pub(crate) unsafe fn drop_body<B, C>(body: *const ffi::NewtonBody) {
    //eprintln!("DROP body");
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

/// Reference to a NewtonWorld
#[derive(Debug)]
pub struct BodyLocked<'a, B, C>(Locked<'a, NewtonWorld<B, C>>, Locked<'a, NewtonBody<B, C>>);

/// Mutable reference to a NewtonWorld
#[derive(Debug)]
pub struct BodyLockedMut<'a, B, C>(
    LockedMut<'a, NewtonWorld<B, C>>,
    LockedMut<'a, NewtonBody<B, C>>,
);

/// Dynamic / Kinematic type
#[repr(i32)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub enum Type {
    Dynamic = ffi::NEWTON_DYNAMIC_BODY as _,
    /// Bodies that only have an effect at collision level.
    Kinematic = ffi::NEWTON_KINEMATIC_BODY as _,
}

impl<B: Clone, C> Body<B, C> {
    pub fn builder<'a, 'b>(
        world: &'a mut NewtonWorld<B, C>,
        collision: &'b NewtonCollision<B, C>,
    ) -> BodyBuilder<'a, 'b, B, C> {
        BodyBuilder::new(world, collision)
    }
}

impl<B, C> Body<B, C> {
    pub fn try_read(&self) -> Result<BodyLocked<B, C>> {
        let body = self.body.try_read()?;
        let world = self.world.try_read()?;
        Ok(BodyLocked(world, body))
    }

    pub fn try_write(&self) -> Result<BodyLockedMut<B, C>> {
        let body = self.body.try_write(self.debug)?;
        let world = self.world.try_write(self.debug)?;
        Ok(BodyLockedMut(world, body))
    }

    pub fn read(&self) -> BodyLocked<B, C> {
        BodyLocked(self.world.read(), self.body.read())
    }

    pub fn write(&self) -> BodyLockedMut<B, C> {
        let debug = self.debug;
        BodyLockedMut(self.world.write(debug), self.body.write(debug))
    }

    fn new(
        world: &mut NewtonWorld<B, C>,
        collision: &NewtonCollision<B, C>,
        type_: Type,
        matrix: &Matrix,
        debug: Option<&'static str>,
        force_torque: Option<Shared<dyn Fn(&mut NewtonBody<B, C>, Duration, raw::c_int)>>,
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
            let body_lock = Lock::new(NewtonBody {
                body: body_ptr,
                world: Some(world_lock.clone()),
                tx: Some(world.tx.clone()),
                owned: true,
                collision: NewtonCollision::new_not_owned(collision as _),
            });

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
                force_torque,
                contained,
                _phantom: PhantomData::<C>,
            });

            ffi::NewtonBodySetUserData(body_ptr, mem::transmute(userdata));
            mem::forget(super::collision::userdata::<B, C>(collision));

            Body {
                world: world_lock,
                body: body_lock,
                debug,
            }
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
    /// Wraps a raw NewtonBody pointer
    ///
    /// TODO document safety
    pub(crate) fn new_not_owned(body: *mut ffi::NewtonBody) -> Self {
        NewtonBody {
            owned: false,
            body,
            world: None,
            tx: None,
            collision: NewtonCollision::new_not_owned(unsafe { ffi::NewtonBodyGetCollision(body) }),
        }
    }

    pub(crate) fn null_not_owned() -> Self {
        NewtonBody {
            owned: false,
            body: ptr::null_mut(),
            world: None,
            tx: None,
            collision: NewtonCollision::null_not_owned(),
        }
    }

    pub fn collision(&self) -> &NewtonCollision<B, C> {
        &self.collision
    }

    pub fn set_material_id(&mut self, id: GroupId) {
        unsafe { ffi::NewtonBodySetMaterialGroupID(self.body, id.0) }
    }

    pub fn material_id(&self) -> GroupId {
        unsafe { GroupId(ffi::NewtonBodyGetMaterialGroupID(self.body)) }
    }

    #[inline]
    fn body_type(&self) -> Type {
        unsafe { mem::transmute(ffi::NewtonBodyGetType(self.body)) }
    }

    #[inline]
    pub fn dynamic(&self) -> bool {
        self.body_type() == Type::Dynamic
    }

    #[inline]
    pub fn kinematic(&self) -> bool {
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

    #[inline]
    pub fn set_mass_params(&mut self, mass: f32, collision: &NewtonCollision<B, C>) {
        unsafe {
            let collision = collision.as_raw();
            ffi::NewtonBodySetMassProperties(self.body, mass, collision);
        }
    }

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
        if !self.owned {
            return;
        }

        let world = self
            .world
            .as_ref()
            .expect("world field from owned NewtonWorld expected to be Some");

        if let Ok(b) = world.try_write(None) {
            unsafe { drop_body::<B, C>(self.body) };
        } else {
            self.tx
                .as_ref()
                .expect("tx field from owned NewtonWorld expected to be Some")
                .send(Command::DestroyBody(self.body))
                .unwrap();
        }
    }
}
