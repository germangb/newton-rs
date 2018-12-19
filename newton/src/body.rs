use crate::ffi;
use crate::world::WorldRef;
use crate::NewtonData;

use std::marker::PhantomData;
use std::mem;
use std::rc::Rc;

/// Reference counted body
pub type NewtonBody<V> = Rc<NewtonBodyInner<V>>;

#[derive(Debug)]
pub struct NewtonBodyInner<V> {
    pub(crate) world: Rc<WorldRef>,
    pub(crate) body: *mut ffi::NewtonBody,
    pub(crate) _ph: PhantomData<V>,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SleepState {
    Sleeping = 1,
    Awake = 0,
}

enum BodyMass<V: NewtonData> {
    Compute,
    PrincipalAxis { ix: f32, iy: f32, iz: f32 },
    FullMatrix { matrix: V::Matrix4 },
}

pub struct NewtonBodyBuilder<V: NewtonData> {
    world: Rc<WorldRef>,
    matrix: V::Matrix4,
    collision: *mut ffi::NewtonCollision,
    mass: Option<(f32, BodyMass<V>)>,
}

impl<V: NewtonData> NewtonBodyBuilder<V> {
    pub(crate) fn new(
        world: Rc<WorldRef>,
        collision: *mut ffi::NewtonCollision,
        matrix: V::Matrix4,
    ) -> NewtonBodyBuilder<V> {
        NewtonBodyBuilder {
            world,
            matrix,
            collision,
            mass: None,
        }
    }

    /// Compute mass properties from the shape itself
    pub fn mass_compute(mut self, mass: f32) -> NewtonBodyBuilder<V> {
        self.mass = Some((mass, BodyMass::Compute));
        self
    }

    /// Set the principal components of the Inerta matrix
    pub fn mass_matrix(mut self, mass: f32, (ix, iy, iz): (f32, f32, f32)) -> NewtonBodyBuilder<V> {
        self.mass = Some((mass, BodyMass::PrincipalAxis { ix, iy, iz }));
        self
    }

    /// Set the full Inertia matrix
    pub fn mass_matrix_full(mut self, mass: f32, inertia: V::Matrix4) -> NewtonBodyBuilder<V> {
        self.mass = Some((mass, BodyMass::FullMatrix { matrix: inertia }));
        self
    }

    pub fn build(self) -> NewtonBody<V> {
        unsafe {
            // XXX fix pointer
            let ptr = &self.matrix as *const _ as *const f32;
            let body = ffi::NewtonCreateDynamicBody(self.world.0, self.collision, ptr);

            // XXX remove this callback
            ffi::NewtonBodySetForceAndTorqueCallback(body, Some(cb_apply_force));

            match self.mass {
                Some((m, BodyMass::Compute)) => {
                    ffi::NewtonBodySetMassProperties(body, m, self.collision)
                }
                Some((m, BodyMass::PrincipalAxis { ix, iy, iz })) => {
                    ffi::NewtonBodySetMassMatrix(body, m, ix, iy, iz)
                }
                Some((m, BodyMass::FullMatrix { matrix })) => {
                    // XXX pointers
                    let mat_ptr = &matrix as *const _ as *const f32;
                    ffi::NewtonBodySetFullMassMatrix(body, m, mat_ptr);
                }
                _ => {}
            }

            let body = Rc::new(NewtonBodyInner {
                world: self.world,
                body,
                _ph: PhantomData,
            });

            let datum = mem::transmute(Rc::downgrade(&body));
            ffi::NewtonBodySetUserData(body.body, datum);

            body
        }
    }
}

// XXX pointers
impl<V> NewtonBodyInner<V>
where
    V: NewtonData,
{
    pub fn matrix(&self) -> V::Matrix4 {
        unsafe {
            let mut mat = mem::zeroed();
            ffi::NewtonBodyGetMatrix(self.body, &mut mat as *mut _ as *mut f32);
            mat
        }
    }

    pub fn position(&self) -> V::Vector3 {
        unsafe {
            let mut pos = mem::zeroed();
            ffi::NewtonBodyGetPosition(self.body, &mut pos as *mut _ as *mut f32);
            pos
        }
    }

    pub fn rotation(&self) -> V::Quaternion {
        unsafe {
            let mut rotation = mem::zeroed();
            ffi::NewtonBodyGetRotation(self.body, &mut rotation as *mut _ as *mut f32);
            rotation
        }
    }

    pub fn aabb(&self) -> (V::Vector3, V::Vector3) {
        unsafe {
            let mut p0: V::Vector3 = mem::zeroed();
            let mut p1: V::Vector3 = mem::zeroed();
            ffi::NewtonBodyGetAABB(
                self.body,
                &mut p0 as *mut _ as *mut f32,
                &mut p1 as *mut _ as *mut f32,
            );
            (p0, p1)
        }
    }

    pub fn sleep_state(&self) -> SleepState {
        unsafe { mem::transmute(ffi::NewtonBodyGetSleepState(self.body)) }
    }
}

impl<V> Drop for NewtonBodyInner<V> {
    fn drop(&mut self) {
        unsafe {
            ffi::NewtonDestroyBody(self.body);
        }
    }
}

// XXX
extern "C" fn cb_apply_force(body: *const ffi::NewtonBody, _timestep: f32, _thread_idx: i32) {
    unsafe {
        ffi::NewtonBodySetForce(body, [0.0, -4.0, 0.0].as_ptr());
    }
}
