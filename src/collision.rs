use std::mem;
use std::os::raw;

use super::ffi;
use super::world::Newton;
use super::{Matrix, Vector};

#[derive(Debug, Hash, Eq, PartialEq, Clone, Copy)]
pub struct Handle(pub(crate) *const ffi::NewtonCollision);

unsafe impl Send for Handle {}
unsafe impl Sync for Handle {}

#[derive(Debug)]
pub struct Collision<'world> {
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

unsafe impl<'world> Send for Collision<'world> {}
unsafe impl<'world> Sync for Collision<'world> {}

impl<'world> Collision<'world> {
    fn from_ptr(newton: &'world Newton, collision: *mut ffi::NewtonCollision) -> Self {
        Self {
            newton,
            collision,
            owned: true,
        }
    }

    pub fn cylinder(
        newton: &'world Newton,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<&Matrix>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset);
            let collision =
                ffi::NewtonCreateCylinder(newton.as_ptr(), radius0, radius1, height, 0, offset);
            Self::from_ptr(newton, collision)
        }
    }

    pub fn capsule(
        newton: &'world Newton,
        radius0: f32,
        radius1: f32,
        height: f32,
        offset: Option<&Matrix>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset);
            let collision =
                ffi::NewtonCreateCapsule(newton.as_ptr(), radius0, radius1, height, 0, offset);
            Self::from_ptr(newton, collision)
        }
    }

    /// Creates a sphere collision.
    ///
    /// Unless `into_handle` is called, this collision will be freed when the type is dropped.
    pub fn sphere(newton: &'world Newton, radius: f32, offset: Option<&Matrix>) -> Self {
        unsafe {
            let offset = mem::transmute(offset);
            let collision = ffi::NewtonCreateSphere(newton.as_ptr(), radius, 0, offset);
            Self::from_ptr(newton, collision)
        }
    }

    /// Creates a box collision.
    ///
    /// Method name is `cuboid` because `box` is reserved keyword...
    /// When the returned object is dropped, unless `into_handle` is
    /// called, the collision will be freed.
    pub fn cuboid(
        newton: &'world Newton,
        dx: f32,
        dy: f32,
        dz: f32,
        offset: Option<&Matrix>,
    ) -> Self {
        unsafe {
            let offset = mem::transmute(offset);
            let collision = ffi::NewtonCreateBox(newton.as_ptr(), dx, dy, dz, 0, offset);
            Self::from_ptr(newton, collision)
        }
    }

    /// Creates a null collision, which is a collision with no geometry (a point).
    ///
    /// Unless `into_handle` is called, this collision will be free when the type is dropped.
    pub fn null(newton: &'world Newton) -> Self {
        unsafe { Self::from_ptr(newton, ffi::NewtonCreateNull(newton.as_ptr())) }
    }

    /// Transfers ownership to the Newton context.
    ///
    /// The point of this method is to be able to hold a reference to a NewtonCollision
    /// without borrowing the Newton context indefinitely (so you can have a long-lived
    /// collision that you can borrow again later on).
    ///
    /// The returned handle can be used to borrow or take ownership back from the Newton
    /// context by its [`collision`][col] and [`collision_owned`][col_own] methods.
    ///
    /// [col]: #
    /// [col_owned]: #
    ///
    /// ## Example
    /// ```
    /// use newton::{Newton, Collision};
    ///
    /// let mut newton = Newton::default();
    ///
    /// let handle = Collision::sphere(&newton, 1.0, None).into_handle();
    ///
    /// // take ownership back to free the collision
    /// let _ = newton.collision_owned(&handle).unwrap();
    /// ```
    pub fn into_handle(self) -> Handle {
        self.newton.move_collision(self)
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
    pub const fn as_ptr(&self) -> *const ffi::NewtonCollision {
        self.collision
    }
}

/// FFI wrappers
impl<'world> Collision<'world> {
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
            ffi::NewtonCollisionForEachPolygonDo(self.as_ptr(), matrix, Some(iterator::<F>), udata);
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
        unsafe { ffi::NewtonCollisionSetScale(self.as_ptr(), s.x, s.y, s.z) }
    }

    pub fn set_matrix(&self, mat: &Matrix) {
        unsafe { ffi::NewtonCollisionSetMatrix(self.as_ptr(), mat.as_ptr()) }
    }

    pub fn matrix(&self) -> Matrix {
        unsafe {
            let mut matrix: Matrix = mem::zeroed();
            ffi::NewtonCollisionGetMatrix(self.as_ptr(), matrix.as_mut_ptr() as *const f32);
            matrix
        }
    }

    pub fn scale(&self) -> Vector {
        let mut scale = Vector::zero();
        unsafe {
            #[rustfmt::skip]
            let &mut Vector { mut x, mut y, mut z, .. } = &mut scale;
            ffi::NewtonCollisionGetScale(self.as_ptr(), &mut x, &mut y, &mut z);
        }
        scale
    }
}

impl<'world> Drop for Collision<'world> {
    fn drop(&mut self) {
        if self.owned {
            unsafe { ffi::NewtonDestroyCollision(self.as_ptr()) }
        }
    }
}
