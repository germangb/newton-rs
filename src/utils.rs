use super::ffi;
use super::{Mat4, Vec3};

/// Converts from Euler angles to a 4x4 rotation matrix.
pub fn set_euler_angle(euler: Vec3) -> Mat4 {
    let mut mat: Mat4 = Default::default();
    unsafe {
        ffi::NewtonSetEulerAngle(euler.as_ptr(), mat[0].as_mut_ptr());
    }
    mat
}

/// Converts a 4x4 rotation matrix to Euler angles.
///
/// http://newtondynamics.com/wiki/index.php5?title=NewtonGetEulerAngle
pub fn get_euler_angles(matrix: Mat4) -> (Vec3, Vec3) {
    let mut sel0: Vec3 = Default::default();
    let mut sel1: Vec3 = Default::default();
    unsafe {
        ffi::NewtonGetEulerAngle(matrix[0].as_ptr(), sel0.as_mut_ptr(), sel1.as_mut_ptr());
    }
    (sel0, sel1)
}
