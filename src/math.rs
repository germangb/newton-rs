use crate::ffi;

/// 3D vector
pub type Vec3 = [f32; 3];
/// 4D vector
pub type Vec4 = [f32; 4];
/// Quaternion
pub type Quat = [f32; 4];
/// 4x4 matrix, arranged in columns
pub type Mat4 = [Vec4; 4];

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
    let mut sel0 = [0.0, 0.0, 0.0];
    let mut sel1 = [0.0, 0.0, 0.0];
    unsafe {
        ffi::NewtonGetEulerAngle(matrix[0].as_ptr(), sel0.as_mut_ptr(), sel1.as_mut_ptr());
    }
    (sel0, sel1)
}
