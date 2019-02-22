use super::ffi;

/// Converts from Euler angles to a 4x4 rotation matrix.
pub fn set_euler_angle(euler: [f32; 3]) -> [[f32; 4]; 4] {
    let mut mat: [[f32; 4]; 4] = Default::default();
    unsafe {
        ffi::NewtonSetEulerAngle(euler.as_ptr(), mat[0].as_mut_ptr());
    }
    mat
}

/// Converts a 4x4 rotation matrix to Euler angles.
///
/// http://newtondynamics.com/wiki/index.php5?title=NewtonGetEulerAngle
pub fn get_euler_angles(matrix: [[f32; 4]; 4]) -> ([f32; 3], [f32; 3]) {
    let mut sel0: [f32; 3] = Default::default();
    let mut sel1: [f32; 3] = Default::default();
    unsafe {
        ffi::NewtonGetEulerAngle(matrix[0].as_ptr(), sel0.as_mut_ptr(), sel1.as_mut_ptr());
    }
    (sel0, sel1)
}
/// Calculates acceleration that satisfies a given damper system.
///
/// http://newtondynamics.com/wiki/index.php5?title=NewtonCalculateSpringDamperAcceleration
pub fn calculate_spring_damper_acceleration(dt: f32, ks: f32, x: f32, kd: f32, s: f32) -> f32 {
    unsafe { ffi::NewtonCalculateSpringDamperAcceleration(dt, ks, x, kd, s) }
}
