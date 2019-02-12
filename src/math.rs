/// 4x4 Matrix type
///
/// The matrix is arranged in columns.
pub type Matrix = [[f32; 4]; 4];
/// 3D Vector type
pub type Vector = [f32; 3];
/// Quaternion type.
///
/// The first component of the array corresponds to the real component of the quaternion.
pub type Quaternion = [f32; 4];

/// Returns an identity matrix
pub const fn identity() -> Matrix {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}
