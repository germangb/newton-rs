pub extern crate newton_sys as ffi;

macro_rules! assert_config {
    ($v:tt) => {
        assert_eq!(
            ::std::mem::size_of::<$v::Vector3>(),
            3 * ::std::mem::size_of::<f32>()
        );
        assert_eq!(
            ::std::mem::size_of::<$v::Vector4>(),
            4 * ::std::mem::size_of::<f32>()
        );
        assert_eq!(
            ::std::mem::size_of::<$v::Matrix4>(),
            16 * ::std::mem::size_of::<f32>()
        );
        assert_eq!(
            ::std::mem::size_of::<$v::Quaternion>(),
            4 * ::std::mem::size_of::<f32>()
        );
    };
}

pub mod body;
pub mod callback;
pub mod collision;
pub mod joint;
pub mod mesh;
pub mod world;

use self::body::NewtonBody;

pub trait NewtonConfig: std::fmt::Debug {
    /// Default gravity
    const GRAVITY: Self::Vector3;
    /// 3D vector type.
    ///
    /// It should have a size of 3 * std::mem::size_of::<f32>()
    type Vector3: Copy;
    /// 4D vector type
    ///
    /// It should have a size of 4 * std::mem::size_of::<f32>()
    type Vector4: Copy;
    /// 4x4 Matrix type. The first 4 `f32`s correspond to the first column, the next 4 to the
    /// second column and so on...
    ///
    /// It should have a size of 16 * std::mem::size_of::<f32>(),
    type Matrix4: Copy;
    /// Quaternion type
    type Quaternion: Copy;
}
