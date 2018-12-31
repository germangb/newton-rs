use ffi;

use super::body::NewtonBody;
use super::collision::NewtonCollision;
use super::Types;

use std::os::raw;
use std::time::Duration;

pub unsafe extern "C" fn force_and_torque_callback<T: Types>(
    body: *const ffi::NewtonBody,
    timestep: raw::c_float,
    thread: raw::c_int,
) {
    fn to_duration(timestep: f32) -> Duration {
        const NANO: u64 = 1_000_000_000;
        let nanos = (NANO as f32 * timestep) as u64;
        Duration::new(nanos / NANO, (nanos % NANO) as u32)
    }

    // TODO refactor
    let body_udata = super::body::userdata::<T>(body);

    if let &Some(ref callback) = &body_udata.force_torque {
        let mut body = NewtonBody::<T>::new_not_owned(body as _);
        callback(&mut body, to_duration(timestep), thread);
    } else {
        let world = ffi::NewtonBodyGetWorld(body);
        let world_udata = super::world::userdata::<T>(world);

        if let &Some(ref callback) = &world_udata.force_torque {
            let mut body = NewtonBody::<T>::new_not_owned(body as _);
            callback(&mut body, to_duration(timestep), thread);
        }
    }
}

pub unsafe extern "C" fn contact_generation_callback(
    material: *const ffi::NewtonMaterial,
    body0: *const ffi::NewtonBody,
    collision0: *const ffi::NewtonCollision,
    body1: *const ffi::NewtonBody,
    collision1: *const ffi::NewtonCollision,
    contact_buffer: *const ffi::NewtonUserContactPoint,
    max_count: raw::c_int,
    thread_index: raw::c_int,
) -> raw::c_int {
    0
}
