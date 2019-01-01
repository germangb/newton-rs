use ffi;

use super::body::NewtonBody;
use super::collision::NewtonCollision;
use super::material::NewtonMaterial;
use super::Types;

use std::mem;
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
    }
}

pub unsafe extern "C" fn aabb_overlap_callback<T, C>(
    material: *const ffi::NewtonMaterial,
    body0: *const ffi::NewtonBody,
    body1: *const ffi::NewtonBody,
    thread_index: raw::c_int,
) -> raw::c_int
where
    C: Fn(&NewtonMaterial<T>, &NewtonBody<T>, &NewtonBody<T>, raw::c_int) -> bool + 'static,
{
    let userdata = ffi::NewtonMaterialGetMaterialPairUserData(material);

    if !userdata.is_null() {
        let boxed_callback: Box<C> = Box::from_raw(userdata as _);
        //let resol = boxed_callback();

        let material = NewtonMaterial::from_raw(material as _);
        let body0 = NewtonBody::new_not_owned(body0 as _);
        let body1 = NewtonBody::new_not_owned(body1 as _);

        let result = boxed_callback(&material, &body0, &body1, thread_index);

        mem::forget(boxed_callback);

        if result {
            1
        } else {
            0
        }
    } else {
        1
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
    println!("{}", max_count);
    max_count
}
