use ffi;

use super::body::NewtonBody;
use super::collision::NewtonCollision;
use super::material::NewtonMaterial;

use std::mem;
use std::os::raw;
use std::time::Duration;

pub unsafe extern "C" fn force_and_torque_callback<B, C>(
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
    let body_udata = super::body::userdata::<B, C>(body);

    if let &Some(ref callback) = &body_udata.force_torque {
        let mut body = NewtonBody::new_not_owned(body as _);
        callback(&mut body, to_duration(timestep), thread);
    }
}

pub unsafe extern "C" fn aabb_overlap_callback<B, C, Callback>(
    material: *const ffi::NewtonMaterial,
    body0: *const ffi::NewtonBody,
    body1: *const ffi::NewtonBody,
    thread_index: raw::c_int,
) -> raw::c_int
where
    Callback:
        Fn(&NewtonMaterial, &NewtonBody<B, C>, &NewtonBody<B, C>, raw::c_int) -> bool + 'static,
{
    let userdata = ffi::NewtonMaterialGetMaterialPairUserData(material);

    if !userdata.is_null() {
        let boxed_callback: Box<Callback> = Box::from_raw(userdata as _);
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

pub unsafe extern "C" fn contacts_process_callback(
    contact: *const ffi::NewtonJoint,
    timestep: f32,
    thread_index: raw::c_int,
) {
    let material = ffi::NewtonContactGetMaterial(contact as _);
    //eprintln!("{:?}", material);
    //println!("hit");
}
