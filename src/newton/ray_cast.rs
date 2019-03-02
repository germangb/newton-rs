use std::mem;
use std::os::raw::{c_longlong, c_void};
use std::ptr;

use crate::body::Body;
use crate::collision::Collision;
use crate::ffi;
use crate::math::Vec3;
use crate::newton::Newton;

/// RayCast hit.
pub struct RayHit<'a> {
    pub body: Body<'a>,
    pub collision: Collision<'a>,
    pub position: Vec3,
    pub normal: Vec3,
    pub collision_id: c_longlong,
    pub intersect_param: f32,
}

/// Ray-cast algorithm
pub trait RayCastAlgorithm<'a> {
    /// Algorithm-specific params.
    type Params: Default;
    /// RayCast hit(s).
    type Result;
    fn ray_cast(newton: &'a Newton,
                p0: Vec3,
                p1: Vec3,
                params: Self::Params)
                -> Option<Self::Result>;
}

/// Return the closest hit
pub enum ClosestHit {}
/// Return all intersecting hits.
pub enum AllHits {}
/// Return the N closest hits.
pub enum NClosestHits {}

/// Parameters for the user-defined RayCasting algorithm
pub struct CustomParams<F, P> {
    pub filter: F,
    pub prefilter: P,
}

/// User-defined algorithm.
pub enum Custom {}

impl<'a> RayCastAlgorithm<'a> for ClosestHit {
    type Params = ();
    type Result = RayHit<'a>;

    fn ray_cast(newton: &'a Newton, p0: Vec3, p1: Vec3, _: Self::Params) -> Option<Self::Result> {
        #[derive(Default, Clone, Copy)]
        struct Udata {
            param: Option<f32>,
            body: Option<*const ffi::NewtonBody>,
            col: Option<*const ffi::NewtonCollision>,
            contact: Option<Vec3>,
            normal: Option<Vec3>,
            col_id: Option<c_longlong>,
        }

        let mut udata = Udata::default();

        unsafe {
            ffi::NewtonWorldRayCast(newton.as_raw(),
                                    p0.as_ptr(),
                                    p1.as_ptr(),
                                    Some(cfilter),
                                    mem::transmute(&mut udata),
                                    None,
                                    0 as _);
        }

        return if let Udata { body: Some(body),
                              col: Some(col),
                              contact: Some(contact),
                              normal: Some(normal),
                              param: Some(param),
                              col_id: Some(col_id), } = udata
        {
            unsafe {
                Some(RayHit { body: Body::from_raw(body, false),
                              collision: Collision::from_raw(col, false),
                              collision_id: col_id,
                              position: contact,
                              intersect_param: param,
                              normal })
            }
        } else {
            None
        };

        unsafe extern "C" fn cfilter(body: *const ffi::NewtonBody,
                                     collision: *const ffi::NewtonCollision,
                                     contact: *const f32,
                                     normal: *const f32,
                                     collision_id: c_longlong,
                                     user_data: *const c_void,
                                     intersect_param: f32)
                                     -> f32 {
            let mut udata = mem::transmute::<_, &mut Udata>(user_data);

            if intersect_param < udata.param.unwrap_or(2.0) {
                udata.contact = Some(*mem::transmute::<_, &Vec3>(contact));
                udata.normal = Some(*mem::transmute::<_, &Vec3>(normal));
                udata.param = Some(intersect_param);
                udata.body = Some(body);
                udata.col = Some(collision);
                udata.col_id = Some(collision_id);
            }

            udata.param.unwrap_or(intersect_param)
        }
    }
}

impl<'a> RayCastAlgorithm<'a> for NClosestHits {
    type Params = usize;
    type Result = Vec<RayHit<'a>>;

    fn ray_cast(newton: &'a Newton,
                p0: Vec3,
                p1: Vec3,
                params: Self::Params)
                -> Option<Self::Result> {
        unimplemented!()
    }
}
