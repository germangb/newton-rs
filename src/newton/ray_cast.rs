use std::cmp::{Eq, Ord, Ordering, PartialEq, PartialOrd};
use std::collections::BinaryHeap;
use std::mem;
use std::os::raw::{c_longlong, c_void};

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
    fn ray_cast(newton: &'a Newton, p0: Vec3, p1: Vec3, params: Self::Params) -> Self::Result;
}

/// Return the closest hit
pub enum ClosestHit {}
/// Return all intersecting hits.
pub enum AllHits {}
/// Return the N closest hits.
pub enum NClosestHits {}
/// Returns the first body found by the Newton API.
///
/// CUsing this algorithm may not always return the same result. It can be used as a binary check
/// to test whether a ray intersects something or not.
pub enum AnyHit {}

impl<'a> RayCastAlgorithm<'a> for ClosestHit {
    type Params = ();
    type Result = Option<RayHit<'a>>;

    fn ray_cast(newton: &'a Newton, p0: Vec3, p1: Vec3, _: Self::Params) -> Self::Result {
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

    fn ray_cast(newton: &'a Newton, p0: Vec3, p1: Vec3, params: Self::Params) -> Self::Result {
        struct Node {
            intersect: f32,
            body: *const ffi::NewtonBody,
            collision: *const ffi::NewtonCollision,
            collision_id: c_longlong,
            contact: Vec3,
            normal: Vec3,
        }

        impl Eq for Node {}
        impl PartialEq for Node {
            fn eq(&self, other: &Self) -> bool {
                self.intersect.eq(&other.intersect)
            }
        }

        impl PartialOrd for Node {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                self.intersect.partial_cmp(&other.intersect)
            }
        }

        impl Ord for Node {
            fn cmp(&self, other: &Self) -> Ordering {
                const K: f32 = 100000000.0;
                let left = (self.intersect * K) as u32;
                let right = (other.intersect * K) as u32;
                left.cmp(&right)
            }
        }

        struct Udata {
            heap: BinaryHeap<Node>,
            n: usize,
        }

        let mut udata = Udata { heap: BinaryHeap::with_capacity(params), n: params };

        unsafe {
            ffi::NewtonWorldRayCast(newton.as_raw(),
                                    p0.as_ptr(),
                                    p1.as_ptr(),
                                    Some(cfilter),
                                    mem::transmute(&mut udata),
                                    None,
                                    0 as _);
        }

        return udata.heap
                    .iter()
                    .map(|n| RayHit { body: unsafe { Body::from_raw(n.body, false) },
                                      collision: unsafe { Collision::from_raw(n.collision, false) },
                                      position: n.contact,
                                      normal: n.normal,
                                      collision_id: n.collision_id,
                                      intersect_param: n.intersect })
                    .collect();

        unsafe extern "C" fn cfilter(body: *const ffi::NewtonBody,
                                     collision: *const ffi::NewtonCollision,
                                     contact: *const f32,
                                     normal: *const f32,
                                     collision_id: c_longlong,
                                     user_data: *const c_void,
                                     intersect: f32)
                                     -> f32 {
            let mut udata = mem::transmute::<_, &mut Udata>(user_data);

            udata.heap.push(Node { intersect,
                                   body,
                                   collision,
                                   collision_id,
                                   contact: *unsafe { mem::transmute::<_, &Vec3>(contact) },
                                   normal: *unsafe { mem::transmute::<_, &Vec3>(normal) } });

            while udata.heap.len() > udata.n {
                udata.heap.pop();
            }

            udata.heap.peek().map(|h| h.intersect).unwrap_or(1.0)
        }
    }
}
