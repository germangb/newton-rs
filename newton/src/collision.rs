use crate::ffi;
use crate::NewtonData;
use crate::world::{NewtonWorld, WorldRef};
use crate::body::{NewtonBodyInner, NewtonBodyBuilder};

use std::rc::Rc;
use std::mem;
use std::ptr;
use std::marker::PhantomData;

macro_rules! primitives {
    ($(
        #[ $( $meta:meta )+ ]
        pub struct $struct:ident<V: NewtonData> => $ffi:ident ( $($param:ident),+ );
    )+) => {
        $( #[ $( $meta )+ ]
        pub struct $struct<V> {
            pub(self) world: Rc<WorldRef>,
            collision: *mut ffi::NewtonCollision,
            _ph: PhantomData<V>,
        }
        impl<V> $struct<V>
        where
            V: NewtonData,
        {
            pub fn new(world: &NewtonWorld<V>, $( $param : f32 ),+ ) -> $struct<V> {
                unsafe {
                    $struct {
                        world: Rc::clone(&world.world),
                        collision: ffi::$ffi(world.world.0, $( $param, )+ 0, ptr::null()),
                        _ph: PhantomData,
                    }
                }
            }

            pub fn body(&self, matrix: V::Matrix4) -> NewtonBodyBuilder<V> {
                NewtonBodyBuilder::new(self.world.clone(), self.collision, matrix)
            }
        }
        impl<V> Drop for $struct<V> {
            fn drop(&mut self) {
                unsafe { ffi::NewtonDestroyCollision(self.collision) }
            }
        })+
    }
}

primitives! {
    #[derive(Debug)]
    pub struct NewtonCuboid<V: NewtonData> => NewtonCreateBox(dx, dy, dz);

    #[derive(Debug)]
    pub struct NewtonSphere<V: NewtonData> => NewtonCreateSphere(radius);

    #[derive(Debug)]
    pub struct NewtonCapsule<V: NewtonData> => NewtonCreateCapsule(radius0, radius1, height);

    #[derive(Debug)]
    pub struct NewtonCone<V: NewtonData> => NewtonCreateCone(radius, height);

    #[derive(Debug)]
    pub struct NewtonCylinder<V: NewtonData> => NewtonCreateCylinder(radius0, radius1, height);
}

