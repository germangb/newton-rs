use ffi;

use super::NewtonBody;
use super::NewtonWorld;

use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;

// https://doc.rust-lang.org/std/ptr/struct.NonNull.html
macro_rules! body_iterator {
    (
        $(#[$($meta:meta)+])*
        $struct_name:ident < 'a, B, C > ,
        $item:ty,
        $new:ty
    ) => {
        $(#[$($meta)+])*
        pub struct $struct_name<'a, B, C> {
            world: *const ffi::NewtonWorld,
            next: *const ffi::NewtonBody,
            // The NewtonBody reference that gets returned by the iterator.
            // This data has to be stored in the heap since we can't set the lifetime of the next() method...
            body: *mut NewtonBody<B, C>,
            _phantom: PhantomData<&'a ()>,
        }

        impl<'a, B, C> $struct_name<'a, B, C> {
            pub(crate) fn new(world: $new) -> $struct_name<B, C> {
                unsafe {
                    let world = world.as_raw();
                    let first = ffi::NewtonWorldGetFirstBody(world);
                    let body = Box::new(NewtonBody::new_not_owned(first));

                    $struct_name {
                        world,
                        next: first,
                        body: Box::into_raw(body),
                        _phantom: PhantomData,
                    }
                }
            }
        }

        impl<'a, B: 'a, C: 'a> Iterator for $struct_name<'a, B, C> {
            type Item = $item;

            fn next(&mut self) -> Option<Self::Item> {
                if self.next.is_null() {
                    None
                } else {
                    unsafe {
                        let mut boxed = Box::from_raw(self.body);
                        boxed.body = self.next as _;
                        self.next = ffi::NewtonWorldGetNextBody(self.world, boxed.body);
                        Some(mem::transmute(Box::into_raw(boxed)))
                    }
                }
            }
        }
        impl<'a, B, C> Drop for $struct_name<'a, B, C> {
            fn drop(&mut self) {
                unsafe {
                    let _: Box<NewtonBody<B, C>> = Box::from_raw(self.body);
                }
            }
        }
    };
}

body_iterator! {
/// Iterator that yields NewtonBodies
#[derive(Debug)]
NewtonBodies<'a, B, C>, &'a NewtonBody<B, C>, &NewtonWorld<B, C>
}

body_iterator! {
/// Iterator that yields mutable NewtonBodies
#[derive(Debug)]
NewtonBodiesMut<'a, B, C>, &'a mut NewtonBody<B, C>, &mut NewtonWorld<B, C>
}
