macro_rules! collision_enum {
    (
        $(#[$($meta:meta)+])*
        pub enum $typee:ident<$gen:ident> {
            $(
                $(#[$($metai:meta)+])*
                $enumm:ident ( $structt:ty  ) ,
            )+
        }
    ) => {
        $(#[$($meta)+])*
        pub enum $typee <$gen> {
            $(
                $(#[$($metai)+])*
                $enumm ( $structt ) ,
            )+
        }

        impl<$gen: $crate::NewtonApp> $typee<$gen> {
            pub fn as_raw(&self) -> *const ffi::NewtonCollision {
                match &self {
                    $(
                        &$crate::collision::Collision::$enumm (ref c) => c.raw,
                    )+
                }
            }
            pub(crate) fn pointer(&self) -> &Rc<$crate::pointer::NewtonCollisionPtr<$gen>> {
                match &self {
                    $(
                        &$crate::collision::Collision::$enumm (ref c) => &c.collision,
                    )+
                }
            }
        }

        $(impl<C> $crate::collision::IntoCollision<C> for $structt {
            fn into_collision(self) -> $crate::collision::Collision<C> {
                $crate::collision::Collision::$enumm(self)
            }
        })+

        impl<C> crate::collision::IntoCollision<C> for $crate::collision::Collision<C> {
            fn into_collision(self) -> $crate::collision::Collision<C> {
                self
            }
        }
    }
}

macro_rules! collisions {
    ($(
        $(#[$($meta:meta)+])*
        pub struct $struct_ident:ident<C>;
    )+) => {$(
        $(#[$($meta)+])*
        pub struct $struct_ident<C> {
            pub(crate) collision: ::std::rc::Rc<$crate::pointer::NewtonCollisionPtr<C>>,
            pub(crate) raw: *mut ffi::NewtonCollision,
        }
    )+}
}

macro_rules! collision_methods {
    (fn $method:ident ( $($param:ident),+ ) -> ffi::$ffi:ident) => {
        pub fn $method(
            world: &$crate::world::World<C>,
            $($param: f32,)+
            shape_id: $crate::collision::ShapeId,
            offset: Option<C::Matrix>,
        ) -> Self {
            unsafe {
                // TODO check type of borrow
                let world_mut = world.world.borrow_mut();

                let raw = ffi::$ffi(
                    world_mut.0,
                    $($param ,)+
                    shape_id,
                    ::std::mem::transmute(offset.as_ref()),
                );
                let world = world.world.clone();
                let collision = ::std::rc::Rc::new(NewtonCollisionPtr(raw, world));
                Self { collision, raw }
            }
        }
        pub fn as_raw(&self) -> *const ffi::NewtonCollision {
            self.raw
        }
    };
    (fn scale) => {
        pub fn scale(&self) -> (f32, f32, f32) {
            unsafe {
                let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
                ffi::NewtonCollisionGetScale(self.raw, &mut x, &mut y, &mut z);
                (x, y, z)
            }
        }
        pub fn set_scale(&self, (x, y, z): (f32, f32, f32)) {
            unsafe { ffi::NewtonCollisionSetScale(self.raw, x, y, z) };
        }
    };
    (fn offset, $type:ty) => {
        pub fn offset(&self) -> $type {
            unsafe {
                let mut matrix: $type = ::std::mem::zeroed();
                ffi::NewtonCollisionGetMatrix(self.raw, ::std::mem::transmute(&mut matrix));
                matrix
            }
        }
        pub fn set_offset(&self, matrix: $type) {
            unsafe { ffi::NewtonCollisionSetMatrix(self.raw, ::std::mem::transmute(&matrix)) };
        }
    };
}
