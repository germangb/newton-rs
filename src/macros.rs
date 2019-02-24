macro_rules! bodies {
    ($(
        $(#[$($meta:meta)+])*
        ($enum:ident, $ffi:ident, $option:ident) => pub struct $body:ident<'a>(...);
    )*) => {

        /// Enum grouping all body types.
        #[derive(Debug, Eq, PartialEq)]
        pub enum Body<'a> {
            $( $enum($body<'a>) ),*
        }

        impl<'a> NewtonBody for Body<'a> {
            fn as_raw(&self) -> *const ffi::NewtonBody {
                match self {
                    $(Body::$enum(body) => body.as_raw()),*
                }
            }
        }

        impl<'a> IntoHandle for Body<'a> {
            fn into_handle(mut self, newton: &Newton) -> Handle {
                match &mut self {
                    $(Body::$enum(ref mut body) => if !body.owned { panic!() } else { body.owned = false; }),*
                }
                newton.storage().move_body(self)
            }
        }

        impl<'a> AsHandle for Body<'a> {
            fn as_handle(&self, _: &Newton) -> Handle {
                Handle::from_ptr(self.as_raw() as _)
            }
        }

        impl<'a> FromHandle<'a> for Body<'a> {
            fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self> {
                newton.storage().body(handle)
            }

            fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
                newton.storage_mut().take_body(handle)
            }
        }

        impl<'a> Body<'a> {
            pub(crate) unsafe fn from_raw(raw: *const ffi::NewtonBody, owned: bool) -> Self {
                let body_type = ffi::NewtonBodyGetType(raw);
                match body_type as _ {
                    ffi::NEWTON_DYNAMIC_BODY => Body::Dynamic(DynamicBody::from_raw(raw, owned)),
                    ffi::NEWTON_KINEMATIC_BODY => Body::Kinematic(KinematicBody::from_raw(raw, owned)),
                    _ => unreachable!("Unexpected body type ({})", body_type),
                }
            }

            pub fn dynamic(self) -> Option<DynamicBody<'a>> {
                match self {
                    Body::Dynamic(body) => Some(body),
                    _ => None,
                }
            }

            pub fn kinematic(self) -> Option<KinematicBody<'a>> {
                match self {
                    Body::Kinematic(body) => Some(body),
                    _ => None,
                }
            }
        }

        $(
            $(#[$($meta)+])*
            pub struct $body<'a> {
                raw: *const ffi::NewtonBody,
                _phantom: PhantomData<&'a ()>,
                // Bodies from iterators or callbacks generally won't be owned.
                // When they are, the memory is freed when the object is dropped.
                pub(crate) owned: bool,
            }

            impl<'a> From<$body<'a>> for Body<'a> {
                fn from(body: $body<'a>) -> Self {
                    Body::$enum ( body )
                }
            }

            impl<'a> FromHandle<'a> for $body<'a> {
                fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self> {
                    newton.storage().body(handle).and_then(|h| h.$option())
                }

                fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
                    newton.storage_mut().take_body(handle).and_then(|h| h.$option())
                }
            }
/*
            unsafe impl<'a> Send for $body<'a> {}
            unsafe impl<'a> Sync for $body<'a> {}
*/
            impl<'a> IntoBody<'a> for $body<'a> {
                fn into_body(self) -> Body<'a> {
                    Body::$enum(self)
                }
            }

            impl<'a> NewtonBody for $body<'a> {
                fn as_raw(&self) -> *const ffi::NewtonBody {
                    self.raw
                }
            }

            impl<'a> Drop for $body<'a> {
                fn drop(&mut self) {
                    if self.owned {
                        unsafe {
                            ffi::NewtonDestroyBody(self.raw);
                        }
                    }
                }
            }

            impl<'a> IntoHandle for $body<'a> {
                fn into_handle(mut self, newton: &Newton) -> Handle {
                    if !self.owned { panic!() }
                    self.owned = false;
                    newton.storage().move_body(self.into_body())
                }
            }

            impl<'a> AsHandle for $body<'a> {
                fn as_handle(&self, _: &Newton) -> Handle {
                    Handle::from_ptr(self.raw as _)
                }
            }

            impl<'a> $body<'a> {
                pub unsafe fn from_raw(raw: *const ffi::NewtonBody, owned: bool) -> Self {
                    Self {
                        raw,
                        owned,
                        _phantom: PhantomData,
                    }
                }

                pub fn create<C>(newton: &'a Newton,
                                 collision: &C,
                                 matrix: Mat4,
                                 name: Option<&'static str>) -> Self
                where
                    C: NewtonCollision,
                {
                    unsafe {
                        let newton = newton.as_raw();
                        let matrix = matrix[0].as_ptr();
                        let collision = collision.as_raw();

                        let body = ffi::$ffi(newton, collision, matrix);
                        let userdata = Box::new(UserData { name, ..Default::default() });

                        ffi::NewtonBodySetDestructorCallback(body, Some(body_destructor));
                        ffi::NewtonBodySetUserData(body, mem::transmute(userdata));
                        Self { raw: body, owned: true, _phantom: PhantomData }
                    }
                }
            }
        )*
    }
}

macro_rules! collision {
    ($(
        {
            enum $enum_var:ident
            fn $option:ident
            $( #[ $($meta:meta)+ ] )*
            struct $collision:ident
            const ffi::$id:ident
            params $param_name:ident $params:tt
        }
        //($enum_var:ident , ffi::$id:ident , $option:ident ) => pub struct $collision:ident<'a>(...);
    )*) => {

    /// Enum grouping all collision types.
    #[derive(Debug, Eq, PartialEq)]
    pub enum Collision<'a> {
        $($enum_var($collision<'a>)),*
    }

    /// Collision types.
    #[repr(i32)]
    #[derive(Clone, Copy, Hash, Debug, Eq, PartialEq, PartialOrd, Ord)]
    pub enum Type {
        $($enum_var = ffi::$id as i32),*
    }

    /// Collision params
    #[derive(Clone, Copy, Debug)]
    pub enum Params<'a, T> {
        $( $param_name  $params),*
    }

    fn check_owned(coll: &Collision) {
        match coll {
            $(Collision::$enum_var(ref col) => if !col.owned { panic!() }),*
        }
    }

    impl<'a> FromHandle<'a> for Collision<'a> {
        fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self> {
            newton.storage().collision(handle)
        }

        fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
            newton.storage_mut().take_collision(handle)
        }
    }

    impl<'a> AsHandle for Collision<'a> {
        fn as_handle(&self, newton: &Newton) -> Handle {
            match self {
                $(Collision::$enum_var(ref col) => col.as_handle(newton)),*
            }
        }
    }

    impl<'a> Collision<'a> {
        pub unsafe fn from_raw(raw: *const ffi::NewtonCollision, owned: bool) -> Self {
            let col_type = ffi::NewtonCollisionGetType(raw);
            match col_type as _ {
                $(
                    ffi::$id => Collision::$enum_var($collision::from_raw(raw, owned)),
                )*
                _ => unimplemented!("Collision type ({}) not implemented", col_type),
            }
        }

        pub fn create_instance(col: &Self) -> Self {
            match col {
                $( Collision::$enum_var(ref col) => Collision::$enum_var($collision::create_instance(&col)) ),*
            }
        }

        $(
            pub fn $option(self) -> Option<$collision<'a>> {
                match self {
                    Collision::$enum_var(col) => Some(col),
                    _ => None,
                }
            }
        )*
    }

    impl<'a> NewtonCollision for Collision<'a> {
        fn as_raw(&self) -> *const ffi::NewtonCollision {
            match self {
                $(Collision::$enum_var(col) => col.raw ),*
            }
        }
    }

    $(
        $(#[$($meta)+])*
        pub struct $collision<'a> {
            raw: *const ffi::NewtonCollision,

            // If set to true, the memory is freed when the instance is dropped.
            owned: bool,
            _phantom: PhantomData<&'a ()>,
        }
/*
        unsafe impl<'a> Send for $collision<'a> {}
        unsafe impl<'a> Sync for $collision<'a> {}
*/
        impl<'a> From<$collision<'a>> for Collision<'a> {
            fn from(col: $collision<'a>) -> Self {
                Collision::$enum_var ( col )
            }
        }

        impl<'a> FromHandle<'a> for $collision<'a> {
            fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self> {
                newton.storage().collision(handle).and_then(|h| h.$option())
            }

            fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
                newton.storage_mut().take_collision(handle).and_then(|h| h.$option())
            }
        }

        impl<'a> $collision<'a> {
            pub unsafe fn from_raw(raw: *const ffi::NewtonCollision, owned: bool) -> Self {
                $collision { raw, owned, _phantom: PhantomData }
            }

            pub fn create_instance(col: &Self) -> Self {
                unsafe {
                    let instance = ffi::NewtonCollisionCreateInstance(col.raw);
                    Self::from_raw(instance, true)
                }
            }
        }

        impl<'a> Drop for $collision<'a> {
            fn drop(&mut self) {
                // if collision owns itself, then free the memory
                if self.owned {
                    unsafe {
                        ffi::NewtonDestroyCollision(self.raw);
                    }
                }
            }
        }

        impl<'a> IntoHandle for $collision<'a> {
            fn into_handle(mut self, newton: &Newton) -> Handle {
                if !self.owned { panic!() }
                self.owned = false;
                newton.storage().move_collision(self.into_collision())
            }
        }

        impl<'a> AsHandle for $collision<'a> {
            fn as_handle(&self, _: &Newton) -> Handle {
                Handle::from_ptr(self.raw as _)
            }
        }

        impl<'a> IntoCollision<'a> for $collision<'a> {
            fn into_collision(self) -> Collision<'a> {
                Collision::$enum_var(self)
            }
        }

        impl<'a> NewtonCollision for $collision<'a> {
            fn as_raw(&self) -> *const ffi::NewtonCollision {
                self.raw
            }
        }
    )*}
}
