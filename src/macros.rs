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

        impl<'a> $crate::body::NewtonBody for Body<'a> {
            fn as_raw(&self) -> *const $crate::ffi::NewtonBody {
                match self {
                    $(Body::$enum(body) => body.as_raw()),*
                }
            }
        }

        impl<'a> $crate::handle::IntoHandle for $crate::body::Body<'a> {
            fn into_handle(mut self, newton: &Newton) -> Handle {
                match &mut self {
                    $(Body::$enum(ref mut body) => body.owned = false),*
                }
                newton.storage().move_body(self)
            }
        }

        impl<'a> $crate::handle::AsHandle for $crate::body::Body<'a> {
            fn as_handle(&self, _: &$crate::newton::Newton) -> Handle {
                $crate::handle::Handle::from_ptr(self.as_raw() as _)
            }
        }

        impl<'a> $crate::handle::FromHandle<'a> for $crate::body::Body<'a> {
            fn from_handle(newton: &'a $crate::newton::Newton, handle: $crate::handle::Handle) -> Option<Self> {
                newton.storage().body(handle)
            }

            fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
                newton.storage_mut().take_body(handle)
            }
        }

        impl<'a> $crate::body::Body<'a> {
            pub(crate) unsafe fn from_raw(raw: *const $crate::ffi::NewtonBody, owned: bool) -> Self {
                let body_type = $crate::ffi::NewtonBodyGetType(raw);
                match mem::transmute::<_, Type>(body_type) {
                    Type::Dynamic => $crate::body::Body::Dynamic(DynamicBody::from_raw(raw, owned)),
                    Type::Kinematic => $crate::body::Body::Kinematic(KinematicBody::from_raw(raw, owned)),
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

            pub fn is_dynamic(&self) -> bool {
                match self {
                    Body::Dynamic(_) => true,
                    _ => false,
                }
            }

            pub fn is_kinematic(&self) -> bool {
                match self {
                    Body::Kinematic(_) => true,
                    _ => false,
                }
            }
        }

        $(
            $(#[$($meta)+])*
            pub struct $body<'a> {
                raw: *const $crate::ffi::NewtonBody,
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
                fn as_raw(&self) -> *const $crate::ffi::NewtonBody {
                    self.raw
                }
            }

            impl<'a> Drop for $body<'a> {
                fn drop(&mut self) {
                    if self.owned {
                        unsafe {
                            $crate::ffi::NewtonDestroyBody(self.raw);
                        }
                    }
                }
            }

            impl<'a> IntoHandle for $body<'a> {
                fn into_handle(mut self, newton: &Newton) -> Handle {
                    //if !self.owned { panic!() }
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
                pub unsafe fn from_raw(raw: *const $crate::ffi::NewtonBody, owned: bool) -> Self {
                    Self {
                        raw,
                        owned,
                        _phantom: PhantomData,
                    }
                }

                /// Forgets about this body. It will be freed along the world
                /// If you want to reference the object later, you should call into_handle instead.
                pub fn release(mut self) {
                    self.owned = false;
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

                        let body = $crate::ffi::$ffi(newton, collision, matrix);
                        let userdata = Box::new(UserData { name, ..Default::default() });

                        $crate::ffi::NewtonBodySetDestructorCallback(body, Some(body_destructor));
                        $crate::ffi::NewtonBodySetUserData(body, mem::transmute(userdata));
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
            fn $option:ident, $is:ident
            $( #[ $($meta:meta)+ ] )*
            struct $collision:ident<'a>
            const ffi::$id:ident
            params $param_name:ident $params:tt
        }
        //($enum_var:ident , ffi::$id:ident , $option:ident ) => pub struct $collision:ident<'a>(...);
    )*) => {

    /// Enum grouping all collision types.
    #[derive(Debug, PartialEq)]
    pub enum Collision<'a> {
        $($enum_var($collision<'a>) ,)*
        HeightFieldF32(HeightField<'a, f32>),
        HeightFieldU16(HeightField<'a, u16>),
    }

    /// Collision types.
    #[repr(i32)]
    #[derive(Clone, Copy, Hash, Debug, Eq, PartialEq, PartialOrd, Ord)]
    enum Type {
        $($enum_var = ffi::$id as i32 ,)*
        HeightField = ffi::SERIALIZE_ID_HEIGHTFIELD as i32,
    }

    /// Collision params
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Params<'a> {
        $( $param_name  $params ,)*
        HeightFieldF32(HeightFieldParams<'a, f32>),
        HeightFieldU16(HeightFieldParams<'a, u16>),
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct HeightFieldParams<'a, T: Elevation> {
        pub width: usize,
        pub height: usize,
        pub vertical_elevation: &'a [T],
        pub vertical_scale: f32,
        pub horizontal_scale_x: f32,
        pub horizontal_scale_z: f32,
        pub horizontal_displacement_scale_x: f32,
        pub horizontal_displacement_scale_z: f32,
        pub horizontal_displacement: &'a [i16],
        pub attributes: &'a [i8],
    }

/*
    fn check_owned(coll: &Collision) {
        match coll {
            $(Collision::$enum_var(ref col) => if !col.owned { panic!() }, )*
            Collision::HeightFieldF32(ref col) => if !col.owned { panic!() },
            Collision::HeightFieldU16(ref col) => if !col.owned { panic!() },
        }
    }
*/

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
                $(Collision::$enum_var(ref col) => col.as_handle(newton) ,)*
                Collision::HeightFieldF32(ref col) => col.as_handle(newton),
                Collision::HeightFieldU16(ref col) => col.as_handle(newton),
            }
        }
    }

    impl<'a> Collision<'a> {
        pub unsafe fn from_raw(raw: *const $crate::ffi::NewtonCollision, owned: bool) -> Self {
            let col_type = $crate::ffi::NewtonCollisionGetType(raw);
            match mem::transmute::<_, Type>(col_type) {
                $(
                    Type::$enum_var => Collision::$enum_var($collision::from_raw(raw, owned)),
                )*
                Type::HeightField => unimplemented!(),
                //_ => unimplemented!("Collision type ({}) not implemented", col_type),
            }
        }

        pub fn create_instance(col: &Self) -> Self {
            match col {
                $( Collision::$enum_var(ref col) => Collision::$enum_var($collision::create_instance(&col)), )*
                Collision::HeightFieldF32(ref col) => Collision::HeightFieldF32(HeightField::create_instance(&col)),
                Collision::HeightFieldU16(ref col) => Collision::HeightFieldU16(HeightField::create_instance(&col)),
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

        $(
            pub fn $is(&self) -> bool {
                match self {
                    Collision::$enum_var(_) => true,
                    _ => false,
                }
            }
        )*

        pub fn height_field_f32(self) -> Option<HeightField<'a, f32>> {
                match self {
                    Collision::HeightFieldF32(col) => Some(col),
                    _ => None,
                }
        }

        pub fn height_field_u16(self) -> Option<HeightField<'a, u16>> {
                match self {
                    Collision::HeightFieldU16(col) => Some(col),
                    _ => None,
                }
        }

        pub fn is_height_field_f32(&self) -> bool {
            match self {
                Collision::HeightFieldF32(_) => true,
                _ => false,
            }
        }

        pub fn is_height_field_u16(&self) -> bool {
            match self {
                Collision::HeightFieldU16(_) => true,
                _ => false,
            }
        }
    }

    impl<'a> NewtonCollision for Collision<'a> {
        fn as_raw(&self) -> *const $crate::ffi::NewtonCollision {
            match self {
                $(Collision::$enum_var(col) => col.raw ,)*
                Collision::HeightFieldF32(col) => col.raw ,
                Collision::HeightFieldU16(col) => col.raw ,
            }
        }
    }
    #[derive(Debug, PartialEq)]
    pub struct HeightField<'a, T: Elevation> {
        raw: *const $crate::ffi::NewtonCollision,
        owned: bool,
        _phantom: PhantomData<&'a T>,
    }

    impl<'a> From<HeightField<'a, f32>> for Collision<'a> {
        fn from(col: HeightField<'a, f32>) -> Self {
            Collision::HeightFieldF32 ( col )
        }
    }

    impl<'a> From<HeightField<'a, u16>> for Collision<'a> {
        fn from(col: HeightField<'a, u16>) -> Self {
            Collision::HeightFieldU16 ( col )
        }
    }

    impl<'a, T: Elevation> Drop for HeightField<'a, T> {
        fn drop(&mut self) {
            // if collision owns itself, then free the memory
            if self.owned {
                unsafe {
                    $crate::ffi::NewtonDestroyCollision(self.raw);
                }
            }
        }
    }

    impl<'a, T: Elevation> HeightField<'a, T> {
        pub unsafe fn from_raw(raw: *const $crate::ffi::NewtonCollision, owned: bool) -> Self {
            Self { raw, owned, _phantom: PhantomData }
        }

        pub fn create_instance(col: &Self) -> Self {
            unsafe {
                let instance = $crate::ffi::NewtonCollisionCreateInstance(col.raw);
                Self::from_raw(instance, true)
            }
        }

        /// Forgets about this body. It will be freed along the world
        /// If you want to reference the object later, you should call into_handle instead.
        pub fn release(mut self) {
            self.owned = false;
        }
    }

    impl<'a> IntoHandle for HeightField<'a, u16> {
        fn into_handle(mut self, newton: &Newton) -> Handle {
            //if !self.owned { panic!() }
            self.owned = false;
            newton.storage().move_collision(self.into_collision())
        }
    }

    impl<'a> IntoHandle for HeightField<'a, f32> {
        fn into_handle(mut self, newton: &Newton) -> Handle {
            //if !self.owned { panic!() }
            self.owned = false;
            newton.storage().move_collision(self.into_collision())
        }
    }

    impl<'a> FromHandle<'a> for HeightField<'a, f32> {
        fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self> {
            newton.storage().collision(handle).and_then(|h| h.height_field_f32())
        }

        fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
            newton.storage_mut().take_collision(handle).and_then(|h| h.height_field_f32())
        }
    }

    impl<'a> FromHandle<'a> for HeightField<'a, u16> {
        fn from_handle(newton: &'a Newton, handle: Handle) -> Option<Self> {
            newton.storage().collision(handle).and_then(|h| h.height_field_u16())
        }

        fn from_handle_owned(newton: &'a mut Newton, handle: Handle) -> Option<Self> {
            newton.storage_mut().take_collision(handle).and_then(|h| h.height_field_u16())
        }
    }

    impl<'a, T: Elevation> AsHandle for HeightField<'a, T> {
        fn as_handle(&self, _: &Newton) -> Handle {
            Handle::from_ptr(self.raw as _)
        }
    }

    impl<'a> IntoCollision<'a> for HeightField<'a, f32> {
        fn into_collision(self) -> Collision<'a> {
            Collision::HeightFieldF32(self)
        }
    }

    impl<'a> IntoCollision<'a> for HeightField<'a, u16> {
        fn into_collision(self) -> Collision<'a> {
            Collision::HeightFieldU16(self)
        }
    }

    impl<'a, T: Elevation> NewtonCollision for HeightField<'a, T> {
        fn as_raw(&self) -> *const $crate::ffi::NewtonCollision {
            self.raw
        }
    }

    $(
        $(#[$($meta)+])*
        pub struct $collision<'a> {
            raw: *const $crate::ffi::NewtonCollision,

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
            pub unsafe fn from_raw(raw: *const $crate::ffi::NewtonCollision, owned: bool) -> Self {
                Self { raw, owned, _phantom: PhantomData }
            }

            pub fn create_instance(col: &Self) -> Self {
                unsafe {
                    let instance = $crate::ffi::NewtonCollisionCreateInstance(col.raw);
                    Self::from_raw(instance, true)
                }
            }

            /// Forgets about this body. It will be freed along the world
            /// If you want to reference the object later, you should call into_handle instead.
            pub fn release(mut self) {
                self.owned = false;
            }
        }

        impl<'a> Drop for $collision<'a> {
            fn drop(&mut self) {
                // if collision owns itself, then free the memory
                if self.owned {
                    unsafe {
                        $crate::ffi::NewtonDestroyCollision(self.raw);
                    }
                }
            }
        }

        impl<'a> IntoHandle for $collision<'a> {
            fn into_handle(mut self, newton: &Newton) -> Handle {
                //if !self.owned { panic!() }
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
            fn as_raw(&self) -> *const $crate::ffi::NewtonCollision {
                self.raw
            }
        }
    )*}
}

macro_rules! joints {
    ($(
        $( #[ $($meta:meta)+ ] )*
        struct $joint:ident
        fn $option:ident, $is:ident
    )*) => {
        $(
            $( #[ $($meta)+ ] )*
            pub struct $joint<'a> {
                raw: *const ffi::NewtonJoint,
                owned: bool,
                _phantom: PhantomData<&'a ()>,
            }

            impl<'a> NewtonJoint for $joint<'a> {
                fn as_raw(&self) -> *const ffi::NewtonJoint {
                    self.raw
                }
            }

            impl<'a> Drop for $joint<'a> {
                fn drop(&mut self) {
                    if self.owned {
                        let world = unsafe {
                            let udata = ffi::NewtonJointGetUserData(self.raw);
                            let udata: &Box<UserData> = mem::transmute(&udata);
                            udata.world
                        };
                        unsafe {
                            ffi::NewtonDestroyJoint(world, self.raw);
                        }
                    }
                }
            }

            impl<'a> $joint<'a> {
                pub unsafe fn from_raw(raw: *const ffi::NewtonJoint, owned: bool) -> Self {
                    Self {
                        raw,
                        owned,
                        _phantom: PhantomData,
                    }
                }

                /// Forgets about this body. It will be freed along the world
                /// If you want to reference the object later, you should call into_handle instead.
                pub fn release(mut self) {
                    self.owned = false;
                }
            }

            impl<'a> AsHandle for $joint<'a> {
                fn as_handle(&self, _: &Newton) -> Handle {
                    Handle::from_ptr(self.raw as _)
                }
            }
            impl<'a> IntoHandle for $joint<'a> {
                fn into_handle(mut self, newton: &Newton) -> Handle {
                    self.owned = false;
                    //newton.storage().move_constraint(newton);
                    Handle::from_ptr(self.raw as _)
                }
            }
        )*

        /// Enum grouping all joint types.
        #[derive(Debug)]
        pub enum Joint<'a> {
            $( $joint($joint<'a>) ),*
        }

        #[derive(Debug)]
        enum Type {
            $( $joint ),*
        }

        impl<'a> Joint<'a> {
            pub unsafe fn from_raw(raw: *const ffi::NewtonJoint, owned: bool) -> Self {
                let udata = ffi::NewtonJointGetUserData(raw);
                let udata: &Box<UserData> = mem::transmute(&udata);

                match udata.joint_type {
                    $(
                        Type::$joint => Joint::$joint($joint ::from_raw(raw, owned)),
                    )*
                }
            }

            $(
                pub fn $option(self) -> Option<$joint<'a>> {
                    match self {
                        Joint::$joint(con) => Some(con),
                        _ => None,
                    }
                }

                pub fn $is(&self) -> bool {
                    match self {
                        Joint::$joint(_) => true,
                        _ => false,
                    }
                }
            )*
        }

        impl<'a> NewtonJoint for Joint<'a> {
            fn as_raw(&self) -> *const ffi::NewtonJoint {
                match self {
                    $(Joint::$joint(ref joint) => joint.as_raw() ),*
                }
            }
        }
    }
}

