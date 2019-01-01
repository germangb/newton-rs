use std::marker::PhantomData;

#[derive(Debug)]
pub struct Contacts<'a>(PhantomData<&'a ()>);

#[derive(Debug)]
pub struct Joints<'a>(PhantomData<&'a ()>);
