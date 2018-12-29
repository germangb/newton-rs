use std::marker::PhantomData;

#[derive(Debug)]
pub struct Contacts<'a, T>(PhantomData<&'a T>);

#[derive(Debug)]
pub struct Joints<'a, T>(PhantomData<&'a T>);
