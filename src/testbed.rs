//!
//! * **This module is included iif the *testbed* feature is enabled.**
use std::sync::Arc;

use crate::newton::Newton;

#[cfg(not(feature = "ci"))]
mod renderer;
#[cfg(not(feature = "ci"))]
mod runner;

pub trait Testbed {
    fn newton() -> Newton {
        Newton::create()
    }

    fn reset(newton: &Newton) -> Self;
}

pub fn run<T: Testbed>(title: Option<&str>) {
    #[cfg(not(feature = "ci"))]
    match self::runner::Runner::<T>::run(title) {
        Ok(_) => {}
        Err(e) => {
            eprintln!("ERROR: {}\n{:#?}", e, e);
            std::process::exit(1)
        }
    }
    #[cfg(feature = "ci")]
    panic!("Newton Testbed can't run on a CI environment");
}
