use super::newton::Newton;

#[cfg(not(feature = "ci"))]
mod renderer;
#[cfg(not(feature = "ci"))]
mod runner;

pub trait Testbed {
    fn reset(newton: &mut Newton) -> Self;
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
