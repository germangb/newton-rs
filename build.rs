extern crate bindgen;
extern crate cmake;

use bindgen::Builder;
use cmake::Config;

use std::{env, path::PathBuf};

mod config {
    pub static LIB_PATH: &[&str] = &["lib64/", "lib/"];
    pub static LIB: &str = "Newton_d";
    pub static HEADER_PATH: &str = "include/";
    pub static HEADER: &str = "Newton.h";
}

fn main() {
    let dst = Config::new("newton-dynamics")
        .define("NEWTON_DEMOS_SANDBOX", "OFF")
        .build();

    for lib_path in config::LIB_PATH.iter() {
        println!(
            "cargo:rustc-link-search={build_path}/{lib_path}",
            build_path = dst.display(),
            lib_path = lib_path,
        );
    }
    println!("cargo:rustc-link-lib={}", config::LIB);

    let mut header_file = PathBuf::from(env::var("OUT_DIR").unwrap());
    header_file.push(config::HEADER_PATH);
    header_file.push(config::HEADER);

    eprintln!("Attempting to generate bindings");
    eprintln!("OUT_DIR={dir}", dir = env::var("OUT_DIR").unwrap());
    eprintln!("HEADER={path}", path = header_file.to_str().unwrap());

    let bindings_builder = Builder::default()
        .header(header_file.to_str().unwrap())
        .generate_comments(true)
        .layout_tests(true)
        // derives
        .derive_copy(true)
        .derive_debug(true)
        .derive_default(true)
        .rustfmt_bindings(true)
        .generate()
        .expect("Wrror while generating Newton bindings. Check the build log for more");

    // write generated bindings to the build script output directory
    let mut output_bindings = PathBuf::from(env::var("OUT_DIR").unwrap());
    output_bindings.push("bindgen.rs");

    bindings_builder
        .write_to_file(output_bindings.to_str().unwrap())
        .expect(
            "Bindings were generated but there was an error while writing them to the output file",
        );
}
