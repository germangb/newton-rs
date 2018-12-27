extern crate bindgen;
extern crate cmake;

use bindgen::Builder;
use cmake::Config;

use std::{env, path::PathBuf};

fn main() {
    let dst = Config::new("newton-dynamics")
        .define("NEWTON_DEMOS_SANDBOX", "OFF")
        .build();

    println!("cargo:rustc-link-search={}/lib64/", dst.display());
    println!("cargo:rustc-link-search={}/lib/", dst.display());
    println!("cargo:rustc-link-lib=Newton_d");

    let mut header = PathBuf::from(env::var("OUT_DIR").unwrap());
    header.push("include/");
    header.push("Newton.h");

    let bindings_builder = Builder::default()
        .header(header.to_str().unwrap())
        .generate_comments(true)
        .layout_tests(true)
        // derives
        .derive_copy(true)
        .derive_debug(true)
        .derive_default(true)
        .rustfmt_bindings(true)
        .generate()
        .expect("bindgen error");

    let mut output = PathBuf::from(env::var("OUT_DIR").unwrap());
    output.push("bindgen.rs");

    bindings_builder
        .write_to_file(output.to_str().unwrap())
        .expect("Error writting bindings to disk");
}
