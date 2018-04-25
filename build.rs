extern crate cmake;
extern crate bindgen;

use cmake::Config;
use std::env;
use std::path::PathBuf;

use bindgen::Builder;

static NATIVE_LIB_NAME: &str = "Newton_d";
static HEADER_FILE_PATH: &str = "include/Newton.h";

static BINDINGS_USAGE: &str = r#"
pub mod bindgen {
    #![allow(non_snake_case)]
    #![allow(non_camel_case_types)]
    #![allow(non_upper_case_globals)]

    include!(concat!(env!("OUT_DIR"), "/bindgen.rs"));
}"#;

fn main() {
    let dst = Config::new("newton-dynamics")
        .define("NEWTON_DEMOS_SANDBOX", "OFF").build();

    println!("cargo:rustc-link-search={path}/lib64/", path=dst.display());
    println!("cargo:rustc-link-lib={path}", path=NATIVE_LIB_NAME);

    let mut header_file = PathBuf::from(env::var("OUT_DIR").unwrap());
    header_file.push(HEADER_FILE_PATH);

    eprintln!("Attempting to generate bindings");
    eprintln!("OUT_DIR={dir}", dir=env::var("OUT_DIR").unwrap());
    eprintln!("HEADER={path}", path=header_file.to_str().unwrap());

    let bindings_builder = Builder::default()
        .header(header_file.to_str().unwrap())
        .generate_comments(true)
        .layout_tests(true)

        .header_contents("bool", "#define bool int")

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

    bindings_builder.write_to_file(output_bindings.to_str().unwrap())
        .expect("Bindings were generated but there was an error while writing them to the output file");

    // log useful information
    eprintln!("Newton bindings generated Successfully!");
    eprintln!("---------------------------------------");

    eprintln!("In order to include them, use the following macro:");
    eprintln!("{}", BINDINGS_USAGE);
}
