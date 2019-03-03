***Work in progress***<br>
***Things claimed in this README may not be implemented yet.***

# newton-rs

[![Build Status](https://img.shields.io/travis/germangb/newton-rs/master.svg?style=flat-square)](https://travis-ci.org/germangb/newton-rs)
[![Master docs](https://img.shields.io/badge/docs-master-blue.svg?style=flat-square)](https://germangb.github.io/newton-rs/)

[Newton-Dynamics][repo] safe wrapper for [**Rust**][rustlang].

* **Newton-Dynamics version: 3.13a**

[repo]: https://github.com/MADEAPPS/newton-dynamics
[rustlang]: https://www.rust-lang.org/

## Building

1. Install build dependencies

* `rust` (install using [rustup](https://www.rust-lang.org/tools/install))
* `cmake`
* `libclang-dev`
* `libsdl2-dev` optional (testbed feature)

```bash
apt install cmake
apt install libclang-dev
apt install libsdl2-dev # optional
```

2. Add dependency to your `Cargo.toml`

```toml
[dependencies]
newton = { git = "https://github.com/germangb/newton-rs.git" }
```

## Implemented things

The wrapper gives coverage to a subset of the C API (see [#1][issue]).

* Most Body functions
* Most Collision functions (except mesh)
* Ray casting & convex casting
* Constraints (joints)
* Meshes (More or less)
* Main callbacks

[issue]: https://github.com/germangb/newton-rs/issues/1

## Unimplemented things

* Materials
* Inverse kinematics

## Testbed

The newton testbed is a framework to run and inspect physics simulation.

Right now it is available under a feature flag, but eventually it will be moved into its own crate.

```bash
# Clone repository with submodules.
git clone --recursive https://github.com/germangb/newton-rs.git

# Change into the directory.
cd newton-rs

# Run one of the examples under the "examples/<name>.rs" directory.
# (the "testbed" feature has to be enabled).
cargo run --features testbed --example "<name>"
```
