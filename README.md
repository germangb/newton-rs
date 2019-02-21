***Work in progress***<br>
***Things claimed in this README may not be implemented yet.***

# newton-rs

[![Build Status](https://img.shields.io/travis/germangb/newton-rs/master.svg?style=flat-square)](https://travis-ci.org/germangb/newton-rs)
[![Master docs](https://img.shields.io/badge/docs-master-blue.svg?style=flat-square)](https://germangb.github.io/newton-rs/)

Newton-Dynamics safe wrapper for [**Rust**][rustlang].

## Build dependencies

* `cmake`
* `libclang`

```bash
sudo apt install cmake
sudo apt install libclang-dev
```

## Implemented things

The wrapper gives coverage to a subset of the C API (see [#1][issue]).

## Unimplemented things

* Documentation
* Joints
* Mesh collisions
* Most callbacks

## Testbed

The newton testbed is a framework to run and inspect physics simulation.

Right now it is available under a feature flag, but eventually it will be moved into its own crate.

```bash
cargo run --features testbed --example jenga
```

![][testbed]

[testbed]: assets/testbed.png
[rustlang]: https://www.rust-lang.org/
[issue]: https://github.com/germangb/newton-rs/issues/1