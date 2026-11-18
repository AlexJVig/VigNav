//! The VigNav test suite: every public flow, end to end.
//!
//! There is one module per flow. To cover something new, add a test to the
//! module it belongs to, or add a module here.
//!
//! `support` holds what the tests share: the L-shaped mesh, maps drawn as
//! text ([`support::Map::drawn`]), random maps, and `check_path`, which
//! every route should be put through whatever else a test asserts.

mod build;
mod determinism;
mod lookup;
mod paths;
mod random;
mod support;
