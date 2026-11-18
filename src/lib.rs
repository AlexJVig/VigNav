//! VigNav: deterministic navigation for RTS games.
//!
//! Everything a lockstep simulation reads is exact integer arithmetic on
//! millimetre coordinates, so every peer computes the same bits. The crate
//! has no engine dependency.
//!
//! Only the geometry foundation exists so far; the README carries the
//! roadmap.

pub mod geom;
