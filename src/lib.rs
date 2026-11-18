//! VigNav: deterministic navigation for RTS games.
//!
//! Everything a lockstep simulation reads is exact integer arithmetic on
//! millimetre coordinates, so every peer computes the same bits. The crate
//! has no engine dependency.
//!
//! A [`NavMesh`] is one walkable layer of convex polygons. Build it from
//! polygons, then ask it where a point is and how to get from one point to
//! another:
//!
//! ```
//! use vignav::{NavMesh, Point};
//!
//! // Two 10 m squares side by side, sharing the edge at x = 10 m.
//! let vertices = [
//!     Point::new(0, 0),
//!     Point::new(10_000, 0),
//!     Point::new(20_000, 0),
//!     Point::new(0, 10_000),
//!     Point::new(10_000, 10_000),
//!     Point::new(20_000, 10_000),
//! ];
//! let mesh = NavMesh::from_polygons(&vertices, &[[0, 1, 4, 3], [1, 2, 5, 4]])?;
//!
//! // Endpoints up to half a metre off the mesh are moved onto it.
//! let path = mesh.find_path(Point::new(1_000, 5_000), Point::new(19_000, 5_000), 500)?;
//! assert_eq!(path.points, [Point::new(1_000, 5_000), Point::new(19_000, 5_000)]);
//! assert_eq!(path.corridor.len(), 2);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! The README carries the roadmap.

pub mod geom;
pub mod mesh;
pub mod path;

pub use geom::{Coord, Point};
pub use mesh::{Location, MeshError, NavMesh, PolyId};
pub use path::{Path, PathError};
