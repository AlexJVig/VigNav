//! VigNav: deterministic navigation for RTS games.
//!
//! Everything a lockstep simulation reads is exact integer arithmetic on
//! millimetre coordinates, so every peer computes the same bits. The crate
//! has no engine dependency.
//!
//! A [`NavMesh`] is one walkable layer of convex polygons. Build it from
//! polygons, then ask it where a point is and how to get from one point to
//! another. Obstacles can be laid over it and lifted again at any time:
//!
//! ```
//! use vignav::geom::grow_convex;
//! use vignav::{NavMesh, ObstacleKind, Point};
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
//! let mut mesh = NavMesh::from_polygons(&vertices, &[[0, 1, 4, 3], [1, 2, 5, 4]])?;
//!
//! // Endpoints up to half a metre off the mesh are moved onto it.
//! let (start, goal) = (Point::new(1_000, 5_000), Point::new(19_000, 5_000));
//! let path = mesh.find_path(start, goal, 500)?;
//! assert_eq!(path.points, [start, goal]);
//! assert_eq!(path.corridor.len(), 2);
//!
//! // A building in the way, grown by the half metre these units need round them.
//! let building = [
//!     Point::new(9_000, 3_000),
//!     Point::new(11_000, 3_000),
//!     Point::new(11_000, 7_000),
//!     Point::new(9_000, 7_000),
//! ];
//! let id = mesh.add_obstacle(&grow_convex(&building, 500), ObstacleKind::Blocked)?;
//! assert!(mesh.find_path(start, goal, 500)?.points.len() > 2);
//!
//! // Gone again, and the route is straight again.
//! mesh.remove_obstacle(id);
//! assert_eq!(mesh.find_path(start, goal, 500)?.points, [start, goal]);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! The README carries the limits and the roadmap.

pub mod geom;
pub mod live;
pub mod mesh;
pub mod path;

pub use geom::{Coord, Point};
pub use live::{ObstacleEdit, ObstacleId, ObstacleKind};
pub use mesh::{Location, MeshError, NavMesh, PolyId};
pub use path::{Path, PathError};
