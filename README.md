# VigNav

Deterministic navigation for RTS games, in Rust: navmesh, pathfinding and
steering that give the same result on every machine in a lockstep match.

**Status: milestone 1.** Loading a navmesh, looking points up on it and finding
paths across it work and are tested. Generation, dynamic obstacles and steering
are on the roadmap below.

VigNav is an independent project built from published algorithms. Its feature
set is modelled on what commercial navigation middleware such as NavPower
offers; it is not affiliated with NavPower or BabelFlux and shares no code or
file formats with it.

## What it does today

- `NavMesh::from_polygons` builds a mesh from convex polygons and links them
  wherever two share an edge.
- `NavMesh::from_linked_polygons` builds a mesh whose links are already known,
  such as one exported from another tool.
- `locate` finds the polygon under a point. `nearest` finds the closest point
  of the mesh within a given distance.
- `find_path` and `find_path_between` return the corner points of a route and
  the polygons it crosses: A* over polygons, then the funnel algorithm.
- `island` tells in constant time whether two polygons connect at all.

```rust
use vignav::{NavMesh, Point};

// Two 10 m squares side by side, sharing the edge at x = 10 m.
let vertices = [
    Point::new(0, 0),
    Point::new(10_000, 0),
    Point::new(20_000, 0),
    Point::new(0, 10_000),
    Point::new(10_000, 10_000),
    Point::new(20_000, 10_000),
];
let mesh = NavMesh::from_polygons(&vertices, &[[0, 1, 4, 3], [1, 2, 5, 4]])?;

// Endpoints up to half a metre off the mesh are moved onto it.
let path = mesh.find_path(Point::new(1_000, 5_000), Point::new(19_000, 5_000), 500)?;
assert_eq!(path.points, [Point::new(1_000, 5_000), Point::new(19_000, 5_000)]);
```

## Feeding it a mesh you already have

One `NavMesh` is one layer: the ground as one class of unit sees it. A game
with several unit classes keeps one mesh per class.

1. Convert coordinates to integer millimetres on the ground plane.
2. List each polygon's corners counter-clockwise, as indices into the vertex
   list.
3. If the data records which polygon lies across each edge, pass that to
   `from_linked_polygons`. Edge `i` of a polygon runs from its corner `i` to
   its corner `i + 1`. Otherwise use `from_polygons`.

A corner may dent inwards by up to 2 mm (`CONVEX_SLACK`), which absorbs the
rounding of a mesh that was built in floating point. Anything else that is
wrong is reported as a `MeshError` naming the polygon and the corner or edge.

## Limits of this milestone

- **Unit size.** Routes touch the corners of the mesh, so the mesh must already
  be shrunk by the unit's radius.
- **Overlapping surfaces.** Height is not stored. Where one surface passes over
  another, `locate` returns the polygon with the lowest id; track the unit's
  polygon and call `find_path_between` instead.
- **Shortest paths.** The route is the shortest one through the polygons A*
  picked. That choice is exact while the goal is in sight and a close estimate
  otherwise, so a route is good but not provably the shortest.
- **Memory.** Each path query allocates working memory in proportion to the
  number of polygons.

## Conventions

- Coordinates are integer millimetres (`i32`, 1 m = 1000) on the ground plane.
- Everything the simulation reads uses exact integer arithmetic. No floats, no
  platform math library.
- No iteration over hash containers. Ties break by polygon id.
- Polygons are convex and wound counter-clockwise.
- No engine dependency. The crate has zero dependencies.

## Using it from another project

```toml
[dependencies]
vignav = { git = "https://github.com/AlexJVig/VigNav" }
```

Or, with a checkout next to the consuming workspace:

```toml
[dependencies]
vignav = { path = "../VigNav" }
```

Rust 1.98 or newer, edition 2024; the toolchain is pinned in
`rust-toolchain.toml`.

## Roadmap

1. **Navmesh and queries.** Done, within the limits above.
2. **Generation.** Build layers for maps that ship no mesh, from terrain,
   passability and structure footprints.
3. **Dynamic obstacles.** Structures placed or destroyed re-cut the mesh
   locally.
4. **Steering.** Path following, local avoidance and group movement.
5. **Later.** Line-of-sight queries; heights and overlapping surfaces; reusable
   search memory; provably shortest routes; tiles, streaming and
   serialisation; jump and point-to-point links; generation from arbitrary 3D
   geometry; flying and swimming volumes; debug tooling.

## Tests

```bash
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

CI runs all three on every pull request, with the tests on Linux, macOS and
Windows.

The suite lives in `tests/suite/`, one module per flow:

| Module | What it covers |
|---|---|
| `build` | Both ways of building a mesh, and every way they refuse bad input |
| `lookup` | `locate`, `nearest` and `contains` |
| `paths` | Routes with known answers, on hand-made meshes and maps drawn as text |
| `random` | Random maps, checked against independent searches |
| `determinism` | A recorded fingerprint of routes, which must match on every platform |

To cover something new, add a test to the module it belongs to. A scenario is
usually a small map drawn as text:

```rust
let map = Map::drawn(&[
    "S.#..",
    "..#..",
    "....G",
]);
assert_eq!(route(&map).points, [map.at('S'), map.corner(2, 1), map.at('G')]);
```

If a change alters any route, the fingerprint test fails and prints the new
value. Update it only when the change is meant.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT), at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in VigNav by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
