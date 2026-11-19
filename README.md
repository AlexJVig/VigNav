# VigNav

Deterministic navigation for RTS games, in Rust: navmesh, pathfinding and
steering that give the same result on every machine in a lockstep match.

**Status.** Loading a navmesh, looking points up on it, finding paths across
it and laying obstacles over it at run time work and are tested. Mesh
generation and steering are on the roadmap below.

VigNav is an independent project built from published algorithms. Its feature
set is modelled on what commercial navigation middleware such as NavPower
offers; it is not affiliated with NavPower or BabelFlux and shares no code or
file formats with it.

## What it does today

- **Build.** `NavMesh::from_polygons` links convex polygons wherever two share
  an edge. `NavMesh::from_linked_polygons` takes links that are already known,
  as in a mesh exported from another tool.
- **Look up.** `locate` finds the polygon a point is on, and `locate_all` every
  polygon where surfaces overlap. `nearest` finds the closest point of the
  mesh, and `nearest_reachable` the closest one a given unit can get to.
- **Route.** `find_path` and `find_path_between` return the corner points of a
  route and the polygons it crosses. `find_path_toward` routes as near to the
  goal as the start can get when the goal itself is out of reach.
- **See.** `line_of_sight` tells whether a unit can walk straight from one
  point to another.
- **Obstruct.** `add_obstacle` and `remove_obstacle` lay a convex shape over
  the mesh and lift it again. `edit_obstacles` changes many at once.
  `geom::grow_convex` grows a shape by a unit's clearance.

```rust
use vignav::geom::grow_convex;
use vignav::{NavMesh, ObstacleKind, Point};

// Two 10 m squares side by side, sharing the edge at x = 10 m.
let vertices = [
    Point::new(0, 0),
    Point::new(10_000, 0),
    Point::new(20_000, 0),
    Point::new(0, 10_000),
    Point::new(10_000, 10_000),
    Point::new(20_000, 10_000),
];
let mut mesh = NavMesh::from_polygons(&vertices, &[[0, 1, 4, 3], [1, 2, 5, 4]])?;

// Endpoints up to half a metre off the mesh are moved onto it.
let (start, goal) = (Point::new(1_000, 5_000), Point::new(19_000, 5_000));
let path = mesh.find_path(start, goal, 500)?;
assert_eq!(path.points, [start, goal]);

// A building in the way, grown by the half metre these units need round them.
let building = [
    Point::new(9_000, 3_000),
    Point::new(11_000, 3_000),
    Point::new(11_000, 7_000),
    Point::new(9_000, 7_000),
];
let id = mesh.add_obstacle(&grow_convex(&building, 500), ObstacleKind::Blocked)?;
assert!(mesh.find_path(start, goal, 500)?.points.len() > 2);

// Gone again, and the route is straight again.
mesh.remove_obstacle(id);
assert_eq!(mesh.find_path(start, goal, 500)?.points, [start, goal]);
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

## Obstacles

An obstacle is a convex shape laid over a mesh after it is built. The
polygons of the mesh never change; lookups and routes see the ground the
obstacles leave.

- **Two kinds.** `Blocked` ground is off the mesh: a building, a wall, a rock.
  `Avoid` ground turns other routes away, but a route that starts on it may
  leave and a goal on it is moved to its edge: a unit standing still.
- **Per unit class.** Lay an obstacle on the mesh of every class that cannot
  cross it, grown by that class's clearance, and leave it off the meshes of
  classes that crush it.
- **Cheap to change.** Only the polygons under an obstacle are cut again, and
  only those and their neighbours are joined up again. Change many at once
  through `edit_obstacles`, which does that work once.
- **The same everywhere.** Obstacle ids are handed out in order and never
  reused, and the ground depends only on which obstacles are down, not on the
  order they came and went in.

## Limits

- **Unit size.** Routes touch the corners of the mesh and of obstacles, so the
  mesh must already be shrunk, and each obstacle grown, by the unit's radius.
- **Height.** Nothing has a height. Where one surface passes over another,
  `locate` returns the polygon with the lowest id: use `locate_all` and pick,
  then route with `find_path_between`. An obstacle cuts every surface it lies
  over.
- **Shortest paths.** A route is straight whenever a straight walk is clear.
  Otherwise it is the shortest one through the ground A* picked, with any
  corner dropped that the corner before it can see past. That is good, but
  not provably the shortest.
- **Kinds of ground.** Ground is ordinary, avoided or blocked. There is no
  cost per area.
- **Line of sight.** The answer is yes or no. There is no point of impact.
- **Meshes cut elsewhere.** Every link must run along a whole edge on both
  sides. A mesh with obstacles already cut into it does not load; load the
  mesh without them and lay them as obstacles.
- **Tolerance.** A point within 2 mm of the mesh counts as on it when routing.

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
3. **Obstacles.** Done, within the limits above.
4. **Steering.** Path following, local avoidance and group movement.
5. **Later.** Where a blocked line of sight stops; heights; the walls at each
   corner of a route; a cost per area; provably shortest routes; links along
   part of an edge; tiles, streaming and serialisation; jump and
   point-to-point links; generation from arbitrary 3D geometry; flying and
   swimming volumes; debug tooling.

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
| `obstacles` | Laying, lifting and routing round obstacles of both kinds |
| `sight` | Straight-line tests, and ground that overlaps |
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

For speed, `cargo run --release --example timing` prints rough timings of
route queries and obstacle changes on a mesh the size of a large map.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT), at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in VigNav by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
