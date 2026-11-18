# VigNav

Deterministic navigation for RTS games, in Rust: navmesh, pathfinding and
steering that give the same result on every machine in a lockstep match.

**Status: skeleton.** The crate builds and carries only the integer geometry
foundation (`vignav::geom`). Everything else is on the roadmap below.

VigNav is an independent project built from published algorithms. Its feature
set is modelled on what commercial navigation middleware such as NavPower
offers; it is not affiliated with NavPower or BabelFlux and shares no code or
file formats with it.

## Conventions

- Coordinates are integer millimetres (`i32`, 1 m = 1000) on the ground plane.
  Height is carried separately.
- Everything the simulation reads uses exact integer arithmetic. No floats, no
  platform math library.
- No iteration over hash containers.
- Polygons are convex and wound counter-clockwise.
- No engine dependency. The crate has zero dependencies today.

## Using it from a sibling project

```toml
[dependencies]
vignav = { path = "../VigNav" }
```

Adjust the path to where the consuming workspace sits. Rust 1.98 or newer,
edition 2024; the toolchain is pinned in `rust-toolchain.toml`.

## Roadmap

1. **Navmesh and queries.** Convex-polygon layers, one per unit class, loadable
   from meshes a game already has. Point location, A* and funnel paths.
2. **Generation.** Build layers for maps that ship no mesh, from terrain,
   passability and structure footprints.
3. **Dynamic obstacles.** Structures placed or destroyed re-cut the mesh
   locally.
4. **Steering.** Path following, local avoidance and group movement.
5. **Later.** Tiles, streaming and serialisation; jump and point-to-point
   links; generation from arbitrary 3D geometry; flying and swimming volumes;
   debug tooling; CI.

## Development

```bash
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT), at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in VigNav by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
