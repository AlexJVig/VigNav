//! Routes must come out as the same bits everywhere. Lockstep peers on
//! different machines exchange only their inputs, so any difference in a
//! route is a desync.

use super::support::*;

/// The fingerprint of the routes below. CI runs this on Linux, macOS and
/// Windows, so a platform that computes anything differently fails here.
///
/// A change to the path code that alters any route changes this too. If
/// that is intended, replace the value with the one the failure prints.
const RECORDED: u64 = 0x27ab_09b7_55d2_3f7b;

#[test]
fn routes_match_the_recorded_fingerprint() {
    // FNV-1a over every number of every answer.
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |value: i64| {
        for byte in value.to_le_bytes() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for seed in 1..=6_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let map = Map::random(&mut rng, 24, 16, 28);
        let mesh = map.navmesh();
        for _ in 0..60 {
            // Any cell, blocked or not, so that snapping is covered too.
            let (start, goal) = (
                map.centre(rng.below(24 * 16)),
                map.centre(rng.below(24 * 16)),
            );
            match mesh.find_path(start, goal, 1500) {
                Ok(path) => {
                    mix(path.points.len() as i64);
                    path.points.iter().for_each(|point| {
                        mix(point.x.into());
                        mix(point.y.into());
                    });
                    path.corridor.iter().for_each(|poly| mix(poly.0.into()));
                }
                Err(error) => mix(-1 - error as i64),
            }
        }
        // The same again with obstacles of both kinds scattered about.
        let mut mesh = mesh;
        let mut edit = mesh.edit_obstacles();
        for k in 0..16_u32 {
            let (x, y) = (rng.below(24_000) as i32, rng.below(16_000) as i32);
            let reach = |rng: &mut Rng| 200 + rng.below(1800) as i32;
            let shape = [
                p(x - reach(&mut rng), y),
                p(x, y - reach(&mut rng)),
                p(x + reach(&mut rng), y),
                p(x, y + reach(&mut rng)),
            ];
            let kind = if k.is_multiple_of(4) {
                ObstacleKind::Avoid
            } else {
                ObstacleKind::Blocked
            };
            edit.add(&shape, kind).unwrap();
        }
        drop(edit);
        for _ in 0..40 {
            let (start, goal) = (
                map.centre(rng.below(24 * 16)),
                map.centre(rng.below(24 * 16)),
            );
            match mesh.find_path_toward(start, goal, 3000) {
                Ok(path) => {
                    mix(path.points.len() as i64);
                    path.points.iter().for_each(|point| {
                        mix(point.x.into());
                        mix(point.y.into());
                    });
                    path.corridor.iter().for_each(|poly| mix(poly.0.into()));
                }
                Err(error) => mix(-1 - error as i64),
            }
        }
    }
    assert_eq!(
        hash, RECORDED,
        "routes changed; if that is intended, set RECORDED to {hash:#018x}"
    );
}

#[test]
fn moving_the_map_moves_the_routes_and_nothing_else() {
    let offset = p(-123_457, 98_761);
    let moved = |q: &Point| p(q.x + offset.x, q.y + offset.y);
    for seed in 1..=4_u64 {
        let seeded = || Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let here = Map::random(&mut seeded(), 20, 14, 25);
        let there = Map::random(&mut seeded(), 20, 14, 25).moved_to(offset);
        let (near_mesh, far_mesh) = (here.navmesh(), there.navmesh());
        let mut rng = Rng(seed);
        for _ in 0..50 {
            let (from, to) = (rng.below(20 * 14), rng.below(20 * 14));
            let near = near_mesh.find_path(here.centre(from), here.centre(to), 1500);
            let far = far_mesh.find_path(there.centre(from), there.centre(to), 1500);
            let expected = near.map(|path| Path {
                points: path.points.iter().map(moved).collect(),
                corridor: path.corridor,
            });
            assert_eq!(far, expected, "seed {seed}, cell {from} to cell {to}");
        }
    }
}

#[test]
fn asking_twice_gives_the_same_route() {
    let mut rng = Rng(42);
    let map = Map::random(&mut rng, 20, 14, 25);
    let (mesh, again) = (map.navmesh(), map.navmesh());
    for _ in 0..100 {
        let (start, goal) = (
            map.centre(rng.below(20 * 14)),
            map.centre(rng.below(20 * 14)),
        );
        let first = mesh.find_path(start, goal, 1500);
        assert_eq!(mesh.find_path(start, goal, 1500), first);
        // A mesh built a second time from the same input answers the same.
        assert_eq!(again.find_path(start, goal, 1500), first);
    }
}
