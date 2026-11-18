//! Path queries on random maps, checked against independent searches.

use super::support::*;

#[test]
fn an_open_map_gives_straight_paths() {
    let map = Map::open(20, 14);
    let (mesh, _) = map.mesh();
    assert_eq!(mesh.polygon_count(), 14);
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for _ in 0..200 {
        let (start, goal) = (map.centre(rng.below(280)), map.centre(rng.below(280)));
        let path = mesh.find_path(start, goal, 0).unwrap();
        if start != goal {
            assert_eq!(path.points, [start, goal]);
        }
        check_path(&mesh, &path, start, goal);
    }
}

#[test]
fn random_maps_give_valid_paths() {
    let (mut reachable, mut unreachable) = (0, 0);
    for seed in 1..=12_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let map = Map::random(&mut rng, 20, 14, 20 + 2 * seed as usize);
        let (mesh, cell_poly) = map.mesh();
        let regions = map.regions();
        let free: Vec<usize> = (0..map.blocked.len())
            .filter(|&cell| !map.blocked[cell])
            .collect();
        for _ in 0..80 {
            let (from, to) = (free[rng.below(free.len())], free[rng.below(free.len())]);
            let (start, goal) = (map.centre(from), map.centre(to));
            assert_eq!(mesh.locate(start), cell_poly[from]);
            let found = mesh.find_path(start, goal, 0);
            if regions[from] != regions[to] {
                assert_eq!(found, Err(PathError::Unreachable));
                unreachable += 1;
                continue;
            }
            let path = found.unwrap();
            assert_eq!(path.corridor.first().copied(), cell_poly[from]);
            assert_eq!(path.corridor.last().copied(), cell_poly[to]);
            check_path(&mesh, &path, start, goal);
            // A route free to cut across polygons must not lose to one tied
            // to the cell grid.
            let octile = map.octile(from, to);
            assert!(
                path.length() <= octile,
                "seed {seed}: {start:?} to {goal:?} is {} mm, the grid route {octile} mm",
                path.length()
            );
            // The same question gets the same answer.
            assert_eq!(mesh.find_path(start, goal, 0).unwrap(), path);
            reachable += 1;
        }
    }
    // The maps must exercise both outcomes for the test to mean anything.
    assert!(
        reachable > 200 && unreachable > 50,
        "{reachable} reachable, {unreachable} not"
    );
}

#[test]
fn given_links_route_the_same_as_derived_ones() {
    for seed in 1..=4_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let map = Map::random(&mut rng, 20, 14, 25);
        let (vertices, polygons, _) = map.polygons();
        let derived = NavMesh::from_polygons(&vertices, &polygons).unwrap();
        let given: Vec<Vec<Option<u32>>> = (0..derived.polygon_count())
            .map(|poly| {
                derived
                    .neighbors(PolyId(poly as u32))
                    .map(|n| n.map(|id| id.0))
                    .collect()
            })
            .collect();
        let linked = NavMesh::from_linked_polygons(&vertices, &polygons, &given).unwrap();
        for _ in 0..50 {
            let (start, goal) = (
                map.centre(rng.below(20 * 14)),
                map.centre(rng.below(20 * 14)),
            );
            assert_eq!(
                linked.find_path(start, goal, 1500),
                derived.find_path(start, goal, 1500)
            );
        }
    }
}
