//! Path queries on hand-made meshes with known answers.

use super::support::*;

#[test]
fn a_visible_goal_gives_a_straight_path() {
    let mesh = l_shape();
    let (start, goal) = (p(100, 500), p(1900, 500));
    let path = mesh.find_path(start, goal, 0).unwrap();
    assert_eq!(path.points, [start, goal]);
    assert_eq!(path.corridor, [A, B]);
    assert_eq!(path.length(), 1800);

    let here = mesh.find_path(start, start, 0).unwrap();
    assert_eq!((here.points, here.corridor,), (vec![start], vec![A]));
}

#[test]
fn the_path_bends_round_the_inner_corner() {
    let mesh = l_shape();
    let (start, goal) = (p(500, 900), p(1100, 1900));
    let path = mesh.find_path(start, goal, 0).unwrap();
    assert_eq!(path.points, [start, p(M, M), goal]);
    assert_eq!(path.corridor, [A, B, C]);

    // And the same route backwards.
    let back = mesh.find_path(goal, start, 0).unwrap();
    assert_eq!(back.points, [goal, p(M, M), start]);
    assert_eq!(back.corridor, [C, B, A]);
}

#[test]
fn endpoints_off_the_mesh_are_snapped_or_refused() {
    let mesh = l_shape();
    // From the notch, 400 mm above A, to a point 300 mm right of B.
    let path = mesh.find_path(p(500, 1400), p(2300, 500), 400).unwrap();
    assert_eq!(path.points, [p(500, M), p(2 * M, 500)]);
    assert_eq!(
        mesh.find_path(p(500, 1400), p(1500, 500), 399),
        Err(PathError::StartOffMesh)
    );
    assert_eq!(
        mesh.find_path(p(1500, 500), p(2300, 500), 299),
        Err(PathError::GoalOffMesh)
    );
}

#[test]
fn separate_islands_do_not_connect() {
    let vertices = [
        p(0, 0),
        p(M, 0),
        p(M, M),
        p(0, M),
        p(5 * M, 0),
        p(6 * M, 0),
        p(6 * M, M),
        p(5 * M, M),
    ];
    let mesh = NavMesh::from_polygons(&vertices, &[[0, 1, 2, 3], [4, 5, 6, 7]]).unwrap();
    assert_ne!(mesh.island(A), mesh.island(B));
    assert_eq!(
        mesh.find_path(p(500, 500), p(5500, 500), 0),
        Err(PathError::Unreachable)
    );
    let (start, goal) = (
        Location {
            poly: A,
            point: p(500, 500),
        },
        Location {
            poly: B,
            point: p(5500, 500),
        },
    );
    assert_eq!(mesh.find_path_between(start, goal), None);
    // An id from some other mesh is refused, not a panic.
    assert_eq!(
        mesh.find_path_between(
            start,
            Location {
                poly: PolyId(7),
                point: p(0, 0)
            }
        ),
        None
    );
}

#[test]
fn an_empty_mesh_answers_nothing() {
    let mesh = NavMesh::from_polygons(&[], &[[0_u32; 3]; 0]).unwrap();
    assert_eq!(mesh.polygon_count(), 0);
    assert_eq!(mesh.locate(p(0, 0)), None);
    assert_eq!(mesh.nearest(p(0, 0), M), None);
    assert_eq!(
        mesh.find_path(p(0, 0), p(1, 1), M),
        Err(PathError::StartOffMesh)
    );
}

/// Routes a drawn map from `S` to `G` and puts the result through
/// `check_path`.
fn route(map: &Map) -> Path {
    let mesh = map.navmesh();
    let (start, goal) = (map.at('S'), map.at('G'));
    let path = mesh.find_path(start, goal, 0).unwrap();
    check_path(&mesh, &path, start, goal);
    path
}

#[test]
fn a_wall_is_passed_at_its_open_end() {
    let map = Map::drawn(&[
        "S.#..", //
        "..#..", //
        "....G", //
    ]);
    assert_eq!(
        route(&map).points,
        [map.at('S'), map.corner(2, 1), map.at('G')]
    );
}

#[test]
fn the_shorter_way_round_is_taken() {
    let rows = [
        ".......", //
        "S..#..G", //
        "...#...", //
        "...#...", //
        ".......", //
    ];
    // The wall is nearer the top, so over it is shorter than under it.
    let map = Map::drawn(&rows);
    let over = [map.at('S'), map.corner(3, 4), map.corner(4, 4), map.at('G')];
    assert_eq!(route(&map).points, over);

    // Upside down, under it is.
    let flipped: Vec<&str> = rows.iter().rev().copied().collect();
    let map = Map::drawn(&flipped);
    let under = [map.at('S'), map.corner(3, 1), map.corner(4, 1), map.at('G')];
    assert_eq!(route(&map).points, under);
}

#[test]
fn a_winding_corridor_is_followed_corner_by_corner() {
    let map = Map::drawn(&[
        "S....", //
        "####.", //
        ".....", //
        ".####", //
        "....G", //
    ]);
    let turns = [
        map.corner(4, 4),
        map.corner(4, 3),
        map.corner(1, 2),
        map.corner(1, 1),
    ];
    let path = route(&map);
    assert_eq!(path.points[1..5], turns);
    assert_eq!(path.points.len(), 6);

    // The way back is the same route reversed.
    let mesh = map.navmesh();
    let back = mesh.find_path(map.at('G'), map.at('S'), 0).unwrap();
    assert!(back.points.iter().eq(path.points.iter().rev()));
    assert_eq!(back.length(), path.length());
}

#[test]
fn a_walled_in_goal_is_unreachable() {
    let map = Map::drawn(&[
        "S.#G", //
        "..##", //
    ]);
    let (mesh, cell_poly) = map.mesh();
    assert_eq!(
        mesh.find_path(map.at('S'), map.at('G'), 0),
        Err(PathError::Unreachable)
    );
    let island = |mark: char| mesh.island(cell_poly[map.cell(mark)].unwrap());
    assert_ne!(island('S'), island('G'));
}

#[test]
fn routes_are_exact_at_the_edges_of_the_coordinate_range() {
    // The L of three squares, each 100 km across, pushed into the far corner
    // of the range and then into the opposite one.
    let side = 100_000_000;
    for base in [COORD_LIMIT - 2 * side, -COORD_LIMIT] {
        let vertices: Vec<Point> = L_VERTICES
            .iter()
            .map(|v| p(base + v.x / M * side, base + v.y / M * side))
            .collect();
        let mesh = NavMesh::from_polygons(&vertices, &L_POLYGONS).unwrap();
        // A position in tenths of a side.
        let tenths = |x: i32, y: i32| p(base + x * (side / 10), base + y * (side / 10));

        let (start, goal) = (tenths(5, 9), tenths(11, 19));
        let path = mesh.find_path(start, goal, 0).unwrap();
        assert_eq!(path.points, [start, tenths(10, 10), goal]);
        assert_eq!(path.corridor, [A, B, C]);
        check_path(&mesh, &path, start, goal);

        // From the notch, 40 km above A and 50 km left of C.
        let snapped = Location {
            poly: A,
            point: tenths(5, 10),
        };
        assert_eq!(mesh.nearest(tenths(5, 14), side / 2), Some(snapped));
    }
}
