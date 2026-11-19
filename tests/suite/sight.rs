//! Straight-line tests, and telling apart ground that overlaps.

use super::support::*;

#[test]
fn a_clear_line_is_walkable_across_polygons() {
    let mesh = l_shape();
    let from = Location {
        poly: A,
        point: p(100, 500),
    };
    let to = p(1900, 500);
    assert_eq!(
        mesh.line_of_sight(from, to),
        Some(Location { poly: B, point: to })
    );
    // Standing still is always clear.
    assert_eq!(mesh.line_of_sight(from, from.point), Some(from));
    // Round the inner corner there is no straight way.
    let low = Location {
        poly: A,
        point: p(500, 900),
    };
    assert_eq!(mesh.line_of_sight(low, p(1100, 1900)), None);
    // Nor to anywhere off the mesh.
    assert_eq!(mesh.line_of_sight(from, p(500, 1500)), None);
    assert_eq!(mesh.line_of_sight(from, p(i32::MAX, 0)), None);
    assert_eq!(
        mesh.line_of_sight(
            Location {
                poly: PolyId(9),
                point: p(0, 0)
            },
            to
        ),
        None
    );
    // A line through the inner corner itself only touches the wall.
    let corner_to_corner = mesh.line_of_sight(
        Location {
            poly: A,
            point: p(0, 0),
        },
        p(2 * M, 2 * M),
    );
    assert_eq!(
        corner_to_corner,
        Some(Location {
            poly: C,
            point: p(2 * M, 2 * M)
        })
    );
}

#[test]
fn a_line_may_touch_obstacles_but_not_squeeze_between_them() {
    let mut mesh = field();
    // Two 4 m squares that meet at the single point (10 m, 10 m).
    mesh.add_obstacle(&rect(6, 6, 10, 10), ObstacleKind::Blocked)
        .unwrap();
    mesh.add_obstacle(&rect(10, 10, 14, 14), ObstacleKind::Blocked)
        .unwrap();
    let see = |from: Point, to: Point| {
        mesh.line_of_sight(mesh.nearest(from, 0).unwrap(), to)
            .is_some()
    };

    // Straight through one of them.
    assert!(!see(p(2 * M, 8 * M), p(18 * M, 8 * M)));
    // Along a wall, either way.
    assert!(see(p(2 * M, 6 * M), p(18 * M, 6 * M)));
    assert!(see(p(18 * M, 6 * M), p(2 * M, 6 * M)));
    assert!(see(p(6 * M, 2 * M), p(6 * M, 18 * M)));
    // Past a corner, touching it.
    assert!(see(p(4 * M, 9 * M), p(8 * M, 11 * M)));
    assert!(see(p(8 * M, 11 * M), p(4 * M, 9 * M)));
    // Between the two, through the point where they meet.
    let (north_west, south_east) = (p(8 * M, 12 * M), p(12 * M, 8 * M));
    assert!(!see(north_west, south_east));
    assert!(!see(south_east, north_west));
    // A route between those two has to go round one of the squares.
    let path = mesh.find_path(north_west, south_east, 0).unwrap();
    assert!(path.points.len() >= 4);
    assert!(path.length() > dist(north_west, south_east) + 4 * i64::from(M));
    check_route(&mesh, &path, north_west, south_east);
}

#[test]
fn overlapping_ground_is_told_apart_by_polygon() {
    // A road, and a deck that passes over it without touching it: two
    // squares over the same ground, joined to nothing.
    let square = rect(0, 0, 10, 10);
    let vertices: Vec<Point> = square.iter().chain(&square).copied().collect();
    let none = [None; 4];
    let mesh =
        NavMesh::from_linked_polygons(&vertices, &[[0, 1, 2, 3], [4, 5, 6, 7]], &[none, none])
            .unwrap();
    let (road, deck) = (PolyId(0), PolyId(1));
    let (here, there) = (p(2 * M, 5 * M), p(8 * M, 5 * M));

    assert_eq!(mesh.locate(here), Some(road));
    assert_eq!(mesh.locate_all(here), [road, deck]);
    assert_eq!(mesh.locate_all(p(-1, 5 * M)), []);

    let on = |poly: PolyId, point: Point| Location { poly, point };
    assert_eq!(
        mesh.find_path_between(on(deck, here), on(deck, there))
            .unwrap()
            .corridor,
        [deck]
    );
    assert_eq!(
        mesh.find_path_between(on(road, here), on(road, there))
            .unwrap()
            .corridor,
        [road]
    );
    assert_eq!(
        mesh.find_path_between(on(road, here), on(deck, there)),
        None
    );
    assert_eq!(
        mesh.line_of_sight(on(deck, here), there),
        Some(on(deck, there))
    );
}

#[test]
fn an_obstacle_cuts_every_level_it_lies_over() {
    // Obstacles have no height: one under a deck takes its ground from the
    // deck as well. The README lists this as a limit.
    let square = rect(0, 0, 10, 10);
    let vertices: Vec<Point> = square.iter().chain(&square).copied().collect();
    let none = [None; 4];
    let polygons = [[0, 1, 2, 3], [4, 5, 6, 7]];
    let mut mesh = NavMesh::from_linked_polygons(&vertices, &polygons, &[none, none]).unwrap();
    mesh.add_obstacle(&rect(4, 4, 6, 6), ObstacleKind::Blocked)
        .unwrap();
    assert_eq!(mesh.locate_all(p(5 * M, 5 * M)), []);
    assert_eq!(mesh.locate_all(p(2 * M, 5 * M)), [PolyId(0), PolyId(1)]);
    let on_deck = |point: Point| Location {
        poly: PolyId(1),
        point,
    };
    let path = mesh
        .find_path_between(on_deck(p(2 * M, 5 * M)), on_deck(p(8 * M, 5 * M)))
        .unwrap();
    assert_eq!(path.points.len(), 4);
    assert_eq!(path.corridor, [PolyId(1)]);
}
