//! Obstacles: ground cut out of a mesh after it is built, and put back.

use super::support::*;

/// A 6 m square standing on one corner in the middle of the field.
const DIAMOND: [Point; 4] = [
    Point::new(13 * M, 10 * M),
    Point::new(10 * M, 13 * M),
    Point::new(7 * M, 10 * M),
    Point::new(10 * M, 7 * M),
];
const CENTRE: Point = Point::new(10 * M, 10 * M);

#[test]
fn a_route_bends_round_an_obstacle_and_straightens_when_it_goes() {
    let mut mesh = field();
    // Half a metre above the middle, so over the top is the shorter way.
    let (start, goal) = (p(2 * M, 10_500), p(18 * M, 10_500));
    assert_eq!(
        mesh.find_path(start, goal, 0).unwrap().points,
        [start, goal]
    );

    let id = mesh.add_obstacle(&DIAMOND, ObstacleKind::Blocked).unwrap();
    assert_eq!(mesh.obstacle_count(), 1);
    let path = mesh.find_path(start, goal, 0).unwrap();
    assert_eq!(path.points, [start, p(10 * M, 13 * M), goal]);
    check_route(&mesh, &path, start, goal);

    assert!(mesh.remove_obstacle(id));
    assert!(!mesh.remove_obstacle(id));
    assert_eq!(mesh.obstacle_count(), 0);
    assert_eq!(
        mesh.find_path(start, goal, 0).unwrap().points,
        [start, goal]
    );
}

#[test]
fn ground_under_an_obstacle_is_off_the_mesh() {
    let mut mesh = field();
    mesh.add_obstacle(&DIAMOND, ObstacleKind::Blocked).unwrap();
    assert_eq!(mesh.locate(CENTRE), None);
    assert_eq!(mesh.locate_all(CENTRE), []);
    // Its boundary is still ground.
    assert_eq!(mesh.locate(p(10 * M, 13 * M)), Some(PolyId(0)));
    assert_eq!(mesh.locate(p(8500, 11_500)), Some(PolyId(0)));
    // The polygon itself is unchanged.
    assert!(mesh.contains(PolyId(0), CENTRE));
    assert_eq!(mesh.polygon_count(), 1);

    // The nearest ground is the middle of a side, 1.5 m off on each axis.
    let edge = mesh.nearest(CENTRE, 3 * M).unwrap();
    assert_eq!(dist_sq(CENTRE, edge.point), 2 * 1500 * 1500);
    assert_eq!(mesh.nearest(CENTRE, 2 * M), None);

    let start = p(2 * M, 10 * M);
    assert_eq!(
        mesh.find_path(start, CENTRE, 0),
        Err(PathError::GoalOffMesh)
    );
    assert_eq!(
        mesh.find_path(CENTRE, start, 0),
        Err(PathError::StartOffMesh)
    );
}

#[test]
fn a_bad_obstacle_is_refused_and_changes_nothing() {
    let mut mesh = field();
    let clockwise = [DIAMOND[3], DIAMOND[2], DIAMOND[1], DIAMOND[0]];
    assert_eq!(
        mesh.add_obstacle(&clockwise, ObstacleKind::Blocked),
        Err(MeshError::NotCounterClockwise { polygon: 0 })
    );
    assert_eq!(
        mesh.add_obstacle(&DIAMOND[..2], ObstacleKind::Blocked),
        Err(MeshError::TooFewCorners { polygon: 0 })
    );
    let far = [p(0, 0), p(i32::MAX, 0), p(0, M)];
    assert_eq!(
        mesh.add_obstacle(&far, ObstacleKind::Blocked),
        Err(MeshError::CoordinateOutOfRange { vertex: 1 })
    );
    assert_eq!(mesh.obstacle_count(), 0);
    assert_eq!(mesh.locate(CENTRE), Some(PolyId(0)));
}

#[test]
fn obstacles_route_as_blocked_cells_do() {
    let wall = ["S.#..", "..#..", "....G"];
    let either_way = [".......", "S..#..G", "...#...", "...#...", "......."];
    let winding = ["S....", "####.", ".....", ".####", "....G"];
    for rows in [&wall[..], &either_way[..], &winding[..]] {
        let map = Map::drawn(rows);
        let (drawn, laid) = (map.navmesh(), map.mesh_with_obstacles());
        let (start, goal) = (map.at('S'), map.at('G'));
        let path = laid.find_path(start, goal, 0).unwrap();
        assert_eq!(path.points, drawn.find_path(start, goal, 0).unwrap().points);
        check_route(&laid, &path, start, goal);
    }
    let sealed = Map::drawn(&["S.#G", "..##"]);
    let laid = sealed.mesh_with_obstacles();
    assert_eq!(
        laid.find_path(sealed.at('S'), sealed.at('G'), 0),
        Err(PathError::Unreachable)
    );
}

#[test]
fn random_maps_laid_as_obstacles_give_valid_paths() {
    let (mut reachable, mut unreachable) = (0, 0);
    for seed in 1..=8_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let map = Map::random(&mut rng, 20, 14, 20 + 2 * seed as usize);
        let mesh = map.mesh_with_obstacles();
        let regions = map.regions();
        let free: Vec<usize> = (0..map.blocked.len())
            .filter(|&cell| !map.blocked[cell])
            .collect();
        for _ in 0..60 {
            let (from, to) = (free[rng.below(free.len())], free[rng.below(free.len())]);
            let (start, goal) = (map.centre(from), map.centre(to));
            let found = mesh.find_path(start, goal, 0);
            if regions[from] != regions[to] {
                assert_eq!(found, Err(PathError::Unreachable));
                unreachable += 1;
                continue;
            }
            let path = found.unwrap();
            check_route(&mesh, &path, start, goal);
            let octile = map.octile(from, to);
            assert!(
                path.length() <= octile,
                "seed {seed}: {start:?} to {goal:?} is {} mm, the grid route {octile} mm",
                path.length()
            );
            reachable += 1;
        }
    }
    assert!(
        reachable > 150 && unreachable > 30,
        "{reachable} reachable, {unreachable} not"
    );
}

#[test]
fn removing_every_obstacle_restores_the_ground() {
    for seed in 1..=4_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let map = Map::random(&mut rng, 20, 14, 30);
        let pristine = Map::open(20, 14).navmesh();
        let mut mesh = Map::open(20, 14).navmesh();
        let blocked: Vec<usize> = (0..map.blocked.len())
            .filter(|&cell| map.blocked[cell])
            .collect();

        // All at once, then one by one, must leave the same ground.
        let mut one_by_one = Map::open(20, 14).navmesh();
        let mut edit = mesh.edit_obstacles();
        let ids: Vec<ObstacleId> = blocked
            .iter()
            .map(|&cell| {
                edit.add(&map.cell_corners(cell), ObstacleKind::Blocked)
                    .unwrap()
            })
            .collect();
        drop(edit);
        for &cell in &blocked {
            one_by_one
                .add_obstacle(&map.cell_corners(cell), ObstacleKind::Blocked)
                .unwrap();
        }
        let ask = |rng: &mut Rng| {
            (
                map.centre(rng.below(20 * 14)),
                map.centre(rng.below(20 * 14)),
            )
        };
        for _ in 0..40 {
            let (start, goal) = ask(&mut rng);
            assert_eq!(
                mesh.find_path(start, goal, 1500),
                one_by_one.find_path(start, goal, 1500)
            );
        }

        // Lift half and the rest must match a mesh that only ever had the rest.
        let mut half = Map::open(20, 14).navmesh();
        for (k, (&cell, &id)) in blocked.iter().zip(&ids).enumerate() {
            if k % 2 == 0 {
                assert!(mesh.remove_obstacle(id));
            } else {
                half.add_obstacle(&map.cell_corners(cell), ObstacleKind::Blocked)
                    .unwrap();
            }
        }
        for _ in 0..40 {
            let (start, goal) = ask(&mut rng);
            assert_eq!(
                mesh.find_path(start, goal, 1500),
                half.find_path(start, goal, 1500)
            );
        }

        let mut edit = mesh.edit_obstacles();
        ids.iter()
            .skip(1)
            .step_by(2)
            .for_each(|&id| assert!(edit.remove(id)));
        drop(edit);
        assert_eq!(mesh.obstacle_count(), 0);
        for _ in 0..40 {
            let (start, goal) = ask(&mut rng);
            assert_eq!(
                mesh.find_path(start, goal, 0),
                pristine.find_path(start, goal, 0)
            );
        }
    }
}

#[test]
fn a_grown_obstacle_keeps_routes_at_arms_length() {
    let mut mesh = field();
    let building = rect(9, 9, 11, 11);
    mesh.add_obstacle(&grow_convex(&building, 1000), ObstacleKind::Blocked)
        .unwrap();
    let (start, goal) = (p(2 * M, 10 * M), p(18 * M, 10 * M));
    let path = mesh.find_path(start, goal, 0).unwrap();
    assert!(path.points.len() >= 3);
    check_route(&mesh, &path, start, goal);
    // Distance from the building itself, which is a square.
    let off = |q: Point| {
        let away = |c: i32| f64::from((9 * M - c).max(c - 11 * M).max(0));
        away(q.x).hypot(away(q.y))
    };
    for q in samples(&path) {
        assert!(off(q) >= 999.0, "{q:?} is {} mm from the building", off(q));
    }
}

/// A 2 m square in the middle of a 10 m map, as a unit standing still.
fn standing(kind: ObstacleKind) -> NavMesh {
    let mut mesh = Map::open(10, 10).navmesh();
    mesh.add_obstacle(&rect(4, 4, 6, 6), kind).unwrap();
    mesh
}

#[test]
fn avoided_ground_turns_other_routes_away() {
    let (avoided, blocked) = (
        standing(ObstacleKind::Avoid),
        standing(ObstacleKind::Blocked),
    );
    let (start, goal) = (p(500, 5200), p(9500, 5200));
    let path = avoided.find_path(start, goal, 0).unwrap();
    assert_eq!(path.points, [start, p(4 * M, 6 * M), p(6 * M, 6 * M), goal]);
    assert_eq!(
        path.points,
        blocked.find_path(start, goal, 0).unwrap().points
    );
    // It is still ground, unlike what a building stands on.
    let middle = p(5 * M, 5 * M);
    assert!(avoided.locate(middle).is_some());
    assert_eq!(blocked.locate(middle), None);
}

#[test]
fn a_route_may_leave_avoided_ground_but_not_end_on_it() {
    let mesh = standing(ObstacleKind::Avoid);
    let (middle, outside) = (p(5 * M, 5 * M), p(9500, 5 * M));
    // The unit standing there walks straight out, or about within it.
    assert_eq!(
        mesh.find_path(middle, outside, 0).unwrap().points,
        [middle, outside]
    );
    assert_eq!(
        mesh.find_path(middle, p(5500, 5200), 0).unwrap().points,
        [middle, p(5500, 5200)]
    );

    // Anyone else ordered onto it stops at its edge.
    let goal = p(5800, 5 * M);
    let path = mesh.find_path(outside, goal, 0).unwrap();
    assert_eq!(path.points, [outside, p(6 * M, 5 * M)]);
    // From the far side that means walking round to the same spot.
    let path = mesh.find_path(p(500, 5 * M), goal, 0).unwrap();
    assert_eq!(path.points.last(), Some(&p(6 * M, 5 * M)));
    assert_eq!(path.points.len(), 4);
    check_route(&mesh, &path, p(500, 5 * M), p(6 * M, 5 * M));

    // A straight line sees it the same way.
    let at = |point: Point| mesh.nearest(point, 0).unwrap();
    assert_eq!(mesh.line_of_sight(at(p(500, 5 * M)), outside), None);
    assert!(mesh.line_of_sight(at(middle), outside).is_some());
}

#[test]
fn a_goal_that_cannot_be_reached_is_approached() {
    let mut mesh = field();
    mesh.add_obstacle(&rect(9, 9, 11, 11), ObstacleKind::Blocked)
        .unwrap();
    // Half a metre north of the middle, so round the north side is shorter.
    let (start, inside) = (p(2 * M, 10_500), p(10_400, 10 * M));
    assert_eq!(
        mesh.find_path(start, inside, 0),
        Err(PathError::GoalOffMesh)
    );
    // The nearest ground is the east wall, 600 mm away and round the back.
    assert_eq!(
        mesh.find_path_toward(start, inside, 500),
        Err(PathError::GoalOffMesh)
    );
    let path = mesh.find_path_toward(start, inside, 600).unwrap();
    // Past two corners of the building to reach it.
    assert_eq!(
        path.points,
        [
            start,
            p(9 * M, 11 * M),
            p(11 * M, 11 * M),
            p(11 * M, 10 * M)
        ]
    );
    // A goal that can be reached is reached as usual.
    let open = p(18 * M, 3 * M);
    assert_eq!(
        mesh.find_path_toward(start, open, 600),
        mesh.find_path(start, open, 0)
    );

    // Across a gap, the nearest point on the start's own side.
    let map = Map::drawn(&["S.#G", "..##"]);
    let mesh = map.navmesh();
    let (start, goal) = (map.at('S'), map.at('G'));
    let nearest = map.corner(2, 1).x;
    let path = mesh.find_path_toward(start, goal, 2 * M).unwrap();
    assert_eq!(path.points, [start, p(nearest, 1500)]);
    assert_eq!(
        mesh.find_path_toward(start, goal, M),
        Err(PathError::GoalOffMesh)
    );
    let from = mesh.nearest(start, 0).unwrap();
    let reached = mesh.nearest_reachable(from, goal, 2 * M).unwrap();
    assert_eq!(reached.point, p(nearest, 1500));
    assert_eq!(mesh.nearest_reachable(from, goal, M), None);
}

/// Random convex shapes scattered over a 20 m by 14 m map.
fn scatter(rng: &mut Rng, count: usize) -> Vec<Vec<Point>> {
    let mut shapes = Vec::new();
    while shapes.len() < count {
        let middle = (rng.below(20_000) as i32, rng.below(14_000) as i32);
        let size = 300 + rng.below(2500);
        let spot = |rng: &mut Rng, c: i32| c - size as i32 + rng.below(2 * size) as i32;
        let spots: Vec<Point> = (0..3 + rng.below(4))
            .map(|_| p(spot(rng, middle.0), spot(rng, middle.1)))
            .collect();
        let hull = convex_hull(&spots);
        if hull.len() >= 3 {
            shapes.push(hull);
        }
    }
    shapes
}

#[test]
fn no_route_passes_through_a_scattered_obstacle() {
    let (mut found, mut refused) = (0, 0);
    for seed in 1..=10_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let pristine = Map::open(20, 14).navmesh();
        let mut mesh = Map::open(20, 14).navmesh();
        let shapes = scatter(&mut rng, 30);
        let mut edit = mesh.edit_obstacles();
        let ids: Vec<ObstacleId> = shapes
            .iter()
            .map(|shape| edit.add(shape, ObstacleKind::Blocked).unwrap())
            .collect();
        drop(edit);

        let anywhere = |rng: &mut Rng| p(rng.below(20_000) as i32, rng.below(14_000) as i32);
        for _ in 0..80 {
            let (start, goal) = (anywhere(&mut rng), anywhere(&mut rng));
            let there = mesh.find_path(start, goal, 3 * M);
            // With nothing one-way on the mesh, what can be walked can be walked back.
            let back = mesh.find_path(goal, start, 3 * M);
            assert_eq!(
                there.is_ok(),
                back.is_ok(),
                "seed {seed}: {start:?} and {goal:?}"
            );
            let Ok(path) = there else {
                refused += 1;
                continue;
            };
            found += 1;
            assert!(path.points.windows(2).all(|leg| leg[0] != leg[1]));
            for step in path.corridor.windows(2) {
                assert!(mesh.neighbors(step[0]).any(|n| n == Some(step[1])));
            }
            for q in samples(&path) {
                assert!(
                    mesh.nearest(q, 2).is_some(),
                    "seed {seed}: route leaves the ground at {q:?}"
                );
                let hit = shapes.iter().position(|shape| deep_inside(shape, q));
                assert_eq!(hit, None, "seed {seed}: route enters an obstacle at {q:?}");
            }
            // An obstacle can only make a route longer.
            let clear = pristine
                .find_path(path.points[0], *path.points.last().unwrap(), 0)
                .unwrap();
            assert!(path.length() + path.points.len() as i64 >= clear.length());
        }

        let mut edit = mesh.edit_obstacles();
        ids.iter().for_each(|&id| assert!(edit.remove(id)));
        drop(edit);
        for _ in 0..40 {
            let (start, goal) = (anywhere(&mut rng), anywhere(&mut rng));
            assert_eq!(
                mesh.find_path(start, goal, 0),
                pristine.find_path(start, goal, 0)
            );
        }
    }
    assert!(
        found > 400 && refused > 0,
        "{found} found, {refused} refused"
    );
}

#[test]
fn mixed_obstacles_keep_routes_out_of_the_blocking_ones() {
    for seed in 11..=16_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let mut mesh = Map::open(20, 14).navmesh();
        let shapes = scatter(&mut rng, 24);
        let kind = |k: usize| {
            if k.is_multiple_of(3) {
                ObstacleKind::Avoid
            } else {
                ObstacleKind::Blocked
            }
        };
        let mut edit = mesh.edit_obstacles();
        for (k, shape) in shapes.iter().enumerate() {
            edit.add(shape, kind(k)).unwrap();
        }
        drop(edit);
        let walls: Vec<&Vec<Point>> = shapes
            .iter()
            .enumerate()
            .filter(|&(k, _)| !k.is_multiple_of(3))
            .map(|(_, shape)| shape)
            .collect();
        for _ in 0..80 {
            let start = p(rng.below(20_000) as i32, rng.below(14_000) as i32);
            let goal = p(rng.below(20_000) as i32, rng.below(14_000) as i32);
            let Ok(path) = mesh.find_path_toward(start, goal, 4 * M) else {
                continue;
            };
            for q in samples(&path) {
                assert!(
                    mesh.nearest(q, 2).is_some(),
                    "seed {seed}: route leaves the ground at {q:?}"
                );
                assert!(
                    !walls.iter().any(|shape| deep_inside(shape, q)),
                    "seed {seed}: {q:?}"
                );
            }
            assert_eq!(mesh.find_path_toward(start, goal, 4 * M).unwrap(), path);
        }
    }
}
