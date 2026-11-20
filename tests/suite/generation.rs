//! Generating a mesh from a grid of terrain cells.

use super::support::*;

/// Open ground of 1 m cells with its corner at the origin.
fn open(width: usize, height: usize) -> Terrain {
    Terrain::new(p(0, 0), M, width, height).unwrap()
}

fn corners(mesh: &NavMesh, polygon: u32) -> Vec<Point> {
    mesh.corners(PolyId(polygon)).collect()
}

/// The ground a mesh covers, in square millimetres.
fn area(mesh: &NavMesh) -> i64 {
    let doubled = |polygon: u32| -> i64 {
        let c = corners(mesh, polygon);
        (1..c.len() - 1).map(|i| cross(c[0], c[i], c[i + 1])).sum()
    };
    (0..mesh.polygon_count() as u32).map(doubled).sum::<i64>() / 2
}

/// How far `q` is from the nearest blocked cell of a map and from the
/// outside of the map, whichever is nearer.
fn clearance(map: &Map, q: Point) -> f64 {
    let (x, y) = (f64::from(q.x - map.origin.x), f64::from(q.y - map.origin.y));
    let (wide, high) = (
        (map.width as i32 * M) as f64,
        (map.height as i32 * M) as f64,
    );
    let mut nearest = x.min(wide - x).min(y).min(high - y);
    for cell in (0..map.blocked.len()).filter(|&cell| map.blocked[cell]) {
        let (x0, y0) = (
            ((cell % map.width) as i32 * M) as f64,
            ((cell / map.width) as i32 * M) as f64,
        );
        let off = |low: f64, at: f64| (low - at).max(at - low - f64::from(M)).max(0.0);
        nearest = nearest.min(off(x0, x).hypot(off(y0, y)));
    }
    nearest
}

#[test]
fn open_ground_is_one_rectangle() {
    let terrain = Terrain::new(p(-5 * M, 3 * M), M, 8, 6).unwrap();
    assert_eq!(
        (terrain.width(), terrain.height(), terrain.cell()),
        (8, 6, M)
    );
    assert_eq!(terrain.origin(), p(-5 * M, 3 * M));
    let whole = [
        p(-5 * M, 3 * M),
        p(3 * M, 3 * M),
        p(3 * M, 9 * M),
        p(-5 * M, 9 * M),
    ];
    // With nothing to keep clear of, the lattice makes no difference.
    for resolution in [M, 500, 250, 100] {
        let mesh = terrain.generate(&Layer::new(0, resolution)).unwrap();
        assert_eq!(mesh.polygon_count(), 1);
        assert_eq!(corners(&mesh, 0), whole);
    }
}

#[test]
fn the_radius_is_kept_clear_of_the_map_edge() {
    let terrain = open(8, 6);
    let inset = |by: i32| {
        vec![
            p(by, by),
            p(8 * M - by, by),
            p(8 * M - by, 6 * M - by),
            p(by, 6 * M - by),
        ]
    };
    let ground = |radius: i32, resolution: i32| {
        let mesh = terrain.generate(&Layer::new(radius, resolution)).unwrap();
        assert_eq!(mesh.polygon_count(), 1);
        corners(&mesh, 0)
    };
    assert_eq!(ground(750, 250), inset(750));
    // The radius is rounded up to the lattice, never down.
    assert_eq!(ground(700, 250), inset(750));
    assert_eq!(ground(700, 500), inset(M));
    assert_eq!(ground(1, M), inset(M));
    // A unit as wide as the map has nowhere to stand.
    assert_eq!(
        terrain
            .generate(&Layer::new(3 * M, 500))
            .unwrap()
            .polygon_count(),
        0
    );
}

#[test]
fn a_blocked_cell_is_cut_round() {
    let mut terrain = open(5, 5);
    terrain.set_blocked(2, 2, true);
    assert!(terrain.is_blocked(2, 2) && !terrain.is_blocked(2, 3));
    // Anything outside the terrain counts as blocked.
    assert!(terrain.is_blocked(5, 0) && terrain.is_blocked(0, 5));

    let mesh = terrain.generate(&Layer::new(0, M)).unwrap();
    // Below it, left of it, right of it and above it.
    assert_eq!(mesh.polygon_count(), 4);
    assert_eq!(area(&mesh), 24 * 1_000_000);
    assert_eq!(mesh.locate(p(2500, 2500)), None);
    assert!(mesh.locate(p(2 * M, 2500)).is_some());

    let (start, goal) = (p(500, 2600), p(4500, 2600));
    let path = mesh.find_path(start, goal, 0).unwrap();
    assert_eq!(path.points, [start, p(2 * M, 3 * M), p(3 * M, 3 * M), goal]);
    check_path(&mesh, &path, start, goal);

    // Opening the cell again gives back the open ground.
    terrain.set_blocked(2, 2, false);
    assert_eq!(
        terrain.generate(&Layer::new(0, M)).unwrap().polygon_count(),
        1
    );
}

#[test]
fn clearance_cuts_corners_at_45_degrees() {
    let mut terrain = open(9, 9);
    terrain.set_blocked(4, 4, true);
    let mesh = terrain.generate(&Layer::new(M, 500)).unwrap();
    // One below, one above, and on each side one beside each cut corner and
    // one between them.
    assert_eq!(mesh.polygon_count(), 8);
    // A 7 m square inside the edge clearance, less a 3 m square with half a
    // metre cut off each corner.
    assert_eq!(area(&mesh), 49_000_000 - (9_000_000 - 500_000));
    // The cut by the cell's far corner runs from (6 m, 5.5 m) to (5.5 m, 6 m).
    assert!(mesh.locate(p(6000, 5500)).is_some());
    assert!(mesh.locate(p(5750, 5750)).is_some());
    assert_eq!(mesh.locate(p(5700, 5700)), None);
    assert!(mesh.locate(p(5800, 5800)).is_some());
    // Straight out from a side it is a full metre.
    assert_eq!(mesh.locate(p(5999, 4500)), None);
    assert!(mesh.locate(p(6000, 4500)).is_some());
}

#[test]
fn a_gap_must_be_wider_than_the_unit() {
    // A wall across the middle of the map with a gap in it.
    let walled = |gap: usize| {
        let mut terrain = open(11, 9);
        (0..11)
            .filter(|x| !(4..4 + gap).contains(x))
            .for_each(|x| terrain.set_blocked(x, 4, true));
        terrain.generate(&Layer::new(M, 500)).unwrap()
    };
    let (south, north) = (p(5500, 2 * M), p(5500, 7 * M));
    // A unit 2 m across does not fit a 2 m gap: it would scrape both sides.
    assert_eq!(
        walled(2).find_path(p(5 * M, 2 * M), p(5 * M, 7 * M), 0),
        Err(PathError::Unreachable)
    );
    // It fits a 3 m gap, down the middle.
    assert_eq!(
        walled(3).find_path(south, north, 0).unwrap().points,
        [south, north]
    );
    // With no clearance asked for, even a 1 m gap is open.
    let mut terrain = open(11, 9);
    (0..11)
        .filter(|&x| x != 5)
        .for_each(|x| terrain.set_blocked(x, 4, true));
    let mesh = terrain.generate(&Layer::new(0, M)).unwrap();
    assert_eq!(
        mesh.find_path(south, north, 0).unwrap().points,
        [south, north]
    );
}

#[test]
fn generated_ground_matches_the_same_map_built_by_hand() {
    let (mut reachable, mut unreachable) = (0, 0);
    for seed in 1..=8_u64 {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let map = Map::random(&mut rng, 20, 14, 20 + 2 * seed as usize);
        let mesh = map.terrain().generate(&Layer::new(0, M)).unwrap();
        let regions = map.regions();
        let free: Vec<usize> = (0..map.blocked.len())
            .filter(|&cell| !map.blocked[cell])
            .collect();
        assert_eq!(area(&mesh), free.len() as i64 * 1_000_000);
        // Far fewer polygons than cells, and no more than the hand-built mesh.
        assert!(mesh.polygon_count() <= map.navmesh().polygon_count());
        for _ in 0..60 {
            let (from, to) = (free[rng.below(free.len())], free[rng.below(free.len())]);
            let (start, goal) = (map.centre(from), map.centre(to));
            assert_eq!(mesh.locate_all(start).len(), 1);
            let found = mesh.find_path(start, goal, 0);
            if regions[from] != regions[to] {
                assert_eq!(found, Err(PathError::Unreachable));
                unreachable += 1;
                continue;
            }
            let path = found.unwrap();
            check_path(&mesh, &path, start, goal);
            let octile = map.octile(from, to);
            assert!(
                path.length() <= octile,
                "seed {seed}: {start:?} to {goal:?}"
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
fn random_ground_keeps_the_radius_clear_and_little_more() {
    // The promise: the radius is always kept clear, and ground is only left
    // out if it is nearer than 1.09 x radius + 2 x resolution to something.
    // Tried with a fine lattice, with a radius large enough for the cut
    // corners to matter, and with a lattice as coarse as the radius or
    // coarser, where rounding is at its worst.
    let layers = [
        (700, 250),
        (2000, 250),
        (1500, 500),
        (1000, M),
        (750, M),
        (700, M),
        (300, 500),
        (0, 500),
    ];
    let (mut on, mut off) = (0, 0);
    for (radius, resolution) in layers {
        let layer = Layer::new(radius, resolution);
        let left_out_within = 1.09 * f64::from(radius) + 2.0 * f64::from(resolution);
        for seed in 1..=3_u64 {
            let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
            let map = Map::random(&mut rng, 18, 14, 3 + seed as usize);
            let mesh = map.terrain().generate(&layer).unwrap();
            // Sample off the lattice, so that no sample is on an edge.
            for y in (37..14 * M).step_by(130) {
                for x in (61..18 * M).step_by(130) {
                    let q = p(x, y);
                    let clear = clearance(&map, q);
                    let polygons = mesh.locate_all(q).len();
                    assert!(
                        polygons <= 1,
                        "{layer:?} seed {seed}: polygons overlap at {q:?}"
                    );
                    if polygons == 1 {
                        assert!(
                            clear >= f64::from(radius),
                            "{layer:?} seed {seed}: {q:?} is only {clear} mm clear"
                        );
                        on += 1;
                    } else {
                        assert!(
                            clear < left_out_within,
                            "{layer:?} seed {seed}: {q:?} is {clear} mm clear and left out"
                        );
                        off += 1;
                    }
                }
            }
            // Routes inherit the clearance, less the millimetre a sample is rounded by.
            for _ in 0..30 {
                let (start, goal) = (
                    map.centre(rng.below(18 * 14)),
                    map.centre(rng.below(18 * 14)),
                );
                let Ok(path) = mesh.find_path(start, goal, 3 * M) else {
                    continue;
                };
                check_route(&mesh, &path, path.points[0], *path.points.last().unwrap());
                for q in samples(&path) {
                    assert!(
                        clearance(&map, q) >= f64::from(radius) - 2.0,
                        "{layer:?} seed {seed}: route at {q:?}"
                    );
                }
            }
        }
    }
    assert!(
        on > 50_000 && off > 50_000,
        "{on} samples on the mesh, {off} off it"
    );
}

#[test]
fn a_large_radius_has_its_corners_cut_where_the_octagon_says() {
    // 2 m of clearance on a 250 mm lattice: the cut starts 1 m along each
    // side of the grown cell, so 1 m is taken off each corner.
    let mut terrain = open(15, 15);
    terrain.set_blocked(7, 7, true);
    let mesh = terrain.generate(&Layer::new(2 * M, 250)).unwrap();
    assert_eq!(mesh.polygon_count(), 8);
    // An 11 m square inside the edge clearance, less a 5 m square with four
    // 1 m corners cut off.
    assert_eq!(area(&mesh), 121_000_000 - (25_000_000 - 2_000_000));
    // The cut by the cell's far corner (8 m, 8 m) runs from (10 m, 9 m) to (9 m, 10 m).
    assert!(mesh.locate(p(10 * M, 9 * M)).is_some());
    assert!(mesh.locate(p(9500, 9500)).is_some());
    assert_eq!(mesh.locate(p(9400, 9400)), None);
    // The corner of the uncut square would have been 2.83 m off; this is 2.12 m.
    assert!(mesh.locate(p(9600, 9600)).is_some());
}

#[test]
fn a_radius_of_one_lattice_step_is_still_kept_clear() {
    // A lattice as coarse as the unit: 1 m of clearance on a 1 m lattice.
    // One step would leave the corner of the blocked cell 0.71 m away
    // along the diagonal, so two are kept.
    let mut terrain = open(13, 13);
    terrain.set_blocked(6, 6, true);
    let mesh = terrain.generate(&Layer::new(M, M)).unwrap();
    assert_eq!(mesh.locate(p(7500, 7500)), None);
    assert_eq!(mesh.locate(p(7900, 6500)), None);
    assert_eq!(mesh.locate(p(8900, 6500)), None);
    assert!(mesh.locate(p(9 * M, 6500)).is_some());
    // Its corner is cut one step in, 2.12 m from the cell's corner.
    assert!(mesh.locate(p(8500, 8500)).is_some());
    assert_eq!(mesh.locate(p(8400, 8400)), None);

    // A radius the single step does cover keeps just the one.
    let mesh = terrain.generate(&Layer::new(700, M)).unwrap();
    assert!(mesh.locate(p(8 * M, 6500)).is_some());
    assert_eq!(mesh.locate(p(7900, 6500)), None);
    assert!(mesh.locate(p(7500, 7500)).is_some());
    assert_eq!(mesh.locate(p(7400, 7400)), None);
}

#[test]
fn steep_ground_is_blocked() {
    let mut terrain = open(4, 3);
    // A gentle slope with a 2 m cliff between the second and third columns
    // of corners.
    let heights: Vec<i32> = (0..20)
        .map(|i| (i % 5) * 100 + if i % 5 >= 2 { 2 * M } else { 0 })
        .collect();
    assert_eq!(terrain.block_steep(&heights, 500), Ok(3));
    for y in 0..3 {
        assert!(terrain.is_blocked(1, y));
        assert!(!terrain.is_blocked(0, y) && !terrain.is_blocked(2, y));
    }
    // Asking again blocks nothing new, and the gentle slope passes any limit above it.
    assert_eq!(terrain.block_steep(&heights, 500), Ok(0));
    assert_eq!(open(4, 3).block_steep(&heights, 99), Ok(12));
    assert_eq!(open(4, 3).block_steep(&heights, 100), Ok(3));
    assert_eq!(
        terrain.block_steep(&heights[1..], 500),
        Err(GenerateError::HeightCount {
            expected: 20,
            given: 19
        })
    );

    // The cliff leaves two separate stretches of ground.
    let mesh = terrain.generate(&Layer::new(0, M)).unwrap();
    assert_eq!(mesh.polygon_count(), 2);
    assert_eq!(
        mesh.find_path(p(500, 500), p(3500, 500), 0),
        Err(PathError::Unreachable)
    );
}

#[test]
fn a_footprint_blocks_every_cell_it_covers() {
    let mut terrain = open(6, 6);
    // A 3 m square on its corner, centred where four cells meet.
    let diamond = [
        p(4500, 3 * M),
        p(3 * M, 4500),
        p(1500, 3 * M),
        p(3 * M, 1500),
    ];
    assert_eq!(terrain.block_shape(&diamond), 12);
    for (x, y) in [(2, 2), (3, 3), (1, 2), (4, 3), (2, 1), (3, 4)] {
        assert!(terrain.is_blocked(x, y), "cell ({x}, {y})");
    }
    for (x, y) in [(1, 1), (4, 4), (1, 4), (4, 1), (0, 2), (2, 0)] {
        assert!(!terrain.is_blocked(x, y), "cell ({x}, {y})");
    }
    // The corners may come in any order, and nothing is blocked twice.
    assert_eq!(
        terrain.block_shape(&[diamond[2], diamond[0], diamond[3], diamond[1]]),
        0
    );

    // A shape that only touches a cell along an edge leaves it open.
    assert_eq!(terrain.block_shape(&rect(0, 0, 1, 1)), 1);
    assert!(terrain.is_blocked(0, 0) && !terrain.is_blocked(1, 0) && !terrain.is_blocked(0, 1));
    // So does one with no area, and one off the map blocks nothing.
    assert_eq!(terrain.block_shape(&[p(0, 5 * M), p(M, 6 * M)]), 0);
    assert_eq!(terrain.block_shape(&rect(10, 10, 12, 12)), 0);
    assert_eq!(terrain.block_shape(&rect(-3, -3, 0, 0)), 0);
    // One hanging over the edge blocks what it covers of the map.
    assert_eq!(terrain.block_shape(&rect(5, 5, 9, 9)), 1);
    assert!(terrain.is_blocked(5, 5));
}

#[test]
fn pockets_too_small_to_matter_are_left_out() {
    // The top right cell is walled off from everything else.
    let map = Map::drawn(&["....#.", "....##", "......", "......"]);
    let terrain = map.terrain();
    let pocket = p(5500, 3500);
    let all = terrain.generate(&Layer::new(0, M)).unwrap();
    assert!(all.locate(pocket).is_some());

    let at_least = |min_island: i64| {
        terrain
            .generate(&Layer {
                min_island,
                ..Layer::new(0, M)
            })
            .unwrap()
    };
    let kept = at_least(2_000_000);
    assert_eq!(kept.locate(pocket), None);
    assert_eq!(area(&kept), area(&all) - 1_000_000);
    assert_eq!(kept.polygon_count(), all.polygon_count() - 1);
    // A pocket exactly as large as the limit stays.
    assert!(at_least(1_000_000).locate(pocket).is_some());
    // And a limit larger than everything leaves nothing.
    assert_eq!(at_least(100_000_000).polygon_count(), 0);
}

#[test]
fn bad_input_is_refused() {
    for (cell, width, height) in [(0, 4, 4), (-M, 4, 4), (M, 0, 4), (M, 4, 0)] {
        assert_eq!(
            Terrain::new(p(0, 0), cell, width, height),
            Err(GenerateError::NoCells)
        );
    }
    let edge = COORD_LIMIT - 4 * M;
    assert!(Terrain::new(p(edge, -COORD_LIMIT), M, 4, 4).is_ok());
    assert_eq!(
        Terrain::new(p(edge + 1, 0), M, 4, 4),
        Err(GenerateError::OutOfRange)
    );
    assert_eq!(
        Terrain::new(p(0, -COORD_LIMIT - 1), M, 4, 4),
        Err(GenerateError::OutOfRange)
    );

    let terrain = open(4, 4);
    let refused = |layer: Layer| terrain.generate(&layer).unwrap_err();
    assert_eq!(refused(Layer::new(-1, M)), GenerateError::NegativeRadius);
    for resolution in [0, -250, 300, 2 * M] {
        assert_eq!(
            refused(Layer::new(0, resolution)),
            GenerateError::BadResolution {
                cell: M,
                resolution
            }
        );
    }
    // A hundred kilometres of ground drawn a millimetre at a time.
    let vast = Terrain::new(p(0, 0), 1_000_000, 100, 100).unwrap();
    assert_eq!(
        vast.generate(&Layer::new(0, 1)).unwrap_err(),
        GenerateError::TooFine
    );

    assert_eq!(
        refused(Layer::new(0, 300)).to_string(),
        "a cell of 1000 mm is not a whole number of 300 mm steps"
    );
    let boxed: Box<dyn std::error::Error> = Box::new(GenerateError::NoCells);
    assert!(!boxed.to_string().is_empty());
}

#[test]
#[should_panic(expected = "outside the terrain")]
fn a_cell_off_the_terrain_cannot_be_set() {
    open(4, 4).set_blocked(4, 0, true);
}

#[test]
fn ground_with_nothing_open_gives_an_empty_mesh() {
    let mut terrain = open(3, 3);
    (0..9).for_each(|cell| terrain.set_blocked(cell % 3, cell / 3, true));
    let mesh = terrain.generate(&Layer::new(0, 500)).unwrap();
    assert_eq!(mesh.polygon_count(), 0);
    assert_eq!(
        mesh.find_path(p(500, 500), p(2500, 2500), M),
        Err(PathError::StartOffMesh)
    );
    // The same once clearance has taken the little that was open.
    terrain.set_blocked(1, 1, false);
    assert_eq!(
        terrain
            .generate(&Layer::new(0, 500))
            .unwrap()
            .polygon_count(),
        1
    );
    assert_eq!(
        terrain
            .generate(&Layer::new(500, 500))
            .unwrap()
            .polygon_count(),
        0
    );
}

#[test]
fn moving_the_terrain_moves_the_mesh_and_nothing_else() {
    let offset = p(-123_000, 98_750);
    for seed in 1..=4_u64 {
        let seeded = || Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let here = Map::random(&mut seeded(), 16, 12, 20).terrain();
        let there = Map::random(&mut seeded(), 16, 12, 20)
            .moved_to(offset)
            .terrain();
        let layer = Layer::new(600, 250);
        let (near, far) = (
            here.generate(&layer).unwrap(),
            there.generate(&layer).unwrap(),
        );
        assert_eq!(near.polygon_count(), far.polygon_count());
        for polygon in 0..near.polygon_count() as u32 {
            let moved: Vec<Point> = corners(&near, polygon)
                .iter()
                .map(|q| p(q.x + offset.x, q.y + offset.y))
                .collect();
            assert_eq!(corners(&far, polygon), moved);
            assert_eq!(near.island(PolyId(polygon)), far.island(PolyId(polygon)));
        }
    }
}

#[test]
fn generating_twice_gives_the_same_mesh() {
    let mut rng = Rng(7);
    let terrain = Map::random(&mut rng, 20, 14, 25).terrain();
    let layer = Layer::new(400, 200);
    let (first, second) = (
        terrain.generate(&layer).unwrap(),
        terrain.generate(&layer).unwrap(),
    );
    assert_eq!(first.polygon_count(), second.polygon_count());
    for polygon in 0..first.polygon_count() as u32 {
        assert_eq!(corners(&first, polygon), corners(&second, polygon));
        let links = |mesh: &NavMesh| mesh.neighbors(PolyId(polygon)).collect::<Vec<_>>();
        assert_eq!(links(&first), links(&second));
    }
}

#[test]
fn obstacles_go_on_generated_ground_like_any_other() {
    let mut mesh = open(20, 20).generate(&Layer::new(0, M)).unwrap();
    let (start, goal) = (p(2 * M, 10_500), p(18 * M, 10_500));
    assert_eq!(
        mesh.find_path(start, goal, 0).unwrap().points,
        [start, goal]
    );
    let id = mesh
        .add_obstacle(&rect(9, 9, 11, 11), ObstacleKind::Blocked)
        .unwrap();
    let path = mesh.find_path(start, goal, 0).unwrap();
    assert_eq!(
        path.points,
        [start, p(9 * M, 11 * M), p(11 * M, 11 * M), goal]
    );
    mesh.remove_obstacle(id);
    assert_eq!(
        mesh.find_path(start, goal, 0).unwrap().points,
        [start, goal]
    );
}
