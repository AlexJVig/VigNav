//! End-to-end checks of mesh building, lookup and path queries.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use vignav::geom::dist;
use vignav::{Location, MeshError, NavMesh, Path, PathError, Point, PolyId};

/// One metre.
const M: i32 = 1000;

fn p(x: i32, y: i32) -> Point {
    Point::new(x, y)
}

/// The corners of three 1 m squares in an L: A at the origin, B to its
/// right, C above B. The notch above A is off the mesh.
const L_VERTICES: [Point; 8] = [
    Point::new(0, 0),
    Point::new(M, 0),
    Point::new(2 * M, 0),
    Point::new(0, M),
    Point::new(M, M),
    Point::new(2 * M, M),
    Point::new(M, 2 * M),
    Point::new(2 * M, 2 * M),
];
const L_POLYGONS: [[u32; 4]; 3] = [[0, 1, 4, 3], [1, 2, 5, 4], [4, 5, 7, 6]];
const A: PolyId = PolyId(0);
const B: PolyId = PolyId(1);
const C: PolyId = PolyId(2);

fn l_shape() -> NavMesh {
    NavMesh::from_polygons(&L_VERTICES, &L_POLYGONS).unwrap()
}

fn links(mesh: &NavMesh, poly: PolyId) -> Vec<Option<PolyId>> {
    mesh.neighbors(poly).collect()
}

#[test]
fn shared_edges_become_links() {
    let mesh = l_shape();
    assert_eq!(mesh.polygon_count(), 3);
    assert_eq!(links(&mesh, A), [None, Some(B), None, None]);
    assert_eq!(links(&mesh, B), [None, None, Some(C), Some(A)]);
    assert_eq!(links(&mesh, C), [Some(B), None, None, None]);
    assert_eq!(
        mesh.corners(C).collect::<Vec<_>>(),
        [p(M, M), p(2 * M, M), p(2 * M, 2 * M), p(M, 2 * M)]
    );
    assert!([A, B, C].iter().all(|&poly| mesh.island(poly) == 0));
}

#[test]
fn polygons_link_by_position_without_sharing_vertices() {
    // Two squares, each with its own four vertices.
    let vertices = [
        p(0, 0),
        p(M, 0),
        p(M, M),
        p(0, M),
        p(M, 0),
        p(2 * M, 0),
        p(2 * M, M),
        p(M, M),
    ];
    let mesh = NavMesh::from_polygons(&vertices, &[[0, 1, 2, 3], [4, 5, 6, 7]]).unwrap();
    assert_eq!(links(&mesh, A), [None, Some(B), None, None]);
    assert_eq!(links(&mesh, B), [None, None, None, Some(A)]);
}

#[test]
fn given_links_match_derived_ones() {
    let derived = l_shape();
    let given: Vec<Vec<Option<u32>>> = [A, B, C]
        .iter()
        .map(|&poly| derived.neighbors(poly).map(|n| n.map(|id| id.0)).collect())
        .collect();
    let mesh = NavMesh::from_linked_polygons(&L_VERTICES, &L_POLYGONS, &given).unwrap();
    for poly in [A, B, C] {
        assert_eq!(links(&mesh, poly), links(&derived, poly));
    }
    let (start, goal) = (p(500, 900), p(1100, 1900));
    assert_eq!(
        mesh.find_path(start, goal, 0),
        derived.find_path(start, goal, 0)
    );
}

#[test]
fn given_links_must_be_answered() {
    let none = [None; 4];
    // A claims C across its top edge, but C does not touch A.
    let one_sided = [
        [None, Some(1), Some(2), None],
        [None, None, Some(2), Some(0)],
        [Some(1), None, None, None],
    ];
    assert_eq!(
        NavMesh::from_linked_polygons(&L_VERTICES, &L_POLYGONS, &one_sided).unwrap_err(),
        MeshError::BadLink {
            polygon: 0,
            edge: 2
        }
    );
    let missing = [[None, Some(9), None, None], none, none];
    assert_eq!(
        NavMesh::from_linked_polygons(&L_VERTICES, &L_POLYGONS, &missing).unwrap_err(),
        MeshError::BadLink {
            polygon: 0,
            edge: 1
        }
    );
    assert_eq!(
        NavMesh::from_linked_polygons(&L_VERTICES, &L_POLYGONS, &[none, none]).unwrap_err(),
        MeshError::LinkCountMismatch { polygon: 2 }
    );
}

#[test]
fn bad_polygons_are_rejected() {
    let build = |vertices: &[Point], polygon: &[u32]| {
        NavMesh::from_polygons(vertices, &[polygon]).unwrap_err()
    };
    let v = L_VERTICES;
    assert_eq!(
        build(&v, &[0, 3, 4, 1]),
        MeshError::NotCounterClockwise { polygon: 0 }
    );
    assert_eq!(build(&v, &[0, 1]), MeshError::TooFewCorners { polygon: 0 });
    assert_eq!(
        build(&v, &[0, 1, 99]),
        MeshError::VertexOutOfRange {
            polygon: 0,
            corner: 2
        }
    );
    assert_eq!(
        build(&v, &[0, 0, 1, 4, 3]),
        MeshError::ZeroLengthEdge {
            polygon: 0,
            corner: 1
        }
    );
    assert_eq!(
        build(&[p(0, 0), p(1 << 30, 0), p(0, M)], &[0, 1, 2]),
        MeshError::CoordinateOutOfRange { vertex: 1 }
    );
    // A spike: out along the bottom edge and straight back.
    assert_eq!(
        build(
            &[p(0, 0), p(2 * M, 0), p(M, 0), p(M, M), p(0, M)],
            &[0, 1, 2, 3, 4]
        ),
        MeshError::NotConvex {
            polygon: 0,
            corner: 1
        }
    );
    // The same polygon twice runs along every edge in the same direction.
    assert!(matches!(
        NavMesh::from_polygons(&v, &[L_POLYGONS[0], L_POLYGONS[0]]).unwrap_err(),
        MeshError::EdgeConflict { polygon: 0, .. }
    ));
}

#[test]
fn a_dent_within_the_slack_is_accepted() {
    // A square with an extra corner on its bottom edge, pushed inwards.
    let dented = |by: i32| {
        let vertices = [p(0, 0), p(500, by), p(M, 0), p(M, M), p(0, M)];
        NavMesh::from_polygons(&vertices, &[[0, 1, 2, 3, 4]])
    };
    assert!(dented(0).is_ok());
    assert!(dented(2).is_ok());
    assert_eq!(
        dented(3).unwrap_err(),
        MeshError::NotConvex {
            polygon: 0,
            corner: 1
        }
    );
    assert!(dented(-300).is_ok());
}

#[test]
fn points_are_located_and_snapped() {
    let mesh = l_shape();
    assert_eq!(mesh.locate(p(500, 500)), Some(A));
    assert_eq!(mesh.locate(p(1500, 1500)), Some(C));
    // On a shared edge the lowest id wins.
    assert_eq!(mesh.locate(p(M, 500)), Some(A));
    assert_eq!(mesh.locate(p(2 * M, 2 * M)), Some(C));
    // The notch, and well outside.
    assert_eq!(mesh.locate(p(500, 1500)), None);
    assert_eq!(mesh.locate(p(-1, 500)), None);
    assert_eq!(mesh.locate(p(i32::MAX, i32::MIN)), None);

    let inside = p(1500, 500);
    assert_eq!(
        mesh.nearest(inside, 0),
        Some(Location {
            poly: B,
            point: inside
        })
    );
    // The notch centre is 500 mm from A's top edge and from C's left edge;
    // the tie goes to the lower id.
    let notch = p(500, 1500);
    assert_eq!(
        mesh.nearest(notch, 500),
        Some(Location {
            poly: A,
            point: p(500, M)
        })
    );
    assert_eq!(mesh.nearest(notch, 499), None);
    assert_eq!(
        mesh.nearest(p(2300, 2400), 600),
        Some(Location {
            poly: C,
            point: p(2 * M, 2 * M)
        })
    );
    assert_eq!(mesh.nearest(p(i32::MAX, 0), i32::MAX), None);
}

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

/// A small fixed-seed generator, so the random maps are the same every run.
struct Rng(u64);

impl Rng {
    fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 33) % n as u64) as usize
    }
}

/// A grid of 1 m cells, some of them blocked.
struct Map {
    width: usize,
    height: usize,
    blocked: Vec<bool>,
}

impl Map {
    fn random(rng: &mut Rng, width: usize, height: usize, blocked_percent: usize) -> Self {
        let blocked = (0..width * height)
            .map(|_| rng.below(100) < blocked_percent)
            .collect();
        Self {
            width,
            height,
            blocked,
        }
    }

    fn centre(&self, cell: usize) -> Point {
        p(
            (cell % self.width) as i32 * M + 500,
            (cell / self.width) as i32 * M + 500,
        )
    }

    /// One polygon per horizontal run of free cells, with a corner at every
    /// cell boundary so that runs in neighbouring rows share whole edges.
    /// Also returns the polygon of each cell.
    fn mesh(&self) -> (NavMesh, Vec<Option<PolyId>>) {
        let vertex = |x: usize, y: usize| (y * (self.width + 1) + x) as u32;
        let vertices: Vec<Point> = (0..(self.width + 1) * (self.height + 1))
            .map(|i| {
                p(
                    (i % (self.width + 1)) as i32 * M,
                    (i / (self.width + 1)) as i32 * M,
                )
            })
            .collect();
        let mut polygons: Vec<Vec<u32>> = Vec::new();
        let mut cell_poly = vec![None; self.width * self.height];
        for y in 0..self.height {
            let mut x = 0;
            while x < self.width {
                let from = x;
                while x < self.width && !self.blocked[y * self.width + x] {
                    cell_poly[y * self.width + x] = Some(PolyId(polygons.len() as u32));
                    x += 1;
                }
                if x > from {
                    let bottom = (from..=x).map(|gx| vertex(gx, y));
                    let top = (from..=x).rev().map(|gx| vertex(gx, y + 1));
                    polygons.push(bottom.chain(top).collect());
                } else {
                    x += 1;
                }
            }
        }
        (
            NavMesh::from_polygons(&vertices, &polygons).unwrap(),
            cell_poly,
        )
    }

    /// Labels each free cell with its 4-connected region.
    fn regions(&self) -> Vec<usize> {
        let mut region = vec![usize::MAX; self.blocked.len()];
        for seed in 0..self.blocked.len() {
            if self.blocked[seed] || region[seed] != usize::MAX {
                continue;
            }
            region[seed] = seed;
            let mut pending = vec![seed];
            while let Some(cell) = pending.pop() {
                let (x, y) = (cell % self.width, cell / self.width);
                let around = [
                    (x > 0).then(|| cell - 1),
                    (x + 1 < self.width).then(|| cell + 1),
                    (y > 0).then(|| cell - self.width),
                    (y + 1 < self.height).then(|| cell + self.width),
                ];
                for next in around.into_iter().flatten() {
                    if !self.blocked[next] && region[next] == usize::MAX {
                        region[next] = seed;
                        pending.push(next);
                    }
                }
            }
        }
        region
    }

    /// Length of the shortest route between two cell centres that moves
    /// from centre to centre among the eight neighbours, cutting a corner
    /// only where both cells beside the diagonal are free. It is a real
    /// route, so the shortest route of all is no longer than this.
    fn octile(&self, from: usize, to: usize) -> i64 {
        let (width, height) = (self.width as i64, self.height as i64);
        let free = |x: i64, y: i64| {
            (0..width).contains(&x)
                && (0..height).contains(&y)
                && !self.blocked[(y * width + x) as usize]
        };
        let mut best = vec![i64::MAX; self.blocked.len()];
        let mut open = BinaryHeap::from([Reverse((0, from))]);
        best[from] = 0;
        while let Some(Reverse((cost, cell))) = open.pop() {
            if cell == to {
                return cost;
            }
            let (x, y) = (cell as i64 % width, cell as i64 / width);
            for (dx, dy) in [
                (-1, -1),
                (0, -1),
                (1, -1),
                (-1, 0),
                (1, 0),
                (-1, 1),
                (0, 1),
                (1, 1),
            ] {
                if !(free(x + dx, y + dy) && free(x + dx, y) && free(x, y + dy)) {
                    continue;
                }
                let next = ((y + dy) * width + x + dx) as usize;
                // A diagonal is 1414.2 mm; round it up to stay an upper bound.
                let reached = cost + if dx != 0 && dy != 0 { 1415 } else { 1000 };
                if reached < best[next] {
                    best[next] = reached;
                    open.push(Reverse((reached, next)));
                }
            }
        }
        panic!("cell {to} cannot be reached from cell {from}");
    }
}

/// Checks everything a route must satisfy whatever the map.
fn check_path(mesh: &NavMesh, path: &Path, start: Point, goal: Point) {
    assert_eq!(path.points.first(), Some(&start));
    assert_eq!(path.points.last(), Some(&goal));
    assert!(path.points.windows(2).all(|leg| leg[0] != leg[1]));
    assert!(mesh.contains(path.corridor[0], start));
    assert!(mesh.contains(path.corridor[path.corridor.len() - 1], goal));
    for step in path.corridor.windows(2) {
        assert!(
            mesh.neighbors(step[0]).any(|n| n == Some(step[1])),
            "corridor breaks at {step:?}"
        );
    }
    // Every turn is at a mesh vertex, which on these maps is a whole metre.
    let turns = path.points.len().saturating_sub(2);
    for turn in path.points.iter().skip(1).take(turns) {
        assert!(
            turn.x % M == 0 && turn.y % M == 0,
            "turn at {turn:?} is not a vertex"
        );
    }
    // The route stays on the mesh. Samples are rounded to the millimetre,
    // so allow them that much.
    for leg in path.points.windows(2) {
        let steps = (dist(leg[0], leg[1]) / 50).max(1);
        for k in 0..=steps {
            let along = |a: i32, b: i32| a + (i64::from(b - a) * k / steps) as i32;
            let sample = p(along(leg[0].x, leg[1].x), along(leg[0].y, leg[1].y));
            assert!(
                mesh.nearest(sample, 2).is_some(),
                "route leaves the mesh at {sample:?}"
            );
        }
    }
    // Nothing beats the straight line. Each leg is rounded down, hence the
    // allowance of a millimetre per point.
    assert!(path.length() + path.points.len() as i64 >= dist(start, goal));
}

#[test]
fn an_open_map_gives_straight_paths() {
    let map = Map {
        width: 20,
        height: 14,
        blocked: vec![false; 20 * 14],
    };
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
