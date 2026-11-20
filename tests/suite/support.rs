//! Shared fixtures: the L-shaped mesh, maps drawn as text, random maps and
//! the checks every route must pass.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub use vignav::geom::{COORD_LIMIT, convex_hull, cross, dist, dist_sq, grow_convex};
pub use vignav::{
    GenerateError, Layer, Location, MeshError, NavMesh, ObstacleId, ObstacleKind, Path, PathError,
    Point, PolyId, Terrain,
};

/// One metre.
pub const M: i32 = 1000;

pub fn p(x: i32, y: i32) -> Point {
    Point::new(x, y)
}

/// The corners of three 1 m squares in an L: A at the origin, B to its
/// right, C above B. The notch above A is off the mesh.
pub const L_VERTICES: [Point; 8] = [
    Point::new(0, 0),
    Point::new(M, 0),
    Point::new(2 * M, 0),
    Point::new(0, M),
    Point::new(M, M),
    Point::new(2 * M, M),
    Point::new(M, 2 * M),
    Point::new(2 * M, 2 * M),
];
pub const L_POLYGONS: [[u32; 4]; 3] = [[0, 1, 4, 3], [1, 2, 5, 4], [4, 5, 7, 6]];
pub const A: PolyId = PolyId(0);
pub const B: PolyId = PolyId(1);
pub const C: PolyId = PolyId(2);

pub fn l_shape() -> NavMesh {
    NavMesh::from_polygons(&L_VERTICES, &L_POLYGONS).unwrap()
}

pub fn links(mesh: &NavMesh, poly: PolyId) -> Vec<Option<PolyId>> {
    mesh.neighbors(poly).collect()
}

/// A 20 m square of open ground as a single polygon.
pub fn field() -> NavMesh {
    let corners = [p(0, 0), p(20 * M, 0), p(20 * M, 20 * M), p(0, 20 * M)];
    NavMesh::from_polygons(&corners, &[[0, 1, 2, 3]]).unwrap()
}

/// An axis-aligned rectangle, counter-clockwise, from metres.
pub fn rect(x0: i32, y0: i32, x1: i32, y1: i32) -> [Point; 4] {
    [
        p(x0 * M, y0 * M),
        p(x1 * M, y0 * M),
        p(x1 * M, y1 * M),
        p(x0 * M, y1 * M),
    ]
}

/// Whether `q` is more than 2 mm inside a convex shape: further in than
/// rounding can account for.
pub fn deep_inside(shape: &[Point], q: Point) -> bool {
    (0..shape.len()).all(|i| {
        let (a, b) = (shape[i], shape[(i + 1) % shape.len()]);
        cross(a, b, q) as f64 > 2.0 * (dist_sq(a, b) as f64).sqrt()
    })
}

/// The points of a route every 50 mm along it.
pub fn samples(path: &Path) -> Vec<Point> {
    let mut points = Vec::new();
    for leg in path.points.windows(2) {
        let steps = (dist(leg[0], leg[1]) / 50).max(1);
        for k in 0..=steps {
            let along = |a: i32, b: i32| a + (i64::from(b - a) * k / steps) as i32;
            points.push(p(along(leg[0].x, leg[1].x), along(leg[0].y, leg[1].y)));
        }
    }
    points
}

/// A small fixed-seed generator, so the random maps are the same every run.
pub struct Rng(pub u64);

impl Rng {
    pub fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 33) % n as u64) as usize
    }
}

/// A grid of 1 m cells, some of them blocked. Cell 0 is the bottom-left
/// one and cells count along each row, then up.
pub struct Map {
    pub width: usize,
    pub height: usize,
    pub blocked: Vec<bool>,
    /// Where the bottom-left corner of the map sits.
    pub origin: Point,
    /// The lettered cells of a drawn map.
    marks: Vec<(char, usize)>,
}

impl Map {
    /// A map with nothing blocked.
    pub fn open(width: usize, height: usize) -> Self {
        Self::drawn(
            &vec![".".repeat(width); height]
                .iter()
                .map(String::as_str)
                .collect::<Vec<_>>(),
        )
    }

    /// A map drawn as text, top row first: `#` is blocked, `.` is free, and
    /// a letter is a free cell that [`Map::at`] can find again.
    ///
    /// ```text
    /// S.#..
    /// ..#..
    /// ....G
    /// ```
    pub fn drawn(rows: &[&str]) -> Self {
        let (width, height) = (rows[0].len(), rows.len());
        let mut map = Self {
            width,
            height,
            blocked: vec![false; width * height],
            origin: p(0, 0),
            marks: Vec::new(),
        };
        for (row, line) in rows.iter().enumerate() {
            assert_eq!(
                line.len(),
                width,
                "every row of a drawn map is the same width"
            );
            for (x, symbol) in line.chars().enumerate() {
                let cell = (height - 1 - row) * width + x;
                match symbol {
                    '#' => map.blocked[cell] = true,
                    '.' => {}
                    mark => map.marks.push((mark, cell)),
                }
            }
        }
        map
    }

    pub fn random(rng: &mut Rng, width: usize, height: usize, blocked_percent: usize) -> Self {
        let mut map = Self::open(width, height);
        for cell in 0..width * height {
            map.blocked[cell] = rng.below(100) < blocked_percent;
        }
        map
    }

    /// The same map moved so that its bottom-left corner is at `origin`.
    pub fn moved_to(mut self, origin: Point) -> Self {
        self.origin = origin;
        self
    }

    /// The cell a letter was drawn in.
    pub fn cell(&self, mark: char) -> usize {
        let found = self.marks.iter().find(|&&(m, _)| m == mark);
        found
            .unwrap_or_else(|| panic!("the map has no cell marked {mark}"))
            .1
    }

    /// The centre of the cell a letter was drawn in.
    pub fn at(&self, mark: char) -> Point {
        self.centre(self.cell(mark))
    }

    /// The corner of the cell grid `x` cells right of and `y` cells above
    /// the bottom-left corner of the map.
    pub fn corner(&self, x: i32, y: i32) -> Point {
        p(self.origin.x + x * M, self.origin.y + y * M)
    }

    pub fn centre(&self, cell: usize) -> Point {
        p(
            self.origin.x + (cell % self.width) as i32 * M + 500,
            self.origin.y + (cell / self.width) as i32 * M + 500,
        )
    }

    /// The same map as terrain to generate a mesh from.
    pub fn terrain(&self) -> Terrain {
        let mut terrain = Terrain::new(self.origin, M, self.width, self.height).unwrap();
        for cell in (0..self.blocked.len()).filter(|&cell| self.blocked[cell]) {
            terrain.set_blocked(cell % self.width, cell / self.width, true);
        }
        terrain
    }

    /// The four corners of a cell, counter-clockwise.
    pub fn cell_corners(&self, cell: usize) -> [Point; 4] {
        let (x, y) = ((cell % self.width) as i32, (cell / self.width) as i32);
        [
            self.corner(x, y),
            self.corner(x + 1, y),
            self.corner(x + 1, y + 1),
            self.corner(x, y + 1),
        ]
    }

    /// The same ground built the other way: a mesh of the whole map with an
    /// obstacle laid over every blocked cell.
    pub fn mesh_with_obstacles(&self) -> NavMesh {
        let mut mesh = Map::open(self.width, self.height)
            .moved_to(self.origin)
            .navmesh();
        let mut edit = mesh.edit_obstacles();
        for cell in (0..self.blocked.len()).filter(|&cell| self.blocked[cell]) {
            edit.add(&self.cell_corners(cell), ObstacleKind::Blocked)
                .unwrap();
        }
        drop(edit);
        mesh
    }

    /// The mesh alone, for tests that do not need to know which polygon a
    /// cell landed in.
    pub fn navmesh(&self) -> NavMesh {
        self.mesh().0
    }

    /// Builds the mesh by linking shared edges. Also returns the polygon of
    /// each cell.
    pub fn mesh(&self) -> (NavMesh, Vec<Option<PolyId>>) {
        let (vertices, polygons, cell_poly) = self.polygons();
        (
            NavMesh::from_polygons(&vertices, &polygons).unwrap(),
            cell_poly,
        )
    }

    /// One polygon per horizontal run of free cells, with a corner at every
    /// cell boundary so that runs in neighbouring rows share whole edges.
    /// Returns the vertices, the polygons and the polygon of each cell.
    pub fn polygons(&self) -> (Vec<Point>, Vec<Vec<u32>>, Vec<Option<PolyId>>) {
        let vertex = |x: usize, y: usize| (y * (self.width + 1) + x) as u32;
        let vertices: Vec<Point> = (0..(self.width + 1) * (self.height + 1))
            .map(|i| self.corner((i % (self.width + 1)) as i32, (i / (self.width + 1)) as i32))
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
        (vertices, polygons, cell_poly)
    }

    /// Labels each free cell with its 4-connected region.
    pub fn regions(&self) -> Vec<usize> {
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
    pub fn octile(&self, from: usize, to: usize) -> i64 {
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

/// Checks everything a route must satisfy on a mesh with no obstacles: all
/// of [`check_route`], and that every turn is at a corner of a polygon the
/// route crosses.
pub fn check_path(mesh: &NavMesh, path: &Path, start: Point, goal: Point) {
    check_route(mesh, path, start, goal);
    let turns = path.points.len().saturating_sub(2);
    for turn in path.points.iter().skip(1).take(turns) {
        let at_a_corner = |&poly: &PolyId| mesh.corners(poly).any(|corner| corner == *turn);
        assert!(
            path.corridor.iter().any(at_a_corner),
            "turn at {turn:?} is not a corner"
        );
    }
}

/// Checks everything a route must satisfy whatever is on the mesh.
pub fn check_route(mesh: &NavMesh, path: &Path, start: Point, goal: Point) {
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
