//! Building a mesh from a grid of terrain cells.
//!
//! A [`Terrain`] is a rectangle of square cells, each passable or blocked.
//! [`Terrain::generate`] turns the passable ground into a [`NavMesh`] for
//! one class of unit, keeping that unit's radius clear of every blocked
//! cell and of the edge of the map.
//!
//! # How it works
//!
//! Everything happens on a lattice of points `resolution` apart, which the
//! cell size must be a whole multiple of.
//!
//! 1. **Grow.** Each blocked cell is grown by the radius, rounded up to the
//!    lattice, with its corners cut at 45 degrees. That is the smallest
//!    eight-sided shape on the lattice that holds every point within the
//!    radius of the cell, so the ground left over is never too close.
//! 2. **Slice.** The ground left over is read off a row of the lattice at a
//!    time, as runs that are each a rectangle, a trapezoid or a triangle.
//! 3. **Stack.** A run is added to the polygon below it while the two fit
//!    edge to edge and the result stays convex. Otherwise it starts a new
//!    polygon.
//! 4. **Join.** Where a polygon's top or bottom meets several polygons, it
//!    gets a corner at each place the neighbour changes, so that every
//!    shared edge is a whole edge on both sides.
//!
//! All of it is exact integer arithmetic, and the mesh depends on nothing
//! but the terrain and the [`Layer`].

use std::fmt;

use crate::geom::{COORD_LIMIT, Coord, Point, convex_hull, cross};
use crate::mesh::{MeshError, NavMesh, PolyId};

/// The most lattice points a terrain may be generated on. A finer lattice
/// than this is almost certainly a mistake in the resolution.
const LATTICE_LIMIT: i64 = 1 << 26;

/// Why a terrain or a mesh could not be made.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GenerateError {
    /// The cell size is not positive, or there are no cells.
    NoCells,
    /// Part of the terrain lies beyond [`COORD_LIMIT`].
    OutOfRange,
    /// The list of heights does not hold one height per cell corner.
    HeightCount {
        /// How many heights the terrain needs.
        expected: usize,
        /// How many were given.
        given: usize,
    },
    /// The radius is negative.
    NegativeRadius,
    /// The resolution is not positive, or the cell size is not a whole
    /// multiple of it.
    BadResolution {
        /// The terrain's cell size.
        cell: Coord,
        /// The resolution asked for.
        resolution: Coord,
    },
    /// The resolution is so fine that the lattice would be unreasonably
    /// large.
    TooFine,
    /// The polygons generated did not make a valid mesh. This is a fault in
    /// the generator, not in the terrain.
    Mesh(MeshError),
}

impl fmt::Display for GenerateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoCells => write!(f, "the terrain has no cells"),
            Self::OutOfRange => write!(f, "the terrain reaches beyond the coordinate limit"),
            Self::HeightCount { expected, given } => {
                write!(
                    f,
                    "the terrain needs {expected} heights and was given {given}"
                )
            }
            Self::NegativeRadius => write!(f, "the radius is negative"),
            Self::BadResolution { cell, resolution } => {
                write!(
                    f,
                    "a cell of {cell} mm is not a whole number of {resolution} mm steps"
                )
            }
            Self::TooFine => write!(f, "the resolution is too fine for a terrain this size"),
            Self::Mesh(error) => write!(f, "the generated polygons are not a mesh: {error}"),
        }
    }
}

impl std::error::Error for GenerateError {}

/// What one class of unit needs of the ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layer {
    /// How far, in millimetres, the unit's centre must stay from blocked
    /// ground and from the edge of the map.
    pub radius: Coord,
    /// The spacing of the lattice the mesh is drawn on. The cell size must
    /// be a whole multiple of it.
    ///
    /// The radius is always kept clear, and rounding it up to the lattice
    /// costs some extra: ground is only ever left out if it is nearer than
    /// `1.09 * radius + 2 * resolution` to something. A lattice no coarser
    /// than half the radius keeps that small.
    pub resolution: Coord,
    /// The smallest area, in square millimetres, a patch of ground cut off
    /// from the rest must have to be kept. Smaller pockets are left out.
    pub min_island: i64,
}

impl Layer {
    /// A layer for units of this radius, on a lattice of this spacing, that
    /// keeps every pocket of ground however small.
    pub const fn new(radius: Coord, resolution: Coord) -> Self {
        Self {
            radius,
            resolution,
            min_island: 0,
        }
    }
}

/// A rectangle of square cells, each passable or blocked.
///
/// Cell `(0, 0)` has its lowest corner at the origin. `x` counts cells
/// along the first ground axis and `y` along the second.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Terrain {
    origin: Point,
    cell: Coord,
    width: usize,
    height: usize,
    blocked: Vec<bool>,
}

impl Terrain {
    /// Open ground, `width` by `height` cells of `cell` millimetres, with
    /// the lowest corner of the first cell at `origin`.
    pub fn new(
        origin: Point,
        cell: Coord,
        width: usize,
        height: usize,
    ) -> Result<Self, GenerateError> {
        if cell <= 0 || width == 0 || height == 0 {
            return Err(GenerateError::NoCells);
        }
        let limit = i64::from(COORD_LIMIT);
        let far = |from: Coord, cells: usize| {
            i64::try_from(cells)
                .ok()?
                .checked_mul(i64::from(cell))?
                .checked_add(i64::from(from))
        };
        let fits = |from: Coord, cells: usize| {
            i64::from(from) >= -limit && far(from, cells).is_some_and(|end| end <= limit)
        };
        if !fits(origin.x, width) || !fits(origin.y, height) {
            return Err(GenerateError::OutOfRange);
        }
        Ok(Self {
            origin,
            cell,
            width,
            height,
            blocked: vec![false; width * height],
        })
    }

    /// How many cells wide the terrain is.
    pub fn width(&self) -> usize {
        self.width
    }

    /// How many cells high the terrain is.
    pub fn height(&self) -> usize {
        self.height
    }

    /// The side of a cell, in millimetres.
    pub fn cell(&self) -> Coord {
        self.cell
    }

    /// The lowest corner of cell `(0, 0)`.
    pub fn origin(&self) -> Point {
        self.origin
    }

    /// Whether a cell is blocked. Cells outside the terrain are.
    pub fn is_blocked(&self, x: usize, y: usize) -> bool {
        x >= self.width || y >= self.height || self.blocked[y * self.width + x]
    }

    /// Blocks or opens one cell.
    ///
    /// Panics if the cell is outside the terrain.
    pub fn set_blocked(&mut self, x: usize, y: usize, blocked: bool) {
        assert!(
            x < self.width && y < self.height,
            "cell ({x}, {y}) is outside the terrain"
        );
        self.blocked[y * self.width + x] = blocked;
    }

    /// Blocks every cell that is too steep to cross: one whose four corners
    /// differ in height by more than `max_rise`. Returns how many cells that
    /// newly blocked.
    ///
    /// `heights` holds one height per cell corner, row by row from the
    /// origin: `(width + 1) * (height + 1)` of them.
    pub fn block_steep(
        &mut self,
        heights: &[Coord],
        max_rise: Coord,
    ) -> Result<usize, GenerateError> {
        let across = self.width + 1;
        let expected = across * (self.height + 1);
        if heights.len() != expected {
            return Err(GenerateError::HeightCount {
                expected,
                given: heights.len(),
            });
        }
        let mut newly = 0;
        for y in 0..self.height {
            for x in 0..self.width {
                let at = y * across + x;
                let corners = [
                    heights[at],
                    heights[at + 1],
                    heights[at + across],
                    heights[at + across + 1],
                ];
                let low = corners.iter().copied().min().map_or(0, i64::from);
                let high = corners.iter().copied().max().map_or(0, i64::from);
                let cell = &mut self.blocked[y * self.width + x];
                if high - low > i64::from(max_rise) && !*cell {
                    *cell = true;
                    newly += 1;
                }
            }
        }
        Ok(newly)
    }

    /// Blocks every cell that a shape covers any part of: the footprint of
    /// a building or anything else that will never move. Returns how many
    /// cells that newly blocked.
    ///
    /// The shape is the convex hull of `corners`. A cell the shape only
    /// touches along an edge or at a corner stays open.
    pub fn block_shape(&mut self, corners: &[Point]) -> usize {
        let hull = convex_hull(corners);
        if hull.len() < 3 {
            return 0;
        }
        let (cell, origin) = (i64::from(self.cell), self.origin);
        // The cells the shape's bounding box reaches into.
        let span = |low: Coord, high: Coord, from: Coord, cells: usize| {
            let first = (i64::from(low) - i64::from(from)).div_euclid(cell).max(0);
            let last =
                ((i64::from(high) - i64::from(from) - 1).div_euclid(cell)).min(cells as i64 - 1);
            first..=last
        };
        let (xs, ys) = (hull.iter().map(|p| p.x), hull.iter().map(|p| p.y));
        let columns = span(
            xs.clone().min().unwrap_or(0),
            xs.max().unwrap_or(0),
            origin.x,
            self.width,
        );
        let rows = span(
            ys.clone().min().unwrap_or(0),
            ys.max().unwrap_or(0),
            origin.y,
            self.height,
        );
        let mut newly = 0;
        for y in rows {
            for x in columns.clone() {
                let low = (
                    i64::from(origin.x) + x * cell,
                    i64::from(origin.y) + y * cell,
                );
                let corner = |dx: i64, dy: i64| {
                    Point::new((low.0 + dx * cell) as Coord, (low.1 + dy * cell) as Coord)
                };
                let square = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
                // Two convex shapes overlap unless an edge of one has the
                // whole of the other on its outside. The bounding box has
                // already done that for the cell's edges.
                let apart = (0..hull.len()).any(|i| {
                    let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
                    square.iter().all(|&q| cross(a, b, q) <= 0)
                });
                let at = &mut self.blocked[y as usize * self.width + x as usize];
                if !apart && !*at {
                    *at = true;
                    newly += 1;
                }
            }
        }
        newly
    }

    /// Builds the mesh of the ground a unit of this [`Layer`] can stand on.
    ///
    /// The result may have no polygons at all, if nothing is left once the
    /// radius has been kept clear.
    ///
    /// ```
    /// use vignav::{Layer, Point, Terrain};
    ///
    /// // A 30 m by 20 m map of 1 m cells with a rock in the middle.
    /// let mut terrain = Terrain::new(Point::new(0, 0), 1_000, 30, 20)?;
    /// terrain.set_blocked(15, 10, true);
    ///
    /// // Ground for units that need half a metre round them.
    /// let mesh = terrain.generate(&Layer::new(500, 250))?;
    /// assert!(mesh.locate(Point::new(15_500, 10_500)).is_none());
    ///
    /// // A route across the map has to go round the rock.
    /// let (start, goal) = (Point::new(2_000, 10_500), Point::new(28_000, 10_500));
    /// assert!(mesh.find_path(start, goal, 0)?.points.len() > 2);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn generate(&self, layer: &Layer) -> Result<NavMesh, GenerateError> {
        let lattice = Lattice::new(self, layer)?;
        let shapes = lattice.stack();
        let step = i64::from(layer.resolution);
        let place = |(x, y): (i32, i32)| {
            let along =
                |from: Coord, steps: i32| (i64::from(from) + i64::from(steps) * step) as Coord;
            Point::new(along(self.origin.x, x), along(self.origin.y, y))
        };
        let outlines: Vec<Vec<Point>> = outline(&shapes, lattice.high)
            .into_iter()
            .map(|corners| corners.into_iter().map(place).collect())
            .collect();
        let mesh = assemble(&outlines)?;
        if layer.min_island <= 0 {
            return Ok(mesh);
        }

        // Add up the ground in each connected group and keep the large ones.
        let mut doubled: Vec<i128> = Vec::new();
        for (polygon, corners) in outlines.iter().enumerate() {
            let island = mesh.island(PolyId(polygon as u32)) as usize;
            if doubled.len() <= island {
                doubled.resize(island + 1, 0);
            }
            doubled[island] += (1..corners.len() - 1)
                .map(|i| i128::from(cross(corners[0], corners[i], corners[i + 1])))
                .sum::<i128>();
        }
        if doubled
            .iter()
            .all(|&area| area >= 2 * i128::from(layer.min_island))
        {
            return Ok(mesh);
        }
        let kept: Vec<Vec<Point>> = outlines
            .into_iter()
            .enumerate()
            .filter(|&(polygon, _)| {
                doubled[mesh.island(PolyId(polygon as u32)) as usize]
                    >= 2 * i128::from(layer.min_island)
            })
            .map(|(_, corners)| corners)
            .collect();
        assemble(&kept)
    }
}

/// Makes a mesh of polygons that each carry their own corners.
fn assemble(outlines: &[Vec<Point>]) -> Result<NavMesh, GenerateError> {
    let mut vertices = Vec::new();
    let mut polygons: Vec<Vec<u32>> = Vec::with_capacity(outlines.len());
    for corners in outlines {
        let first = vertices.len() as u32;
        vertices.extend_from_slice(corners);
        polygons.push((first..first + corners.len() as u32).collect());
    }
    NavMesh::from_polygons(&vertices, &polygons).map_err(GenerateError::Mesh)
}

/// The terrain on the lattice: which lattice points a unit's centre may be
/// at, and which cells are open.
struct Lattice<'a> {
    terrain: &'a Terrain,
    /// Lattice steps per cell.
    per_cell: i32,
    wide: i32,
    high: i32,
    /// Whether each lattice point is clear, row by row.
    clear: Vec<bool>,
}

/// What a unit can use of one square of the lattice.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Square {
    None,
    All,
    /// All but the half nearest one corner, cut off along the diagonal.
    Without(Corner),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Corner {
    LowLeft,
    LowRight,
    HighLeft,
    HighRight,
}

/// One run of open ground in one row of the lattice: where it starts and
/// ends along the bottom of the row and along the top.
#[derive(Clone, Copy)]
struct Run {
    bottom: (i32, i32),
    top: (i32, i32),
}

/// A polygon being built upwards a row at a time, and then finished.
struct Stack {
    /// The lattice line its bottom edge is on, and the edge.
    floor: i32,
    bottom: (i32, i32),
    /// The lattice line its top edge is on so far, and the edge.
    ceiling: i32,
    top: (i32, i32),
    /// The corners up its left and right sides, lowest first, not counting
    /// the ends of the bottom and top edges.
    left: Vec<(i32, i32)>,
    right: Vec<(i32, i32)>,
    /// How far its left and right sides leaned across its last row.
    lean: (i32, i32),
}

impl<'a> Lattice<'a> {
    fn new(terrain: &'a Terrain, layer: &Layer) -> Result<Self, GenerateError> {
        let (cell, step) = (terrain.cell, layer.resolution);
        if layer.radius < 0 {
            return Err(GenerateError::NegativeRadius);
        }
        if step <= 0 || cell % step != 0 {
            return Err(GenerateError::BadResolution {
                cell,
                resolution: step,
            });
        }
        let per_cell = i64::from(cell / step);
        let (wide, high) = (
            terrain.width as i64 * per_cell,
            terrain.height as i64 * per_cell,
        );
        if (wide + 1) * (high + 1) > LATTICE_LIMIT {
            return Err(GenerateError::TooFine);
        }
        let (wide, high, per_cell) = (wide as i32, high as i32, per_cell as i32);

        // The radius in lattice steps, rounded up, and the same for where
        // the cut across each corner of a grown cell starts: the radius
        // times tan(22.5 degrees).
        let steps = |length: i64| ((length + i64::from(step) - 1) / i64::from(step)) as i32;
        let radius = i64::from(layer.radius);
        let mut reach = steps(radius);
        let cut = steps((radius * 41_422 + 99_999) / 100_000).min(reach);
        // A half square is open up to its diagonal, which takes one step off
        // the corner of a grown cell whether `cut` asks for it or not. The
        // corner must still be the radius away along its diagonal, which is
        // the radius times the square root of two along the axes. With a
        // radius of a single step it would not be, so keep a second step.
        let across = steps((radius * 141_422 + 99_999) / 100_000);
        if reach > 0 && 2 * reach - 1 < across {
            reach += 1;
        }

        let mut lattice = Self {
            terrain,
            per_cell,
            wide,
            high,
            clear: vec![true; ((wide + 1) * (high + 1)) as usize],
        };
        if reach == 0 {
            return Ok(lattice);
        }
        // Nothing may be nearer the edge of the map than the radius.
        for y in 0..=high {
            for x in 0..=wide {
                if x.min(wide - x) < reach || y.min(high - y) < reach {
                    lattice.clear[(y * (wide + 1) + x) as usize] = false;
                }
            }
        }
        // Nor nearer a blocked cell. Only cells with open ground beside
        // them matter: the rest are further from it than those are.
        for cy in 0..terrain.height as i32 {
            for cx in 0..terrain.width as i32 {
                if !terrain.blocked[cy as usize * terrain.width + cx as usize] {
                    continue;
                }
                let beside =
                    |dx: i32, dy: i32| !terrain.is_blocked((cx + dx) as usize, (cy + dy) as usize);
                let edge = (-1..=1)
                    .any(|dy| (-1..=1).any(|dx| cx + dx >= 0 && cy + dy >= 0 && beside(dx, dy)));
                if !edge {
                    continue;
                }
                let (x0, y0) = (cx * per_cell, cy * per_cell);
                let (x1, y1) = (x0 + per_cell, y0 + per_cell);
                for y in (y0 - reach).max(0)..=(y1 + reach).min(high) {
                    for x in (x0 - reach).max(0)..=(x1 + reach).min(wide) {
                        let (dx, dy) = ((x0 - x).max(x - x1).max(0), (y0 - y).max(y - y1).max(0));
                        if dx < reach && dy < reach && dx + dy < reach + cut {
                            lattice.clear[(y * (wide + 1) + x) as usize] = false;
                        }
                    }
                }
            }
        }
        Ok(lattice)
    }

    /// What is open of the lattice square whose lowest corner is `(x, y)`.
    fn square(&self, x: i32, y: i32) -> Square {
        let (cx, cy) = ((x / self.per_cell) as usize, (y / self.per_cell) as usize);
        if self.terrain.blocked[cy * self.terrain.width + cx] {
            return Square::None;
        }
        let clear = |x: i32, y: i32| self.clear[(y * (self.wide + 1) + x) as usize];
        match (
            clear(x, y),
            clear(x + 1, y),
            clear(x, y + 1),
            clear(x + 1, y + 1),
        ) {
            (true, true, true, true) => Square::All,
            (false, true, true, true) => Square::Without(Corner::LowLeft),
            (true, false, true, true) => Square::Without(Corner::LowRight),
            (true, true, false, true) => Square::Without(Corner::HighLeft),
            (true, true, true, false) => Square::Without(Corner::HighRight),
            _ => Square::None,
        }
    }

    /// The runs of open ground in the row of squares above lattice line
    /// `y`, left to right.
    fn runs(&self, y: i32, runs: &mut Vec<Run>) {
        runs.clear();
        let mut x = 0;
        while x < self.wide {
            match self.square(x, y) {
                Square::None => x += 1,
                // A half that touches only its left side stands alone here:
                // anything to its left that reached it has taken it already.
                Square::Without(Corner::LowRight) => {
                    runs.push(Run {
                        bottom: (x, x),
                        top: (x, x + 1),
                    });
                    x += 1;
                }
                Square::Without(Corner::HighRight) => {
                    runs.push(Run {
                        bottom: (x, x + 1),
                        top: (x, x),
                    });
                    x += 1;
                }
                first => {
                    let (low, high) = match first {
                        Square::Without(Corner::LowLeft) => (x + 1, x),
                        Square::Without(Corner::HighLeft) => (x, x + 1),
                        _ => (x, x),
                    };
                    let mut end = x + 1;
                    while end < self.wide && self.square(end, y) == Square::All {
                        end += 1;
                    }
                    // A half that touches only its left side ends the run.
                    let last = if end < self.wide {
                        self.square(end, y)
                    } else {
                        Square::None
                    };
                    let (bottom, top) = match last {
                        Square::Without(Corner::LowRight) => (end, end + 1),
                        Square::Without(Corner::HighRight) => (end + 1, end),
                        _ => (end, end),
                    };
                    if bottom != top {
                        end += 1;
                    }
                    runs.push(Run {
                        bottom: (low, bottom),
                        top: (high, top),
                    });
                    x = end;
                }
            }
        }
    }

    /// Stacks the runs of every row into convex polygons.
    fn stack(&self) -> Vec<Stack> {
        let mut done = Vec::new();
        // The polygons whose top edge is on the current line, left to right.
        let mut open: Vec<Option<Stack>> = Vec::new();
        let mut runs = Vec::new();
        for y in 0..self.high {
            self.runs(y, &mut runs);
            let mut next: Vec<Option<Stack>> = Vec::with_capacity(runs.len());
            for run in &runs {
                let lean = (run.top.0 - run.bottom.0, run.top.1 - run.bottom.1);
                // The polygon below this run, if its top is exactly this
                // run's bottom and adding the run keeps both sides convex:
                // the left side may only turn rightwards going up, and the
                // right side only leftwards.
                let mut at = open.partition_point(|stack| {
                    stack
                        .as_ref()
                        .is_none_or(|stack| stack.top.0 < run.bottom.0)
                });
                let mut below = None;
                while let Some(Some(stack)) = open.get(at) {
                    if stack.top.0 != run.bottom.0 {
                        break;
                    }
                    let fits = stack.top == run.bottom && run.bottom.0 < run.bottom.1;
                    if fits && lean.0 >= stack.lean.0 && lean.1 <= stack.lean.1 {
                        below = open[at].take();
                        break;
                    }
                    at += 1;
                }
                next.push(Some(match below {
                    Some(mut stack) => {
                        if lean.0 != stack.lean.0 {
                            stack.left.push((stack.top.0, y));
                        }
                        if lean.1 != stack.lean.1 {
                            stack.right.push((stack.top.1, y));
                        }
                        Stack {
                            ceiling: y + 1,
                            top: run.top,
                            lean,
                            ..stack
                        }
                    }
                    None => Stack {
                        floor: y,
                        bottom: run.bottom,
                        ceiling: y + 1,
                        top: run.top,
                        left: Vec::new(),
                        right: Vec::new(),
                        lean,
                    },
                }));
            }
            done.extend(open.drain(..).flatten());
            open = next;
        }
        done.extend(open.into_iter().flatten());
        done
    }
}

/// The corners of each finished polygon, counter-clockwise, in lattice
/// steps, with a corner added wherever the polygon across its top or bottom
/// edge changes.
fn outline(shapes: &[Stack], high: i32) -> Vec<Vec<(i32, i32)>> {
    // Where the top edges ending on each lattice line start and stop, and
    // the same for the bottom edges starting there.
    let mut tops: Vec<Vec<i32>> = vec![Vec::new(); high as usize + 1];
    let mut bottoms: Vec<Vec<i32>> = vec![Vec::new(); high as usize + 1];
    for shape in shapes {
        tops[shape.ceiling as usize].extend([shape.top.0, shape.top.1]);
        bottoms[shape.floor as usize].extend([shape.bottom.0, shape.bottom.1]);
    }
    for ends in tops.iter_mut().chain(&mut bottoms) {
        ends.sort_unstable();
        ends.dedup();
    }
    let within = |ends: &'_ [i32], (low, high): (i32, i32)| {
        let from = ends.partition_point(|&end| end <= low);
        let until = ends.partition_point(|&end| end < high);
        from..until.max(from)
    };

    let mut outlines = Vec::with_capacity(shapes.len());
    for shape in shapes {
        let (floor, ceiling) = (shape.floor, shape.ceiling);
        let mut corners = vec![(shape.bottom.0, floor)];
        let under = &tops[floor as usize];
        corners.extend(
            under[within(under, shape.bottom)]
                .iter()
                .map(|&x| (x, floor)),
        );
        corners.push((shape.bottom.1, floor));
        corners.extend_from_slice(&shape.right);
        corners.push((shape.top.1, ceiling));
        let over = &bottoms[ceiling as usize];
        corners.extend(
            over[within(over, shape.top)]
                .iter()
                .rev()
                .map(|&x| (x, ceiling)),
        );
        corners.push((shape.top.0, ceiling));
        corners.extend(shape.left.iter().rev());
        // A top or bottom edge can be a single point.
        corners.dedup();
        if corners.first() == corners.last() {
            corners.pop();
        }
        outlines.push(corners);
    }
    outlines
}
