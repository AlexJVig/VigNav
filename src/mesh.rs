//! The navigation mesh: convex polygons joined along shared edges.
//!
//! One [`NavMesh`] is one walkable layer, typically the ground as one class
//! of unit sees it. A game with several unit classes keeps one mesh per
//! class.

use std::fmt;
use std::ops::{Range, RangeInclusive};

use crate::geom::{Coord, Point, cross, dist_sq};
use crate::live::Live;

/// How far, in millimetres, a corner may dent inwards and still count as
/// convex.
///
/// A mesh converted from floating point carries about this much error:
/// rounding a vertex to the nearest millimetre moves it by up to √2/2 mm, and
/// its two neighbours can move the line between them by as much again.
pub const CONVEX_SLACK: i64 = 2;

/// Marks "no polygon" in the packed index arrays.
const NONE: u32 = u32::MAX;

/// Index of a polygon within its [`NavMesh`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PolyId(pub u32);

impl PolyId {
    /// The id as an array index.
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A point together with the polygon it is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Location {
    /// The polygon.
    pub poly: PolyId,
    /// The point, inside the polygon or within a millimetre of its boundary.
    pub point: Point,
}

/// Why a set of polygons was rejected.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeshError {
    /// More vertices, polygons or edges than a `u32` can index.
    TooLarge,
    /// A vertex lies beyond [`COORD_LIMIT`](crate::geom::COORD_LIMIT).
    CoordinateOutOfRange {
        /// Index of the vertex.
        vertex: usize,
    },
    /// A polygon has fewer than three corners.
    TooFewCorners {
        /// Index of the polygon.
        polygon: usize,
    },
    /// A corner names a vertex that does not exist.
    VertexOutOfRange {
        /// Index of the polygon.
        polygon: usize,
        /// Which of its corners.
        corner: usize,
    },
    /// A polygon is wound clockwise or has no area.
    NotCounterClockwise {
        /// Index of the polygon.
        polygon: usize,
    },
    /// A corner sits at the same position as the corner before it.
    ZeroLengthEdge {
        /// Index of the polygon.
        polygon: usize,
        /// Which of its corners.
        corner: usize,
    },
    /// A corner dents inwards by more than [`CONVEX_SLACK`], or doubles back.
    NotConvex {
        /// Index of the polygon.
        polygon: usize,
        /// Which of its corners.
        corner: usize,
    },
    /// An edge is used by more than two polygons, or by two polygons running
    /// along it in the same direction.
    EdgeConflict {
        /// Index of one polygon using the edge.
        polygon: usize,
        /// Which of its edges; edge `i` runs from corner `i` to corner `i + 1`.
        edge: usize,
    },
    /// The neighbour lists do not match the polygons in number or length.
    LinkCountMismatch {
        /// Index of the first list that does not fit.
        polygon: usize,
    },
    /// A link names a missing polygon, the polygon itself, or a polygon that
    /// does not link back along the same edge.
    BadLink {
        /// Index of the polygon.
        polygon: usize,
        /// Which of its edges; edge `i` runs from corner `i` to corner `i + 1`.
        edge: usize,
    },
}

impl fmt::Display for MeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::TooLarge => write!(f, "mesh is too large to index with 32 bits"),
            Self::CoordinateOutOfRange { vertex } => {
                write!(f, "vertex {vertex} is beyond the coordinate limit")
            }
            Self::TooFewCorners { polygon } => {
                write!(f, "polygon {polygon} has fewer than three corners")
            }
            Self::VertexOutOfRange { polygon, corner } => {
                write!(
                    f,
                    "polygon {polygon} corner {corner} names a missing vertex"
                )
            }
            Self::NotCounterClockwise { polygon } => {
                write!(f, "polygon {polygon} is clockwise or has no area")
            }
            Self::ZeroLengthEdge { polygon, corner } => {
                write!(
                    f,
                    "polygon {polygon} corner {corner} repeats the corner before it"
                )
            }
            Self::NotConvex { polygon, corner } => {
                write!(f, "polygon {polygon} is not convex at corner {corner}")
            }
            Self::EdgeConflict { polygon, edge } => {
                write!(f, "polygon {polygon} edge {edge} is shared inconsistently")
            }
            Self::LinkCountMismatch { polygon } => {
                write!(f, "neighbour list {polygon} does not match its polygon")
            }
            Self::BadLink { polygon, edge } => {
                write!(
                    f,
                    "polygon {polygon} edge {edge} has a link that is not answered"
                )
            }
        }
    }
}

impl std::error::Error for MeshError {}

/// A set of convex polygons and the links between them.
///
/// Polygons are wound counter-clockwise. Edge `i` of a polygon runs from its
/// corner `i` to its corner `i + 1`, wrapping at the end. An edge either lies
/// on the outside of the mesh or links to exactly one polygon across it.
///
/// The polygons never change once the mesh is built. Obstacles are laid over
/// them (see [`NavMesh::add_obstacle`]); lookups and path queries see the
/// ground the obstacles leave.
#[derive(Clone, Debug)]
pub struct NavMesh {
    vertices: Vec<Point>,
    /// Vertex each edge starts at. It ends where the polygon's next edge starts.
    edge_origin: Vec<u32>,
    /// Polygon across each edge, or `NONE` on the outside of the mesh.
    edge_neighbor: Vec<u32>,
    /// The same edge as the polygon across it holds it, or `NONE`.
    edge_twin: Vec<u32>,
    /// Polygon each edge belongs to.
    edge_poly: Vec<u32>,
    /// First edge of each polygon, then one past the last edge of all.
    first_edge: Vec<u32>,
    /// Connected group each polygon belongs to, ignoring obstacles.
    island: Vec<u32>,
    /// Bounding box of each polygon.
    boxes: Vec<(Point, Point)>,
    pub(crate) grid: Grid,
    pub(crate) live: Live,
}

impl NavMesh {
    /// Builds a mesh, linking polygons wherever two of them share an edge.
    ///
    /// Each polygon lists its corners as indices into `vertices`,
    /// counter-clockwise. Two edges are shared when they join the same two
    /// positions in opposite directions; the polygons need not use the same
    /// vertex indices.
    pub fn from_polygons<P: AsRef<[u32]>>(
        vertices: &[Point],
        polygons: &[P],
    ) -> Result<Self, MeshError> {
        let mut mesh = Self::unlinked(vertices, polygons)?;
        mesh.link_shared_edges()?;
        Ok(mesh.finished())
    }

    /// Builds a mesh whose links are already known, as in a mesh exported
    /// from another tool.
    ///
    /// `neighbors[p][i]` is the polygon across edge `i` of polygon `p`, or
    /// `None` on the outside of the mesh. Every link must be answered by the
    /// other polygon along the same two positions.
    pub fn from_linked_polygons<P, N>(
        vertices: &[Point],
        polygons: &[P],
        neighbors: &[N],
    ) -> Result<Self, MeshError>
    where
        P: AsRef<[u32]>,
        N: AsRef<[Option<u32>]>,
    {
        let mut mesh = Self::unlinked(vertices, polygons)?;
        mesh.link_as_given(neighbors)?;
        Ok(mesh.finished())
    }

    /// How many polygons the mesh has. Their ids run from zero.
    pub fn polygon_count(&self) -> usize {
        self.first_edge.len() - 1
    }

    /// The corners of a polygon, counter-clockwise.
    ///
    /// Panics if `poly` is not a polygon of this mesh.
    pub fn corners(&self, poly: PolyId) -> impl ExactSizeIterator<Item = Point> {
        self.edges_of(poly.index())
            .map(|edge| self.vertices[self.edge_origin[edge] as usize])
    }

    /// The polygon across each edge of a polygon, in edge order.
    ///
    /// Panics if `poly` is not a polygon of this mesh.
    pub fn neighbors(&self, poly: PolyId) -> impl ExactSizeIterator<Item = Option<PolyId>> {
        self.edges_of(poly.index())
            .map(|edge| self.neighbor_across(edge).map(|other| PolyId(other as u32)))
    }

    /// The connected group a polygon belongs to in the mesh as built. With no
    /// obstacles, a path exists between two polygons exactly when they are in
    /// the same group. Obstacles are not taken into account.
    ///
    /// Panics if `poly` is not a polygon of this mesh.
    pub fn island(&self, poly: PolyId) -> u32 {
        self.island[poly.index()]
    }

    /// Whether `p` is inside a polygon or on its boundary. Obstacles are not
    /// taken into account; [`locate`](NavMesh::locate) is the test for
    /// ground a unit can stand on.
    ///
    /// Panics if `poly` is not a polygon of this mesh.
    pub fn contains(&self, poly: PolyId, p: Point) -> bool {
        p.in_range()
            && self.edges_of(poly.index()).all(|edge| {
                let (a, b) = self.edge_points(edge);
                cross(a, b, p) >= 0
            })
    }

    /// The polygon whose ground `p` is on. On a shared edge, and where
    /// polygons overlap, this is the one with the lowest id; see
    /// [`locate_all`](NavMesh::locate_all). A point inside a blocking
    /// obstacle is on no polygon.
    pub fn locate(&self, p: Point) -> Option<PolyId> {
        self.piece_at(p).map(|(polygon, _)| PolyId(polygon))
    }

    /// Every polygon whose ground `p` is on, in ascending order. There is
    /// more than one on a shared edge and wherever one surface passes over
    /// another, such as a bridge over a road.
    pub fn locate_all(&self, p: Point) -> Vec<PolyId> {
        let Some(cell) = self.grid.cell_at(p) else {
            return Vec::new();
        };
        let on = |&&polygon: &&u32| {
            self.live.pieces[polygon as usize]
                .iter()
                .any(|piece| piece.contains(p))
        };
        self.grid
            .polygons_in(cell)
            .iter()
            .filter(on)
            .map(|&polygon| PolyId(polygon))
            .collect()
    }

    /// The point of the mesh nearest to `p`, if one lies within `max_dist`
    /// millimetres. A point already on the mesh is returned unchanged.
    pub fn nearest(&self, p: Point, max_dist: Coord) -> Option<Location> {
        if let Some(poly) = self.locate(p) {
            return Some(Location { poly, point: p });
        }
        self.nearest_piece(p, max_dist, |_| true)
            .map(|((polygon, _), point)| Location {
                poly: PolyId(polygon),
                point,
            })
    }

    /// The edges of a polygon, as indices into the edge arrays.
    pub(crate) fn edges_of(&self, polygon: usize) -> Range<usize> {
        self.first_edge[polygon] as usize..self.first_edge[polygon + 1] as usize
    }

    /// Where an edge starts and ends.
    pub(crate) fn edge_points(&self, edge: usize) -> (Point, Point) {
        let range = self.edges_of(self.owner_of(edge));
        let next = if edge + 1 == range.end {
            range.start
        } else {
            edge + 1
        };
        (
            self.vertices[self.edge_origin[edge] as usize],
            self.vertices[self.edge_origin[next] as usize],
        )
    }

    /// The polygon across an edge.
    pub(crate) fn neighbor_across(&self, edge: usize) -> Option<usize> {
        let other = self.edge_neighbor[edge];
        (other != NONE).then_some(other as usize)
    }

    /// The polygon an edge belongs to.
    pub(crate) fn owner_of(&self, edge: usize) -> usize {
        self.edge_poly[edge] as usize
    }

    /// The same edge as held by the polygon across it.
    pub(crate) fn twin_of(&self, edge: usize) -> Option<usize> {
        let twin = self.edge_twin[edge];
        (twin != NONE).then_some(twin as usize)
    }

    /// The bounding box of a polygon.
    pub(crate) fn box_of(&self, polygon: usize) -> (Point, Point) {
        self.boxes[polygon]
    }

    /// Checks every polygon and stores its edges, with no links yet.
    fn unlinked<P: AsRef<[u32]>>(vertices: &[Point], polygons: &[P]) -> Result<Self, MeshError> {
        if let Some(vertex) = vertices.iter().position(|v| !v.in_range()) {
            return Err(MeshError::CoordinateOutOfRange { vertex });
        }
        let edge_total: usize = polygons.iter().map(|p| p.as_ref().len()).sum();
        let limit = NONE as usize;
        if vertices.len() >= limit || polygons.len() >= limit || edge_total >= limit {
            return Err(MeshError::TooLarge);
        }
        let mut mesh = Self {
            vertices: vertices.to_vec(),
            edge_origin: Vec::with_capacity(edge_total),
            edge_neighbor: vec![NONE; edge_total],
            edge_twin: vec![NONE; edge_total],
            edge_poly: Vec::with_capacity(edge_total),
            first_edge: Vec::with_capacity(polygons.len() + 1),
            island: Vec::new(),
            boxes: Vec::new(),
            grid: Grid::default(),
            live: Live::default(),
        };
        for (polygon, corners) in polygons.iter().enumerate() {
            let corners = corners.as_ref();
            check_polygon(vertices, polygon, corners)?;
            mesh.first_edge.push(mesh.edge_origin.len() as u32);
            mesh.edge_origin.extend_from_slice(corners);
            mesh.edge_poly
                .extend(std::iter::repeat_n(polygon as u32, corners.len()));
        }
        mesh.first_edge.push(mesh.edge_origin.len() as u32);
        Ok(mesh)
    }

    /// Links every pair of edges that join the same two positions.
    fn link_shared_edges(&mut self) -> Result<(), MeshError> {
        // Key each edge by its endpoints in sorted order, so that the two
        // sides of a shared edge sort next to each other.
        let mut keyed: Vec<(Point, Point, usize)> = (0..self.edge_origin.len())
            .map(|edge| {
                let (a, b) = self.edge_points(edge);
                (a.min(b), a.max(b), edge)
            })
            .collect();
        keyed.sort_unstable();
        for group in keyed.chunk_by(|l, r| (l.0, l.1) == (r.0, r.1)) {
            match *group {
                [_] => {}
                [(.., e), (.., f)]
                    if self.owner_of(e) != self.owner_of(f)
                        && self.edge_points(e).0 != self.edge_points(f).0 =>
                {
                    self.edge_neighbor[e] = self.edge_poly[f];
                    self.edge_neighbor[f] = self.edge_poly[e];
                    self.edge_twin[e] = f as u32;
                    self.edge_twin[f] = e as u32;
                }
                _ => {
                    let edge = group[0].2;
                    let polygon = self.owner_of(edge);
                    let edge = edge - self.edges_of(polygon).start;
                    return Err(MeshError::EdgeConflict { polygon, edge });
                }
            }
        }
        Ok(())
    }

    /// Stores the caller's links after checking each is answered.
    fn link_as_given<N: AsRef<[Option<u32>]>>(&mut self, neighbors: &[N]) -> Result<(), MeshError> {
        let count = self.polygon_count();
        if neighbors.len() != count {
            return Err(MeshError::LinkCountMismatch {
                polygon: neighbors.len().min(count),
            });
        }
        for (polygon, links) in neighbors.iter().enumerate() {
            let (links, edges) = (links.as_ref(), self.edges_of(polygon));
            if links.len() != edges.len() {
                return Err(MeshError::LinkCountMismatch { polygon });
            }
            for (edge, link) in links.iter().enumerate() {
                let Some(other) = *link else { continue };
                if other as usize >= count || other as usize == polygon {
                    return Err(MeshError::BadLink { polygon, edge });
                }
                self.edge_neighbor[edges.start + edge] = other;
            }
        }
        for edge in 0..self.edge_origin.len() {
            let Some(other) = self.neighbor_across(edge) else {
                continue;
            };
            let (a, b) = self.edge_points(edge);
            let polygon = self.owner_of(edge);
            let answer = self.edges_of(other).find(|&back| {
                self.neighbor_across(back) == Some(polygon) && self.edge_points(back) == (b, a)
            });
            let Some(back) = answer else {
                let edge = edge - self.edges_of(polygon).start;
                return Err(MeshError::BadLink { polygon, edge });
            };
            self.edge_twin[edge] = back as u32;
        }
        Ok(())
    }

    /// Derives the islands, the lookup grid and the live ground once the
    /// links are in place.
    fn finished(mut self) -> Self {
        let count = self.polygon_count();
        self.island = vec![NONE; count];
        let mut islands = 0;
        let mut pending = Vec::new();
        for seed in 0..count {
            if self.island[seed] != NONE {
                continue;
            }
            self.island[seed] = islands;
            pending.push(seed);
            while let Some(polygon) = pending.pop() {
                for edge in self.edges_of(polygon) {
                    if let Some(next) = self.neighbor_across(edge)
                        && self.island[next] == NONE
                    {
                        self.island[next] = islands;
                        pending.push(next);
                    }
                }
            }
            islands += 1;
        }
        self.boxes = (0..count).map(|p| self.bounding_box(p)).collect();
        self.grid = Grid::new(&self.boxes);
        self.start_live();
        self
    }

    fn bounding_box(&self, polygon: usize) -> (Point, Point) {
        let first = self.vertices[self.edge_origin[self.edges_of(polygon).start] as usize];
        self.corners(PolyId(polygon as u32))
            .fold((first, first), |(lo, hi), p| {
                (
                    Point::new(lo.x.min(p.x), lo.y.min(p.y)),
                    Point::new(hi.x.max(p.x), hi.y.max(p.y)),
                )
            })
    }
}

/// Checks one polygon's corner list against the rules in [`MeshError`].
pub(crate) fn check_polygon(
    vertices: &[Point],
    polygon: usize,
    corners: &[u32],
) -> Result<(), MeshError> {
    let n = corners.len();
    if n < 3 {
        return Err(MeshError::TooFewCorners { polygon });
    }
    if let Some(corner) = corners.iter().position(|&v| v as usize >= vertices.len()) {
        return Err(MeshError::VertexOutOfRange { polygon, corner });
    }
    let at = |i: usize| vertices[corners[i % n] as usize];
    let doubled_area: i128 = (1..n - 1)
        .map(|i| i128::from(cross(at(0), at(i), at(i + 1))))
        .sum();
    if doubled_area <= 0 {
        return Err(MeshError::NotCounterClockwise { polygon });
    }
    if let Some(corner) = (0..n).find(|&corner| at(corner + n - 1) == at(corner)) {
        return Err(MeshError::ZeroLengthEdge { polygon, corner });
    }
    let dented =
        |&corner: &usize| !turns_left_within_slack(at(corner + n - 1), at(corner), at(corner + 1));
    if let Some(corner) = (0..n).find(dented) {
        return Err(MeshError::NotConvex { polygon, corner });
    }
    Ok(())
}

/// Whether the corner `b`, between `a` and `c`, is convex up to
/// [`CONVEX_SLACK`].
fn turns_left_within_slack(a: Point, b: Point, c: Point) -> bool {
    let turn = cross(a, b, c);
    if turn > 0 {
        return true;
    }
    if turn == 0 {
        // Collinear: fine while the boundary keeps going forwards.
        let forwards = (i64::from(b.x) - i64::from(a.x)) * (i64::from(c.x) - i64::from(b.x))
            + (i64::from(b.y) - i64::from(a.y)) * (i64::from(c.y) - i64::from(b.y));
        return forwards > 0;
    }
    // A dent: `b` may sit at most CONVEX_SLACK inside the line a-c. Its
    // distance from that line is |turn| / |ac|; compare the squares.
    let turn = i128::from(turn);
    turn * turn <= i128::from(CONVEX_SLACK * CONVEX_SLACK) * i128::from(dist_sq(a, c))
}

/// A uniform grid over the mesh. Each cell lists the polygons whose bounding
/// box touches it, in ascending order.
#[derive(Clone, Debug, Default)]
pub(crate) struct Grid {
    min: Point,
    max: Point,
    /// Side of a cell, in millimetres.
    pub(crate) cell: i64,
    pub(crate) cols: i64,
    pub(crate) rows: i64,
    /// Where each cell's polygons start in `items`, plus the total at the end.
    starts: Vec<usize>,
    items: Vec<u32>,
}

impl Grid {
    fn new(boxes: &[(Point, Point)]) -> Self {
        let Some(&(mut min, mut max)) = boxes.first() else {
            return Self::default();
        };
        for &(lo, hi) in boxes {
            min = Point::new(min.x.min(lo.x), min.y.min(lo.y));
            max = Point::new(max.x.max(hi.x), max.y.max(hi.y));
        }
        let width = i64::from(max.x) - i64::from(min.x) + 1;
        let height = i64::from(max.y) - i64::from(min.y) + 1;
        let count = boxes.len() as i64;
        // Aim for about one polygon per cell. A long thin mesh would need far
        // more cells than polygons at that size, so grow the cell until the
        // grid is of the same order as the mesh.
        let dims = |cell: i64| ((width + cell - 1) / cell, (height + cell - 1) / cell);
        let mut cell = (width * height / count).isqrt().max(1);
        while dims(cell).0 * dims(cell).1 > 4 * count + 16 {
            cell *= 2;
        }
        let (cols, rows) = dims(cell);
        let cells = (cols * rows) as usize;
        let mut grid = Self {
            min,
            max,
            cell,
            cols,
            rows,
            starts: vec![0; cells + 1],
            items: Vec::new(),
        };

        // Count each cell's polygons, turn the counts into offsets, then fill.
        let corners = |p: Point| (i64::from(p.x), i64::from(p.y));
        for &(lo, hi) in boxes {
            let (cols_hit, rows_hit) = grid.span(corners(lo), corners(hi));
            for row in rows_hit {
                for col in cols_hit.clone() {
                    grid.starts[(row * cols + col) as usize + 1] += 1;
                }
            }
        }
        for index in 0..cells {
            grid.starts[index + 1] += grid.starts[index];
        }
        grid.items = vec![0; grid.starts[cells]];
        let mut next = grid.starts.clone();
        for (polygon, &(lo, hi)) in boxes.iter().enumerate() {
            let (cols_hit, rows_hit) = grid.span(corners(lo), corners(hi));
            for row in rows_hit {
                for col in cols_hit.clone() {
                    let index = (row * cols + col) as usize;
                    grid.items[next[index]] = polygon as u32;
                    next[index] += 1;
                }
            }
        }
        grid
    }

    /// The columns and rows touched by the box `lo`..=`hi`, clamped to the
    /// grid. The grid must not be empty.
    pub(crate) fn span(
        &self,
        lo: (i64, i64),
        hi: (i64, i64),
    ) -> (RangeInclusive<i64>, RangeInclusive<i64>) {
        let col = |x: i64| ((x - i64::from(self.min.x)) / self.cell).clamp(0, self.cols - 1);
        let row = |y: i64| ((y - i64::from(self.min.y)) / self.cell).clamp(0, self.rows - 1);
        (col(lo.0)..=col(hi.0), row(lo.1)..=row(hi.1))
    }

    /// The cell holding `p`, if `p` is within the grid.
    pub(crate) fn cell_at(&self, p: Point) -> Option<usize> {
        let inside = self.cols > 0
            && (self.min.x..=self.max.x).contains(&p.x)
            && (self.min.y..=self.max.y).contains(&p.y);
        inside.then(|| {
            let col = (i64::from(p.x) - i64::from(self.min.x)) / self.cell;
            let row = (i64::from(p.y) - i64::from(self.min.y)) / self.cell;
            (row * self.cols + col) as usize
        })
    }

    pub(crate) fn polygons_in(&self, cell: usize) -> &[u32] {
        &self.items[self.starts[cell]..self.starts[cell + 1]]
    }
}
