//! The live ground: each polygon as the obstacles on it currently leave it.
//!
//! Obstacles never change the polygons a mesh was built from. Each polygon
//! instead keeps a list of *pieces*: the convex parts of it that are left
//! once every obstacle touching it has been cut out. A polygon with nothing
//! on it has one piece, itself. Lookups and path queries run over pieces.
//!
//! Adding or removing an obstacle re-cuts only the polygons it touches and
//! re-links only those and their neighbours.

use std::collections::BTreeMap;

use crate::geom::{Coord, Point, closest_on_segment, cross, dist_sq, dot, split_point};
use crate::mesh::{CONVEX_SLACK, MeshError, NavMesh, check_polygon};

/// Marks a piece that belongs to no island.
pub(crate) const NO_ISLAND: u32 = u32::MAX;

/// A piece, named by its polygon and its place in that polygon's list.
pub(crate) type Node = (u32, u32);

/// What an obstacle does to the ground it covers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObstacleKind {
    /// Nothing may stand on or cross the ground: a building, a wall, a rock.
    Blocked,
    /// Routes may not enter the ground, but one that starts on it may leave,
    /// and a goal on it is moved to its edge: a unit standing still.
    Avoid,
}

/// Names an obstacle on the mesh it was added to. Ids are handed out in
/// order and never reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObstacleId(u32);

#[derive(Clone, Debug)]
struct Obstacle {
    corners: Vec<Point>,
    kind: ObstacleKind,
    lo: Point,
    hi: Point,
    /// The polygons its bounding box touches.
    polygons: Vec<u32>,
}

/// Where an edge of a piece came from. Edges with the same tag lie on the
/// same straight line, which is how pieces are matched up again.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Tag {
    /// Part of this edge of the polygon, by its index in the mesh.
    Base(u32),
    /// Part of the cut along this edge of this obstacle.
    Cut(u32, u32),
}

/// An opening from one piece into another: the stretch of boundary they
/// share, from its start to its end as the first piece winds.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Link {
    pub(crate) to: Node,
    pub(crate) from: Point,
    pub(crate) until: Point,
    /// Whether the piece it opens onto is ground to be avoided. Kept here
    /// so that a search need not look at the piece to find out.
    pub(crate) avoid: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct Piece {
    pub(crate) corners: Vec<Point>,
    tags: Vec<Tag>,
    pub(crate) links: Vec<Link>,
    /// Inside an [`ObstacleKind::Avoid`] obstacle.
    pub(crate) avoid: bool,
    /// The connected group of ordinary ground this piece is in.
    pub(crate) island: u32,
    lo: Point,
    hi: Point,
}

impl Piece {
    fn new(corners: Vec<Point>, tags: Vec<Tag>, avoid: bool) -> Self {
        let (mut lo, mut hi) = (corners[0], corners[0]);
        for p in &corners {
            lo = Point::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = Point::new(hi.x.max(p.x), hi.y.max(p.y));
        }
        Self {
            corners,
            tags,
            links: Vec::new(),
            avoid,
            island: NO_ISLAND,
            lo,
            hi,
        }
    }

    fn edge(&self, i: usize) -> (Point, Point) {
        (self.corners[i], self.corners[(i + 1) % self.corners.len()])
    }

    /// Whether `p` is inside the piece or on its boundary.
    pub(crate) fn contains(&self, p: Point) -> bool {
        (self.lo.x..=self.hi.x).contains(&p.x)
            && (self.lo.y..=self.hi.y).contains(&p.y)
            && (0..self.corners.len()).all(|i| {
                let (a, b) = self.edge(i);
                cross(a, b, p) >= 0
            })
    }

    /// The point of the piece nearest to `p`.
    pub(crate) fn closest_point(&self, p: Point) -> Point {
        if self.contains(p) {
            return p;
        }
        (0..self.corners.len())
            .map(|i| {
                let (a, b) = self.edge(i);
                closest_on_segment(p, a, b)
            })
            .min_by_key(|&q| dist_sq(p, q))
            .expect("a piece has at least three edges")
    }
}

/// The pieces of every polygon and the obstacles that shaped them.
#[derive(Clone, Debug, Default)]
pub(crate) struct Live {
    pub(crate) pieces: Vec<Vec<Piece>>,
    /// The number of pieces before each polygon's, then the total.
    pub(crate) first: Vec<u32>,
    obstacles: BTreeMap<u32, Obstacle>,
    next_id: u32,
    /// The obstacles touching each polygon, in ascending order.
    on_polygon: Vec<Vec<u32>>,
    /// Polygons whose obstacles changed since they were last cut.
    dirty: Vec<u32>,
}

impl Live {
    pub(crate) fn piece(&self, node: Node) -> &Piece {
        &self.pieces[node.0 as usize][node.1 as usize]
    }

    /// The place of a piece in an array with one entry per piece.
    pub(crate) fn slot(&self, node: Node) -> usize {
        (self.first[node.0 as usize] + node.1) as usize
    }

    /// How many pieces there are in all.
    pub(crate) fn total(&self) -> usize {
        self.first.last().map_or(0, |&total| total as usize)
    }
}

/// A set of changes to the obstacles of a mesh, applied together when it is
/// dropped. Re-cutting the ground once for many obstacles is much cheaper
/// than once for each.
pub struct ObstacleEdit<'a> {
    mesh: &'a mut NavMesh,
}

impl ObstacleEdit<'_> {
    /// Lays an obstacle over the mesh. `corners` is a convex shape,
    /// counter-clockwise, already grown by the clearance of the units the
    /// mesh is for (see [`grow_convex`](crate::geom::grow_convex)).
    ///
    /// The shape is checked as a polygon of a mesh is; an error names
    /// polygon 0.
    pub fn add(&mut self, corners: &[Point], kind: ObstacleKind) -> Result<ObstacleId, MeshError> {
        if let Some(vertex) = corners.iter().position(|v| !v.in_range()) {
            return Err(MeshError::CoordinateOutOfRange { vertex });
        }
        let order: Vec<u32> = (0..corners.len() as u32).collect();
        check_polygon(corners, 0, &order)?;
        Ok(self.mesh.place(corners, kind))
    }

    /// Takes an obstacle away again. Returns whether it was there.
    pub fn remove(&mut self, id: ObstacleId) -> bool {
        self.mesh.lift(id)
    }
}

impl Drop for ObstacleEdit<'_> {
    fn drop(&mut self) {
        self.mesh.settle();
    }
}

impl NavMesh {
    /// Lays one obstacle over the mesh; see [`ObstacleEdit::add`]. To change
    /// several at once, use [`edit_obstacles`](NavMesh::edit_obstacles).
    pub fn add_obstacle(
        &mut self,
        corners: &[Point],
        kind: ObstacleKind,
    ) -> Result<ObstacleId, MeshError> {
        self.edit_obstacles().add(corners, kind)
    }

    /// Takes one obstacle away again. Returns whether it was there.
    pub fn remove_obstacle(&mut self, id: ObstacleId) -> bool {
        self.edit_obstacles().remove(id)
    }

    /// Starts a set of changes to the obstacles, applied when the returned
    /// value is dropped.
    pub fn edit_obstacles(&mut self) -> ObstacleEdit<'_> {
        ObstacleEdit { mesh: self }
    }

    /// How many obstacles are on the mesh.
    pub fn obstacle_count(&self) -> usize {
        self.live.obstacles.len()
    }

    /// The piece `p` is on, lowest polygon first.
    pub(crate) fn piece_at(&self, p: Point) -> Option<Node> {
        let cell = self.grid.cell_at(p)?;
        self.grid.polygons_in(cell).iter().find_map(|&polygon| {
            let at = self.live.pieces[polygon as usize]
                .iter()
                .position(|piece| piece.contains(p));
            at.map(|piece| (polygon, piece as u32))
        })
    }

    /// The piece of one polygon that `p` is on, or within rounding of.
    pub(crate) fn piece_in(&self, polygon: usize, p: Point) -> Option<Node> {
        let pieces = self.live.pieces.get(polygon)?;
        if !p.in_range() {
            return None;
        }
        let exact = pieces.iter().position(|piece| piece.contains(p));
        let near = || {
            let reach = CONVEX_SLACK * CONVEX_SLACK;
            let off = |piece: &Piece| dist_sq(p, piece.closest_point(p));
            let best = (0..pieces.len()).min_by_key(|&i| off(&pieces[i]))?;
            (off(&pieces[best]) <= reach).then_some(best)
        };
        exact
            .or_else(near)
            .map(|piece| (polygon as u32, piece as u32))
    }

    /// The nearest point to `p`, within `max_dist`, on a piece `allow`
    /// accepts, and the piece it is on.
    ///
    /// Searches the lookup grid in growing rings round `p` and stops at the
    /// first ring that cannot hold anything nearer than what is in hand.
    pub(crate) fn nearest_piece(
        &self,
        p: Point,
        max_dist: Coord,
        allow: impl Fn(&Piece) -> bool,
    ) -> Option<(Node, Point)> {
        if !p.in_range() || max_dist < 0 || self.polygon_count() == 0 {
            return None;
        }
        let grid = &self.grid;
        let reach = i64::from(max_dist);
        let here = (i64::from(p.x), i64::from(p.y));
        let (cols, rows) = grid.span(here, here);
        let (col, row) = (*cols.start(), *rows.start());
        let mut best: Option<(i64, Node, Point)> = None;
        for ring in 0..=grid.cols.max(grid.rows) {
            // Everything first met in this ring is at least this far away.
            let floor = (ring - 1).max(0) * grid.cell;
            if floor > reach || best.is_some_and(|(nearest, ..)| floor * floor > nearest) {
                break;
            }
            for r in (row - ring).max(0)..=(row + ring).min(grid.rows - 1) {
                let rim = r == row - ring || r == row + ring;
                let step = if rim { 1 } else { 2 * ring.max(1) };
                for c in (col - ring..=col + ring).step_by(step as usize) {
                    if c < 0 || c >= grid.cols {
                        continue;
                    }
                    for &polygon in grid.polygons_in((r * grid.cols + c) as usize) {
                        for (i, piece) in self.live.pieces[polygon as usize].iter().enumerate() {
                            if !allow(piece) {
                                continue;
                            }
                            let point = piece.closest_point(p);
                            let found = (dist_sq(p, point), (polygon, i as u32), point);
                            if found.0 <= reach * reach && best.is_none_or(|best| found < best) {
                                best = Some(found);
                            }
                        }
                    }
                }
            }
        }
        best.map(|(_, node, point)| (node, point))
    }

    /// Gives every polygon its one whole piece and links them all.
    pub(crate) fn start_live(&mut self) {
        let count = self.polygon_count();
        self.live.on_polygon = vec![Vec::new(); count];
        self.live.pieces = (0..count)
            .map(|polygon| vec![self.whole(polygon)])
            .collect();
        for polygon in 0..count {
            self.relink(polygon);
        }
        self.number_islands();
    }

    /// A polygon as one piece, with nothing cut from it.
    fn whole(&self, polygon: usize) -> Piece {
        let edges = self.edges_of(polygon);
        let corners = edges.clone().map(|edge| self.edge_points(edge).0).collect();
        Piece::new(
            corners,
            edges.map(|edge| Tag::Base(edge as u32)).collect(),
            false,
        )
    }

    /// Records a checked obstacle and marks the polygons under it.
    fn place(&mut self, corners: &[Point], kind: ObstacleKind) -> ObstacleId {
        let shape = Piece::new(corners.to_vec(), Vec::new(), false);
        let (lo, hi) = (shape.lo, shape.hi);
        let mut polygons = Vec::new();
        if self.polygon_count() > 0 {
            let far = |p: Point| (i64::from(p.x), i64::from(p.y));
            let (cols, rows) = self.grid.span(far(lo), far(hi));
            for row in rows {
                for col in cols.clone() {
                    polygons.extend_from_slice(
                        self.grid.polygons_in((row * self.grid.cols + col) as usize),
                    );
                }
            }
            polygons.sort_unstable();
            polygons.dedup();
            polygons.retain(|&polygon| {
                let (box_lo, box_hi) = self.box_of(polygon as usize);
                box_lo.x < hi.x && lo.x < box_hi.x && box_lo.y < hi.y && lo.y < box_hi.y
            });
        }
        let id = self.live.next_id;
        self.live.next_id += 1;
        for &polygon in &polygons {
            // Ids only grow, so pushing keeps each list in order.
            self.live.on_polygon[polygon as usize].push(id);
        }
        self.live.dirty.extend_from_slice(&polygons);
        self.live.obstacles.insert(
            id,
            Obstacle {
                corners: corners.to_vec(),
                kind,
                lo,
                hi,
                polygons,
            },
        );
        ObstacleId(id)
    }

    /// Forgets an obstacle and marks the polygons that were under it.
    fn lift(&mut self, id: ObstacleId) -> bool {
        let Some(obstacle) = self.live.obstacles.remove(&id.0) else {
            return false;
        };
        for &polygon in &obstacle.polygons {
            self.live.on_polygon[polygon as usize].retain(|&other| other != id.0);
        }
        self.live.dirty.extend_from_slice(&obstacle.polygons);
        true
    }

    /// Brings the pieces up to date with the obstacles: re-cuts the marked
    /// polygons, then re-links them and the polygons round them.
    fn settle(&mut self) {
        let mut dirty = std::mem::take(&mut self.live.dirty);
        if dirty.is_empty() {
            return;
        }
        dirty.sort_unstable();
        dirty.dedup();
        for &polygon in &dirty {
            self.recut(polygon as usize);
        }
        let mut touched = dirty.clone();
        for &polygon in &dirty {
            let around = self
                .edges_of(polygon as usize)
                .filter_map(|edge| self.neighbor_across(edge));
            touched.extend(around.map(|other| other as u32));
        }
        touched.sort_unstable();
        touched.dedup();
        for &polygon in &touched {
            self.relink(polygon as usize);
        }
        self.number_islands();
    }

    /// Cuts every obstacle on a polygon out of it, in the order they were
    /// added.
    fn recut(&mut self, polygon: usize) {
        let mut pieces = vec![self.whole(polygon)];
        for id in &self.live.on_polygon[polygon] {
            pieces = cut_out(pieces, *id, &self.live.obstacles[id]);
        }
        self.live.pieces[polygon] = pieces;
    }

    /// Works out afresh what every piece of a polygon opens onto: the other
    /// pieces of the polygon, and the pieces of the polygons round it.
    fn relink(&mut self, polygon: usize) {
        let pieces = &self.live.pieces[polygon];
        let mut found: Vec<Vec<(u32, Link)>> = vec![Vec::new(); pieces.len()];

        // Inside the polygon, pieces meet along the cuts.
        let mut cuts: Vec<(Tag, u32, u32)> = Vec::new();
        for (i, piece) in pieces.iter().enumerate() {
            for (edge, &tag) in piece.tags.iter().enumerate() {
                if matches!(tag, Tag::Cut(..)) {
                    cuts.push((tag, i as u32, edge as u32));
                }
            }
        }
        cuts.sort_unstable();
        for group in cuts.chunk_by(|l, r| l.0 == r.0) {
            let Tag::Cut(id, side) = group[0].0 else {
                continue;
            };
            let shape = &self.live.obstacles[&id].corners;
            let line = (
                shape[side as usize],
                shape[(side as usize + 1) % shape.len()],
            );
            let spans: Vec<Span> = group
                .iter()
                .map(|&(_, i, edge)| Span::of(line, &pieces[i as usize], i, edge))
                .collect();
            for (a, x) in spans.iter().enumerate() {
                for y in &spans[a + 1..] {
                    if let Some((low, high)) = x.overlap(y) {
                        let (ahead, back) = if x.forward { (x, y) } else { (y, x) };
                        let there = (polygon as u32, back.piece);
                        found[ahead.piece as usize].push((
                            ahead.edge,
                            Link {
                                to: there,
                                from: low,
                                until: high,
                                avoid: pieces[back.piece as usize].avoid,
                            },
                        ));
                        let here = (polygon as u32, ahead.piece);
                        found[back.piece as usize].push((
                            back.edge,
                            Link {
                                to: here,
                                from: high,
                                until: low,
                                avoid: pieces[ahead.piece as usize].avoid,
                            },
                        ));
                    }
                }
            }
        }

        // Across each edge of the polygon, they meet the neighbour's pieces.
        for edge in self.edges_of(polygon) {
            let Some(twin) = self.twin_of(edge) else {
                continue;
            };
            let other = self.owner_of(twin);
            let line = self.edge_points(edge);
            let along = |pieces: &[Piece], tag: Tag| -> Vec<Span> {
                let mut spans = Vec::new();
                for (i, piece) in pieces.iter().enumerate() {
                    for (at, _) in piece.tags.iter().enumerate().filter(|&(_, &t)| t == tag) {
                        spans.push(Span::of(line, piece, i as u32, at as u32));
                    }
                }
                spans
            };
            let mine = along(pieces, Tag::Base(edge as u32));
            let theirs = along(&self.live.pieces[other], Tag::Base(twin as u32));
            for x in mine.iter().filter(|x| x.forward) {
                for y in theirs.iter().filter(|y| !y.forward) {
                    if let Some((low, high)) = x.overlap(y) {
                        let there = (other as u32, y.piece);
                        found[x.piece as usize].push((
                            x.edge,
                            Link {
                                to: there,
                                from: low,
                                until: high,
                                avoid: self.live.pieces[other][y.piece as usize].avoid,
                            },
                        ));
                    }
                }
            }
        }

        for (i, links) in found.into_iter().enumerate() {
            let merged = merge(links, &self.live.pieces[polygon][i].corners);
            self.live.pieces[polygon][i].links = merged;
        }
    }

    /// Numbers the connected groups of ordinary ground. Avoided ground is in
    /// none: a route may leave it but not cross it.
    fn number_islands(&mut self) {
        let live = &mut self.live;
        live.first.clear();
        let mut total = 0;
        for pieces in &mut live.pieces {
            live.first.push(total);
            total += pieces.len() as u32;
            pieces.iter_mut().for_each(|piece| piece.island = NO_ISLAND);
        }
        live.first.push(total);

        let mut islands = 0;
        let mut pending: Vec<Node> = Vec::new();
        for polygon in 0..live.pieces.len() {
            for i in 0..live.pieces[polygon].len() {
                let seed = &mut live.pieces[polygon][i];
                if seed.avoid || seed.island != NO_ISLAND {
                    continue;
                }
                seed.island = islands;
                pending.push((polygon as u32, i as u32));
                while let Some(node) = pending.pop() {
                    for k in 0..live.piece(node).links.len() {
                        let next = live.piece(node).links[k].to;
                        let piece = &mut live.pieces[next.0 as usize][next.1 as usize];
                        if !piece.avoid && piece.island == NO_ISLAND {
                            piece.island = islands;
                            pending.push(next);
                        }
                    }
                }
                islands += 1;
            }
        }
    }
}

/// One edge of a piece, measured along the line it lies on.
struct Span {
    piece: u32,
    edge: u32,
    from: Point,
    until: Point,
    /// Where the edge starts and ends along the line.
    low: i64,
    high: i64,
    /// Whether the edge runs the way the line does.
    forward: bool,
}

impl Span {
    fn of(line: (Point, Point), piece: &Piece, index: u32, edge: u32) -> Self {
        let (from, until) = piece.edge(edge as usize);
        let (start, end) = (dot(line.0, line.1, from), dot(line.0, line.1, until));
        let forward = start < end;
        let (low, high) = if forward { (start, end) } else { (end, start) };
        Self {
            piece: index,
            edge,
            from,
            until,
            low,
            high,
            forward,
        }
    }

    /// The stretch two edges on one line share, if they face each other and
    /// overlap by more than a point: its ends, lower first.
    fn overlap(&self, other: &Span) -> Option<(Point, Point)> {
        if self.forward == other.forward {
            return None;
        }
        if self.low.max(other.low) >= self.high.min(other.high) {
            return None;
        }
        // Each end of the overlap is an end of one of the two edges. Where
        // both end level, take the lesser point so that the two sides agree.
        let ends = |span: &Span| {
            if span.forward {
                (span.from, span.until)
            } else {
                (span.until, span.from)
            }
        };
        let ((my_low, my_high), (their_low, their_high)) = (ends(self), ends(other));
        let low = match self.low.cmp(&other.low) {
            std::cmp::Ordering::Greater => my_low,
            std::cmp::Ordering::Less => their_low,
            std::cmp::Ordering::Equal => my_low.min(their_low),
        };
        let high = match self.high.cmp(&other.high) {
            std::cmp::Ordering::Less => my_high,
            std::cmp::Ordering::Greater => their_high,
            std::cmp::Ordering::Equal => my_high.min(their_high),
        };
        Some((low, high))
    }
}

/// Puts a piece's links in order round it and joins those that open onto
/// the same piece end to end. Two convex pieces meet along one straight
/// stretch, however many edges it is divided into.
fn merge(mut links: Vec<(u32, Link)>, corners: &[Point]) -> Vec<Link> {
    links.sort_by_key(|&(edge, link)| (edge, dist_sq(corners[edge as usize], link.from)));
    let mut merged: Vec<Link> = Vec::with_capacity(links.len());
    for (_, link) in links {
        match merged.last_mut() {
            Some(last) if last.to == link.to && last.until == link.from => last.until = link.until,
            _ => merged.push(link),
        }
    }
    if merged.len() > 1 {
        let (first, last) = (merged[0], merged[merged.len() - 1]);
        if first.to == last.to && last.until == first.from {
            merged[0].from = last.from;
            merged.pop();
        }
    }
    merged
}

/// Cuts one obstacle out of a set of pieces.
///
/// Each piece the obstacle overlaps is peeled along the obstacle's edges in
/// turn: what lies outside an edge is kept as a piece of its own, and what
/// lies inside goes on to the next edge. What is left after the last edge
/// is the part under the obstacle.
fn cut_out(pieces: Vec<Piece>, id: u32, obstacle: &Obstacle) -> Vec<Piece> {
    let mut kept = Vec::with_capacity(pieces.len() + obstacle.corners.len());
    for piece in pieces {
        let apart = piece.hi.x <= obstacle.lo.x
            || obstacle.hi.x <= piece.lo.x
            || piece.hi.y <= obstacle.lo.y
            || obstacle.hi.y <= piece.lo.y;
        if apart {
            kept.push(piece);
            continue;
        }
        let mut peeled = Vec::new();
        let mut inside = Some((piece.corners.clone(), piece.tags.clone()));
        for side in 0..obstacle.corners.len() {
            let Some((corners, tags)) = inside.take() else {
                break;
            };
            let (a, b) = (
                obstacle.corners[side],
                obstacle.corners[(side + 1) % obstacle.corners.len()],
            );
            let (left, right) = split(&corners, &tags, a, b, Tag::Cut(id, side as u32));
            peeled.extend(right);
            inside = left;
        }
        match inside {
            // The obstacle misses this piece after all: leave it whole.
            None => kept.push(piece),
            Some((corners, tags)) => {
                kept.extend(
                    peeled
                        .into_iter()
                        .map(|(c, t)| Piece::new(c, t, piece.avoid)),
                );
                if obstacle.kind == ObstacleKind::Avoid {
                    kept.push(Piece::new(corners, tags, true));
                }
            }
        }
    }
    kept
}

type Shape = (Vec<Point>, Vec<Tag>);

/// Splits a convex shape along the line through `a` and `b` into the part
/// on the line's left and the part on its right. The new edge along the
/// line is tagged `cut` on both; every other edge keeps its tag.
fn split(
    corners: &[Point],
    tags: &[Tag],
    a: Point,
    b: Point,
    cut: Tag,
) -> (Option<Shape>, Option<Shape>) {
    let sides: Vec<i64> = corners.iter().map(|&v| cross(a, b, v)).collect();
    if sides.iter().all(|&side| side >= 0) {
        return (Some((corners.to_vec(), tags.to_vec())), None);
    }
    if sides.iter().all(|&side| side <= 0) {
        return (None, Some((corners.to_vec(), tags.to_vec())));
    }
    let (mut left, mut right): (Shape, Shape) = Default::default();
    let push = |shape: &mut Shape, corner: Point, tag: Tag| {
        shape.0.push(corner);
        shape.1.push(tag);
    };
    let n = corners.len();
    for i in 0..n {
        let (v, w) = (corners[i], corners[(i + 1) % n]);
        let (here, next) = (sides[i], sides[(i + 1) % n]);
        let crosses = (here > 0 && next < 0) || (here < 0 && next > 0);
        let crossing = crosses.then(|| split_point(v, w, here, next));
        // Each part keeps the corners on its side and on the line. Where
        // the boundary leaves a part, that part's next edge is the cut.
        for (shape, here, next) in [(&mut left, here, next), (&mut right, -here, -next)] {
            if here >= 0 {
                if next >= 0 {
                    push(shape, v, tags[i]);
                } else if here == 0 {
                    push(shape, v, cut);
                } else {
                    push(shape, v, tags[i]);
                    push(shape, crossing.expect("the edge crosses the line"), cut);
                }
            } else if next > 0 {
                push(shape, crossing.expect("the edge crosses the line"), tags[i]);
            }
        }
    }
    (tidy(left), tidy(right))
}

/// Drops edges that rounding left with no length, and the shape itself if
/// it is left with no area.
fn tidy((mut corners, mut tags): Shape) -> Option<Shape> {
    let mut i = 0;
    while corners.len() >= 3 && i < corners.len() {
        if corners[i] == corners[(i + 1) % corners.len()] {
            corners.remove(i);
            tags.remove(i);
        } else {
            i += 1;
        }
    }
    let doubled_area: i128 = (1..corners.len().saturating_sub(1))
        .map(|i| i128::from(cross(corners[0], corners[i], corners[i + 1])))
        .sum();
    (corners.len() >= 3 && doubled_area > 0).then_some((corners, tags))
}
