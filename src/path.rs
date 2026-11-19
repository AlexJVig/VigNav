//! Path queries. A straight walk is tried first. Failing that, A* picks a
//! corridor of pieces, the funnel algorithm pulls the route taut inside it,
//! and corners the route can see past are dropped.

use std::cell::RefCell;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::fmt;

use crate::geom::{Coord, Point, cross, crossing_point, dist, dot};
use crate::live::{Node, Piece};
use crate::mesh::{CONVEX_SLACK, Location, NavMesh, PolyId};

/// A route across the mesh.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Path {
    /// The corners of the route, from the start to the goal inclusive. No
    /// two consecutive points are equal, so a route that starts on its goal
    /// is a single point. Every point between the first and the last is a
    /// corner of the mesh or of an obstacle.
    pub points: Vec<Point>,
    /// The polygons the route crosses or touches, in order. Each opens onto
    /// the next.
    pub corridor: Vec<PolyId>,
}

impl Path {
    /// Length of the route in millimetres, each leg rounded down.
    pub fn length(&self) -> i64 {
        self.points.windows(2).map(|leg| dist(leg[0], leg[1])).sum()
    }
}

/// Why no path was returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathError {
    /// The start is further from the mesh than the snap distance.
    StartOffMesh,
    /// The goal is further from the mesh than the snap distance.
    GoalOffMesh,
    /// The start and the goal are on ground that does not connect.
    Unreachable,
}

impl fmt::Display for PathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::StartOffMesh => "the start is off the mesh",
            Self::GoalOffMesh => "the goal is off the mesh",
            Self::Unreachable => "the goal cannot be reached from the start",
        })
    }
}

impl std::error::Error for PathError {}

/// How far off the mesh a point may be and still count as on it. Rounding
/// to the millimetre can leave a point this far outside the piece it
/// belongs to.
const TOUCHING: Coord = CONVEX_SLACK as Coord;

/// The most pieces a straight walk will look at before giving up. A walk
/// that gives up is treated as blocked, and the search takes over.
const TRAIL_LIMIT: usize = 2048;

/// What A* knows about a piece it has reached.
#[derive(Clone, Copy)]
struct Reached {
    cost: i64,
    closed: bool,
    /// The piece it was reached from.
    parent: Node,
    /// The opening it was entered by, left end first as the route sees it.
    gate: (Point, Point),
    /// Where the route is taken to enter it.
    entry: Point,
}

/// Working memory for a thread's queries. It is kept between them, so a
/// query costs what it touches and not the size of the mesh.
#[derive(Default)]
struct Scratch {
    /// The query that last wrote each slot of `state`.
    stamp: Vec<u32>,
    query: u32,
    state: Vec<Reached>,
    open: BinaryHeap<Reverse<(i64, i64, Node)>>,
    /// The pieces a straight walk has reached, each with the place in this
    /// list of the piece it was reached from.
    trail: Vec<(Node, u32)>,
    /// The walk that last reached each piece.
    walked: Vec<u32>,
    walk: u32,
}

impl Scratch {
    /// Makes ready for a straight walk over `total` pieces.
    fn begin_walk(&mut self, total: usize, root: Node) {
        if self.walked.len() < total {
            self.walked.resize(total, 0);
        }
        self.walk = self.walk.wrapping_add(1);
        if self.walk == 0 {
            self.walked.fill(0);
            self.walk = 1;
        }
        self.trail.clear();
        self.trail.push((root, u32::MAX));
    }

    /// Makes ready for a search over `total` pieces without clearing them.
    fn begin(&mut self, total: usize, start: Node, at: Point) {
        if self.stamp.len() < total {
            let blank = Reached {
                cost: 0,
                closed: false,
                parent: start,
                gate: (at, at),
                entry: at,
            };
            self.stamp.resize(total, 0);
            self.state.resize(total, blank);
        }
        self.query = self.query.wrapping_add(1);
        if self.query == 0 {
            self.stamp.fill(0);
            self.query = 1;
        }
        self.open.clear();
    }
}

thread_local! {
    static SCRATCH: RefCell<Scratch> = RefCell::new(Scratch::default());
}

/// Where a straight walk is trying to get to.
#[derive(Clone, Copy)]
enum Arrive {
    /// This piece.
    At(Node),
    /// Any piece with this point in or on it.
    Around(Point),
}

impl Path {
    fn straight(start: Point, goal: Point, corridor: Vec<PolyId>) -> Self {
        let points = if start == goal {
            vec![start]
        } else {
            vec![start, goal]
        };
        Self { points, corridor }
    }
}

impl NavMesh {
    /// Finds a route between two points, first moving each onto the mesh if
    /// it is within `snap` millimetres of it.
    ///
    /// The returned route starts and ends at the moved points. A goal on
    /// ground to be avoided is moved to the nearest ordinary ground the
    /// start can reach.
    pub fn find_path(&self, start: Point, goal: Point, snap: Coord) -> Result<Path, PathError> {
        let reach = snap.max(TOUCHING);
        let (from, start) = self.snapped(start, reach).ok_or(PathError::StartOffMesh)?;
        let (to, goal) = self.snapped(goal, reach).ok_or(PathError::GoalOffMesh)?;
        self.route(from, start, to, goal)
            .ok_or(PathError::Unreachable)
    }

    /// Finds a route from `start` to the point nearest `goal` that the
    /// start can reach, if there is one within `reach` millimetres of the
    /// goal. The start is moved onto the mesh within the same distance.
    ///
    /// Use this where a unit ordered to somewhere it cannot go should get as
    /// near as it can: a goal inside a building or across a gap.
    pub fn find_path_toward(
        &self,
        start: Point,
        goal: Point,
        reach: Coord,
    ) -> Result<Path, PathError> {
        let reach = reach.max(TOUCHING);
        let (from, start) = self.snapped(start, reach).ok_or(PathError::StartOffMesh)?;
        let islands = self.outlets(from).0;
        let open = |piece: &Piece| !piece.avoid && islands.contains(&piece.island);
        let (to, goal) = self
            .nearest_piece(goal, reach, open)
            .ok_or(PathError::GoalOffMesh)?;
        self.route(from, start, to, goal)
            .ok_or(PathError::Unreachable)
    }

    /// Finds a route between two points whose polygons are already known,
    /// or `None` if they do not connect or either is under a blocking
    /// obstacle.
    ///
    /// Use this where the caller tracks which polygon a unit is on, and
    /// wherever polygons overlap and [`locate`](NavMesh::locate) cannot tell
    /// them apart.
    pub fn find_path_between(&self, start: Location, goal: Location) -> Option<Path> {
        let from = self.piece_in(start.poly.index(), start.point)?;
        let to = self.piece_in(goal.poly.index(), goal.point)?;
        self.route(from, start.point, to, goal.point)
    }

    /// The point nearest to `p`, within `max_dist` millimetres, that a route
    /// from `from` can reach.
    pub fn nearest_reachable(&self, from: Location, p: Point, max_dist: Coord) -> Option<Location> {
        let node = self.piece_in(from.poly.index(), from.point)?;
        let islands = self.outlets(node).0;
        let open = |piece: &Piece| !piece.avoid && islands.contains(&piece.island);
        let ((polygon, _), point) = self.nearest_piece(p, max_dist, open)?;
        Some(Location {
            poly: PolyId(polygon),
            point,
        })
    }

    /// Whether a unit at `from` can walk straight to `to`. Returns where it
    /// arrives, which is `to` with the polygon it is on, or `None` if
    /// anything is in the way or `to` is off the mesh.
    ///
    /// Touching a wall or a corner is allowed; squeezing between two
    /// obstacles that meet at a point is not.
    pub fn line_of_sight(&self, from: Location, to: Point) -> Option<Location> {
        let root = self.piece_in(from.poly.index(), from.point)?;
        if !to.in_range() {
            return None;
        }
        SCRATCH.with_borrow_mut(|scratch| {
            let end = self.walk(scratch, root, from.point, to, Arrive::Around(to))?;
            Some(Location {
                poly: PolyId(scratch.trail[end].0.0),
                point: to,
            })
        })
    }

    /// The piece `p` is on, or the nearest point of the mesh within `reach`.
    fn snapped(&self, p: Point, reach: Coord) -> Option<(Node, Point)> {
        match self.piece_at(p) {
            Some(node) => Some((node, p)),
            None => self.nearest_piece(p, reach, |_| true),
        }
    }

    /// The islands a route starting on this piece can reach, and the
    /// avoided ground it may cross on the way out.
    fn outlets(&self, from: Node) -> (Vec<u32>, Vec<Node>) {
        let piece = self.live.piece(from);
        if !piece.avoid {
            return (vec![piece.island], Vec::new());
        }
        let (mut islands, mut ground) = (Vec::new(), vec![from]);
        let mut next = 0;
        while next < ground.len() {
            for link in &self.live.piece(ground[next]).links {
                let other = self.live.piece(link.to);
                if !other.avoid {
                    if !islands.contains(&other.island) {
                        islands.push(other.island);
                    }
                } else if !ground.contains(&link.to) {
                    ground.push(link.to);
                }
            }
            next += 1;
        }
        (islands, ground)
    }

    /// A route between two points on known pieces.
    fn route(&self, from: Node, start: Point, to: Node, goal: Point) -> Option<Path> {
        let (first, last) = (self.live.piece(from), self.live.piece(to));
        let (to, goal) = if first.avoid || last.avoid {
            let (islands, ground) = self.outlets(from);
            if last.avoid && !ground.contains(&to) {
                // The goal is on ground this route may not enter: stop at
                // the nearest ordinary ground instead.
                let open = |piece: &Piece| !piece.avoid && islands.contains(&piece.island);
                self.nearest_piece(goal, Coord::MAX, open)?
            } else if !last.avoid && !islands.contains(&last.island) {
                return None;
            } else {
                (to, goal)
            }
        } else if first.island != last.island {
            return None;
        } else {
            (to, goal)
        };
        if from == to {
            return Some(Path::straight(start, goal, vec![PolyId(from.0)]));
        }
        SCRATCH.with_borrow_mut(|scratch| {
            if let Some(end) = self.walk(scratch, from, start, goal, Arrive::At(to)) {
                let mut corridor = Vec::new();
                extend_corridor(&mut corridor, &scratch.trail, end);
                return Some(Path::straight(start, goal, corridor));
            }
            let (gates, chain) = self.search(scratch, from, start, to, goal)?;
            let points = pull_taut(start, goal, &gates);
            Some(self.shorten(scratch, points, chain))
        })
    }

    /// Follows the straight line from `s`, on the piece `root`, to `t`,
    /// stepping from piece to piece through the openings the line passes
    /// through. Returns the place in `trail` of the piece arrived in.
    ///
    /// Every opening stepped through holds a point of the line, and a
    /// convex piece holds the line between any two of its points, so the
    /// pieces on the way to the arrival cover the line without a gap. An
    /// opening is closed at both ends: the line may run along a wall or
    /// touch a corner, and the walk then goes round that corner through
    /// whatever pieces meet there.
    fn walk(
        &self,
        scratch: &mut Scratch,
        root: Node,
        s: Point,
        t: Point,
        arrive: Arrive,
    ) -> Option<usize> {
        let live = &self.live;
        scratch.begin_walk(live.total(), root);
        scratch.walked[live.slot(root)] = scratch.walk;
        let mut head = 0;
        while head < scratch.trail.len() {
            let node = scratch.trail[head].0;
            let piece = live.piece(node);
            let there = match arrive {
                Arrive::At(goal) => node == goal,
                Arrive::Around(point) => piece.contains(point),
            };
            if there {
                return Some(head);
            }
            for link in piece.links.iter().filter(|_| s != t) {
                let slot = live.slot(link.to);
                if scratch.walked[slot] == scratch.walk {
                    continue;
                }
                if link.avoid && !piece.avoid {
                    continue;
                }
                // Seen from inside, an opening starts on the right of
                // anything leaving through it and ends on the left.
                if cross(s, t, link.from) > 0 || cross(s, t, link.until) < 0 {
                    continue;
                }
                // It must also lie between the two ends of the line.
                let before = |p: Point| dot(s, t, p) < 0;
                let beyond = |p: Point| dot(t, s, p) < 0;
                if (before(link.from) && before(link.until))
                    || (beyond(link.from) && beyond(link.until))
                {
                    continue;
                }
                if scratch.trail.len() == TRAIL_LIMIT {
                    return None;
                }
                scratch.walked[slot] = scratch.walk;
                scratch.trail.push((link.to, head as u32));
            }
            head += 1;
        }
        None
    }

    /// A* over pieces. Returns the openings from start to goal, left end
    /// first, and the pieces they lead through.
    ///
    /// Each piece is entered where the shortest route from the previous
    /// entry point to the goal crosses the opening into it. That is exact
    /// while the goal is in sight and a close estimate otherwise, so the
    /// corridor is good rather than provably shortest.
    #[allow(clippy::type_complexity)]
    fn search(
        &self,
        scratch: &mut Scratch,
        from: Node,
        start: Point,
        to: Node,
        goal: Point,
    ) -> Option<(Vec<(Point, Point)>, Vec<Node>)> {
        let live = &self.live;
        scratch.begin(live.total(), from, start);
        let first = live.slot(from);
        scratch.stamp[first] = scratch.query;
        scratch.state[first] = Reached {
            cost: 0,
            closed: false,
            parent: from,
            gate: (start, start),
            entry: start,
        };
        // Ordered by estimated total, then by distance left, then by piece,
        // so ties always break the same way.
        let remaining = dist(start, goal);
        scratch.open.push(Reverse((remaining, remaining, from)));

        let mut arrived = false;
        while let Some(Reverse((_, _, node))) = scratch.open.pop() {
            let slot = live.slot(node);
            if scratch.state[slot].closed {
                continue;
            }
            scratch.state[slot].closed = true;
            if node == to {
                arrived = true;
                break;
            }
            let Reached { cost, entry, .. } = scratch.state[slot];
            let piece = live.piece(node);
            for link in &piece.links {
                if link.avoid && !piece.avoid {
                    continue;
                }
                let at = live.slot(link.to);
                let known = scratch.stamp[at] == scratch.query;
                if known && scratch.state[at].closed {
                    continue;
                }
                let crossing = crossing_point(entry, goal, link.from, link.until);
                let reached = cost + dist(entry, crossing);
                if !known || reached < scratch.state[at].cost {
                    scratch.stamp[at] = scratch.query;
                    scratch.state[at] = Reached {
                        cost: reached,
                        closed: false,
                        parent: node,
                        gate: (link.until, link.from),
                        entry: crossing,
                    };
                    let remaining = dist(crossing, goal);
                    scratch
                        .open
                        .push(Reverse((reached + remaining, remaining, link.to)));
                }
            }
        }
        if !arrived {
            return None;
        }

        let mut gates = Vec::new();
        let mut chain = vec![to];
        let mut node = to;
        while node != from {
            let reached = scratch.state[live.slot(node)];
            gates.push(reached.gate);
            node = reached.parent;
            chain.push(node);
        }
        gates.reverse();
        chain.reverse();
        Some((gates, chain))
    }

    /// Drops every corner of a taut route that the corner before it can see
    /// past, and lists the polygons the result crosses.
    ///
    /// The taut route is the shortest inside the corridor A* chose. Where a
    /// straight line reaches further than the corridor allowed, the corridor
    /// was the wrong one there and the line is better.
    fn shorten(&self, scratch: &mut Scratch, taut: Vec<Point>, chain: Vec<Node>) -> Path {
        let (from, to) = (chain[0], chain[chain.len() - 1]);
        let as_found = |points: Vec<Point>| {
            let mut corridor: Vec<PolyId> = chain.iter().map(|node| PolyId(node.0)).collect();
            corridor.dedup();
            Path { points, corridor }
        };
        let arrive = |points: &[Point], k: usize| {
            if k + 1 == points.len() {
                Arrive::At(to)
            } else {
                Arrive::Around(points[k])
            }
        };

        // A route with one corner cannot lose it: the straight line from
        // start to goal has already been tried.
        let mut points = taut.clone();
        let mut dropped = false;
        let (mut i, mut along) = (0, 0);
        while points.len() > 3 && i + 2 < points.len() {
            // Stand on a piece of the corridor that has this corner on it.
            while along + 1 < chain.len() && !self.live.piece(chain[along]).contains(points[i]) {
                along += 1;
            }
            let past = arrive(&points, i + 2);
            if self
                .walk(scratch, chain[along], points[i], points[i + 2], past)
                .is_some()
            {
                points.remove(i + 1);
                dropped = true;
            } else {
                i += 1;
            }
        }
        if !dropped {
            return as_found(taut);
        }

        // The route has left the corridor, so find what it crosses now.
        let mut corridor = vec![PolyId(from.0)];
        let mut root = from;
        for i in 0..points.len() - 1 {
            let next = arrive(&points, i + 1);
            let Some(end) = self.walk(scratch, root, points[i], points[i + 1], next) else {
                // Every leg was walked once already. If rounding now says
                // otherwise, keep the route as A* found it.
                return as_found(taut);
            };
            root = scratch.trail[end].0;
            extend_corridor(&mut corridor, &scratch.trail, end);
        }
        Path { points, corridor }
    }
}

/// Adds the polygons a straight walk passed through, from its start to the
/// piece at `end`, leaving out any repeat of the polygon before.
fn extend_corridor(corridor: &mut Vec<PolyId>, trail: &[(Node, u32)], end: usize) {
    let mut walked = Vec::new();
    let mut at = end as u32;
    while at != u32::MAX {
        let (node, parent) = trail[at as usize];
        walked.push(PolyId(node.0));
        at = parent;
    }
    for &poly in walked.iter().rev() {
        if corridor.last() != Some(&poly) {
            corridor.push(poly);
        }
    }
}

/// The funnel algorithm: the shortest route from `start` to `goal` that
/// passes through every portal in order. Each portal is `(left, right)` as
/// seen by someone walking the route.
fn pull_taut(start: Point, goal: Point, portals: &[(Point, Point)]) -> Vec<Point> {
    let mut gates = Vec::with_capacity(portals.len() + 2);
    gates.push((start, start));
    gates.extend_from_slice(portals);
    gates.push((goal, goal));

    let mut points = vec![start];
    let mut push = |p: Point| {
        if points.last() != Some(&p) {
            points.push(p);
        }
    };

    // The funnel is the wedge at `apex` between the rays to `right` and to
    // `left`. Each gate narrows it; when one side crosses the other, the
    // side that was crossed is a corner of the route and the next apex.
    let (mut apex, mut left, mut right) = (start, start, start);
    let (mut left_at, mut right_at) = (0, 0);
    let mut i = 1;
    while i < gates.len() {
        let (new_left, new_right) = gates[i];

        if cross(apex, right, new_right) >= 0 {
            if apex == right || cross(apex, left, new_right) < 0 {
                right = new_right;
                right_at = i;
            } else {
                push(left);
                apex = left;
                i = left_at + 1;
                (left, right, left_at, right_at) = (apex, apex, left_at, left_at);
                continue;
            }
        }

        if cross(apex, left, new_left) <= 0 {
            if apex == left || cross(apex, right, new_left) > 0 {
                left = new_left;
                left_at = i;
            } else {
                push(right);
                apex = right;
                i = right_at + 1;
                (left, right, left_at, right_at) = (apex, apex, right_at, right_at);
                continue;
            }
        }

        i += 1;
    }
    push(goal);
    points
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: Coord, y: Coord) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn no_portals_is_a_straight_line() {
        assert_eq!(pull_taut(p(0, 0), p(5, 5), &[]), [p(0, 0), p(5, 5)]);
        assert_eq!(pull_taut(p(3, 3), p(3, 3), &[]), [p(3, 3)]);
    }

    #[test]
    fn a_visible_goal_ignores_the_portals() {
        // Heading in +x, so left is +y.
        let portals = [(p(10, 5), p(10, -5)), (p(20, 5), p(20, -5))];
        assert_eq!(pull_taut(p(0, 0), p(30, 0), &portals), [p(0, 0), p(30, 0)]);
    }

    #[test]
    fn the_route_wraps_each_corner_it_has_to_pass() {
        // The second portal sits wholly to the left of the first, and the
        // goal is below the line from the first corner through the second
        // portal, so the route touches a left corner and then a right one.
        let portals = [(p(10, 5), p(10, -5)), (p(20, 30), p(20, 20))];
        let route = pull_taut(p(0, 0), p(30, 25), &portals);
        assert_eq!(route, [p(0, 0), p(10, 5), p(20, 20), p(30, 25)]);

        // And mirrored.
        let portals = [(p(10, 5), p(10, -5)), (p(20, -20), p(20, -30))];
        let route = pull_taut(p(0, 0), p(30, -25), &portals);
        assert_eq!(route, [p(0, 0), p(10, -5), p(20, -20), p(30, -25)]);
    }

    #[test]
    fn portals_fanning_round_one_corner_add_it_once() {
        // Three portals share the left point (10, 10), as when a route turns
        // left through a fan of polygons.
        let hub = p(10, 10);
        let portals = [(hub, p(10, 0)), (hub, p(20, 10)), (hub, p(10, 20))];
        let route = pull_taut(p(0, 5), p(0, 15), &portals);
        assert_eq!(route, [p(0, 5), hub, p(0, 15)]);
    }
}
