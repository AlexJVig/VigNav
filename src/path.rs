//! Path queries: A* picks a corridor of polygons, then the funnel algorithm
//! pulls the route taut inside it.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::fmt;

use crate::geom::{Coord, Point, cross, crossing_point, dist};
use crate::mesh::{Location, NavMesh, PolyId};

/// A route across the mesh.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Path {
    /// The corners of the route, from the start to the goal inclusive. No
    /// two consecutive points are equal, so a route that starts on its goal
    /// is a single point. Every point between the first and the last is a
    /// corner of the mesh.
    pub points: Vec<Point>,
    /// The polygons the route crosses, in order.
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
    /// The start and the goal are on parts of the mesh that do not connect.
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

impl NavMesh {
    /// Finds a route between two points, first moving each onto the mesh if
    /// it is within `snap` millimetres of it.
    ///
    /// The returned route starts and ends at the moved points.
    pub fn find_path(&self, start: Point, goal: Point, snap: Coord) -> Result<Path, PathError> {
        let start = self.nearest(start, snap).ok_or(PathError::StartOffMesh)?;
        let goal = self.nearest(goal, snap).ok_or(PathError::GoalOffMesh)?;
        self.find_path_between(start, goal)
            .ok_or(PathError::Unreachable)
    }

    /// Finds a route between two points whose polygons are already known,
    /// or `None` if they do not connect.
    ///
    /// Use this where the caller tracks which polygon a unit is on, and
    /// wherever polygons overlap and [`locate`](NavMesh::locate) cannot tell
    /// them apart.
    pub fn find_path_between(&self, start: Location, goal: Location) -> Option<Path> {
        let (corridor, crossed) = self.corridor(start, goal)?;
        // Seen from inside a counter-clockwise polygon, a stretch of its
        // boundary being crossed has its end on the left and its start on
        // the right.
        let portals: Vec<(Point, Point)> = crossed
            .iter()
            .map(|&edge| {
                let (from, to) = self.portal(edge);
                (to, from)
            })
            .collect();
        let points = pull_taut(start.point, goal.point, &portals);
        Some(Path { points, corridor })
    }

    /// A* over polygons. Returns the polygons from start to goal and the
    /// edge crossed between each consecutive pair.
    ///
    /// Each polygon is entered where the shortest route from the previous
    /// entry point to the goal crosses the shared boundary. That is exact
    /// while the goal is in sight and a close estimate otherwise, so the
    /// corridor is good rather than provably shortest.
    fn corridor(&self, start: Location, goal: Location) -> Option<(Vec<PolyId>, Vec<usize>)> {
        let count = self.polygon_count();
        let (first, last) = (start.poly.index(), goal.poly.index());
        if first >= count || last >= count || !start.point.in_range() || !goal.point.in_range() {
            return None;
        }
        if self.island(start.poly) != self.island(goal.poly) {
            return None;
        }
        if first == last {
            return Some((vec![start.poly], Vec::new()));
        }

        let mut cost = vec![i64::MAX; count];
        let mut entered_by = vec![usize::MAX; count];
        let mut entry = vec![start.point; count];
        let mut closed = vec![false; count];
        // Ordered by estimated total, then by distance left, then by id, so
        // ties always break the same way.
        let mut open = BinaryHeap::new();
        cost[first] = 0;
        let remaining = dist(start.point, goal.point);
        open.push(Reverse((remaining, remaining, first)));

        while let Some(Reverse((_, _, polygon))) = open.pop() {
            if closed[polygon] {
                continue;
            }
            closed[polygon] = true;
            if polygon == last {
                break;
            }
            for edge in self.edges_of(polygon) {
                let Some(next) = self.neighbor_across(edge) else {
                    continue;
                };
                if closed[next] {
                    continue;
                }
                let (a, b) = self.portal(edge);
                let crossing = crossing_point(entry[polygon], goal.point, a, b);
                let reached = cost[polygon] + dist(entry[polygon], crossing);
                if reached < cost[next] {
                    cost[next] = reached;
                    entered_by[next] = edge;
                    entry[next] = crossing;
                    let remaining = dist(crossing, goal.point);
                    open.push(Reverse((reached + remaining, remaining, next)));
                }
            }
        }
        if !closed[last] {
            return None;
        }

        let mut polygons = vec![goal.poly];
        let mut crossed = Vec::new();
        let mut polygon = last;
        while polygon != first {
            let edge = entered_by[polygon];
            crossed.push(edge);
            polygon = self.owner_of(edge);
            polygons.push(PolyId(polygon as u32));
        }
        polygons.reverse();
        crossed.reverse();
        Some((polygons, crossed))
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
