//! Integer geometry on the navigation plane.
//!
//! Coordinates are whole millimetres (1 m = 1000). The navigation plane is
//! the host game's ground plane (x/z in a Y-up world); height is not part of
//! a [`Point`].

use std::cmp::Ordering;

/// One coordinate on the navigation plane, in millimetres.
pub type Coord = i32;

/// Millimetres per metre.
pub const MM_PER_M: Coord = 1000;

/// Every coordinate must satisfy `|c| <= COORD_LIMIT` (about 537 km).
///
/// The difference of two in-range coordinates is then at most 2^30 in
/// magnitude, so the products in this module cannot overflow an `i64`.
pub const COORD_LIMIT: Coord = 1 << 29;

/// A point on the navigation plane.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Point {
    /// Millimetres along the first ground axis.
    pub x: Coord,
    /// Millimetres along the second ground axis.
    pub y: Coord,
}

impl Point {
    /// A point from millimetre coordinates.
    pub const fn new(x: Coord, y: Coord) -> Self {
        Self { x, y }
    }

    /// Whether both coordinates are within [`COORD_LIMIT`].
    pub const fn in_range(self) -> bool {
        let limit = COORD_LIMIT.unsigned_abs();
        self.x.unsigned_abs() <= limit && self.y.unsigned_abs() <= limit
    }
}

/// Which way three points turn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    /// `a -> b -> c` turns left. Polygons are wound this way.
    CounterClockwise,
    /// The three points lie on one line.
    Collinear,
    /// `a -> b -> c` turns right.
    Clockwise,
}

/// Twice the signed area of the triangle `a`, `b`, `c`: positive when the
/// points turn counter-clockwise, zero when they are collinear.
///
/// Exact for in-range points (see [`COORD_LIMIT`]).
pub fn cross(a: Point, b: Point, c: Point) -> i64 {
    let abx = i64::from(b.x) - i64::from(a.x);
    let aby = i64::from(b.y) - i64::from(a.y);
    let acx = i64::from(c.x) - i64::from(a.x);
    let acy = i64::from(c.y) - i64::from(a.y);
    abx * acy - aby * acx
}

/// The turn direction of `a -> b -> c`, decided exactly.
pub fn orientation(a: Point, b: Point, c: Point) -> Orientation {
    match cross(a, b, c).cmp(&0) {
        Ordering::Greater => Orientation::CounterClockwise,
        Ordering::Equal => Orientation::Collinear,
        Ordering::Less => Orientation::Clockwise,
    }
}

/// Squared distance between two points, in mm². Exact for in-range points.
pub fn dist_sq(a: Point, b: Point) -> i64 {
    let dx = i64::from(b.x) - i64::from(a.x);
    let dy = i64::from(b.y) - i64::from(a.y);
    dx * dx + dy * dy
}

/// Distance between two points in whole millimetres, rounded down.
pub fn dist(a: Point, b: Point) -> i64 {
    dist_sq(a, b).isqrt()
}

/// Where the shortest route from `from` to `to` that touches the segment
/// `a`-`b` touches it, rounded to the nearest millimetre on each axis.
///
/// If the straight line between the two points crosses the segment, this is
/// the crossing; otherwise it is the end of the segment the route bends
/// round. Like [`closest_on_segment`], the result lies within the segment's
/// bounding box.
pub fn crossing_point(from: Point, to: Point, a: Point, b: Point) -> Point {
    // Distances of the two points from the line through the segment, both
    // scaled by its length. Mirroring one point in the line leaves these
    // unchanged, so points on the same side need no special case.
    let near = i128::from(cross(a, b, from).abs());
    let far = i128::from(cross(a, b, to).abs());
    if near + far == 0 || a == b {
        return closest_on_segment(from, a, b);
    }
    let abx = i64::from(b.x) - i64::from(a.x);
    let aby = i64::from(b.y) - i64::from(a.y);
    let along = |p: Point| {
        i128::from(
            (i64::from(p.x) - i64::from(a.x)) * abx + (i64::from(p.y) - i64::from(a.y)) * aby,
        )
    };
    // The route touches the line at the average of the two points' positions
    // along it, each weighted by the other's distance from it. As a fraction
    // of the segment that is `numerator / denominator`.
    let numerator = along(from) * far + along(to) * near;
    let denominator = i128::from(abx * abx + aby * aby) * (near + far);
    if numerator <= 0 {
        return a;
    }
    if numerator >= denominator {
        return b;
    }
    // Both can reach 2^123. Drop low bits so the product with an edge delta
    // fits in 128; what is lost is far below a millimetre.
    let spare = (128 - denominator.leading_zeros()).saturating_sub(94);
    let (numerator, denominator) = (numerator >> spare, denominator >> spare);
    let offset = |delta: i64| {
        (2 * i128::from(delta) * numerator + denominator).div_euclid(2 * denominator) as Coord
    };
    Point::new(a.x + offset(abx), a.y + offset(aby))
}

/// The point of the segment `a`-`b` nearest to `p`, rounded to the nearest
/// millimetre on each axis.
///
/// The result always lies within the segment's bounding box, so it is in
/// range whenever `a` and `b` are.
pub fn closest_on_segment(p: Point, a: Point, b: Point) -> Point {
    let abx = i64::from(b.x) - i64::from(a.x);
    let aby = i64::from(b.y) - i64::from(a.y);
    let len_sq = abx * abx + aby * aby;
    let along = (i64::from(p.x) - i64::from(a.x)) * abx + (i64::from(p.y) - i64::from(a.y)) * aby;
    if along <= 0 {
        return a;
    }
    if along >= len_sq {
        return b;
    }
    // a + ab * along / len_sq, rounding halves up. 0 < along < len_sq, so
    // each offset stays between zero and the full edge delta.
    let offset = |delta: i64| {
        let scaled = i128::from(delta) * i128::from(along);
        (2 * scaled + i128::from(len_sq)).div_euclid(2 * i128::from(len_sq)) as Coord
    };
    Point::new(a.x + offset(abx), a.y + offset(aby))
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Point = Point::new(0, 0);
    const B: Point = Point::new(4 * MM_PER_M, 0);

    #[test]
    fn turn_directions() {
        let above = Point::new(0, 3 * MM_PER_M);
        let below = Point::new(0, -3 * MM_PER_M);
        let beyond = Point::new(8 * MM_PER_M, 0);
        assert_eq!(orientation(A, B, above), Orientation::CounterClockwise);
        assert_eq!(orientation(A, B, below), Orientation::Clockwise);
        assert_eq!(orientation(A, B, beyond), Orientation::Collinear);
    }

    #[test]
    fn cross_is_twice_the_signed_area() {
        let c = Point::new(0, 3 * MM_PER_M);
        assert_eq!(cross(A, B, c), 12_000_000);
        assert_eq!(cross(A, c, B), -12_000_000);
    }

    #[test]
    fn one_millimetre_off_the_line_is_not_collinear() {
        let far = Point::new(COORD_LIMIT, 0);
        let nudged = Point::new(COORD_LIMIT, 1);
        assert_eq!(orientation(A, far, far), Orientation::Collinear);
        assert_eq!(
            orientation(Point::new(-COORD_LIMIT, 0), A, nudged),
            Orientation::CounterClockwise
        );
    }

    #[test]
    fn distances_round_down() {
        assert_eq!(dist_sq(A, Point::new(3, 4)), 25);
        assert_eq!(dist(A, Point::new(3, 4)), 5);
        assert_eq!(dist(A, Point::new(1, 1)), 1);
        let l = COORD_LIMIT;
        assert_eq!(dist(Point::new(-l, 0), Point::new(l, 0)), 2 * i64::from(l));
    }

    #[test]
    fn crossing_point_is_where_the_shortest_route_touches() {
        let b = Point::new(10_000, 0);
        let hit = Point::new(6000, 0);
        // Straight through, from below to above.
        assert_eq!(
            crossing_point(Point::new(2000, -4000), Point::new(8000, 2000), A, b),
            hit
        );
        // Both above: out to the segment and back, as a mirror would show it.
        assert_eq!(
            crossing_point(Point::new(2000, 4000), Point::new(8000, 2000), A, b),
            hit
        );
        // The line misses the segment, so the route bends round an end.
        assert_eq!(
            crossing_point(Point::new(-5000, -1000), Point::new(-4000, 1000), A, b),
            A
        );
        assert_eq!(
            crossing_point(Point::new(20_000, -1), Point::new(30_000, 5), A, b),
            b
        );
        // Both on the line: the nearest point to the first.
        assert_eq!(
            crossing_point(Point::new(3000, 0), Point::new(20_000, 0), A, b),
            Point::new(3000, 0)
        );
    }

    #[test]
    fn crossing_point_is_exact_at_the_coordinate_limit() {
        let l = COORD_LIMIT;
        let (a, b) = (Point::new(-l, 0), Point::new(l, 0));
        assert_eq!(
            crossing_point(Point::new(-l, -l), Point::new(l, l), a, b),
            A
        );
        assert_eq!(crossing_point(Point::new(-l, l), Point::new(l, l), a, b), A);
        assert_eq!(
            crossing_point(Point::new(-l, -l), Point::new(-l, l), a, b),
            a
        );
        assert_eq!(
            crossing_point(Point::new(l, -1), Point::new(l - 2, 1), a, b),
            Point::new(l - 1, 0)
        );
    }

    #[test]
    fn closest_point_clamps_to_the_segment() {
        assert_eq!(
            closest_on_segment(Point::new(1000, 700), A, B),
            Point::new(1000, 0)
        );
        assert_eq!(closest_on_segment(Point::new(-50, 700), A, B), A);
        assert_eq!(closest_on_segment(Point::new(9000, -1), A, B), B);
        assert_eq!(closest_on_segment(Point::new(5, 5), A, A), A);
        // The exact foot is (1.5, 1.5); halves round up.
        let diagonal = Point::new(3, 3);
        assert_eq!(
            closest_on_segment(Point::new(3, 0), A, diagonal),
            Point::new(2, 2)
        );
    }

    #[test]
    fn exact_at_the_coordinate_limit() {
        let l = COORD_LIMIT;
        let (a, b, c) = (Point::new(-l, -l), Point::new(l, -l), Point::new(l, l));
        assert!(a.in_range() && b.in_range() && c.in_range());
        assert_eq!(cross(a, b, c), 1_i64 << 60);
        assert!(!Point::new(l + 1, 0).in_range());
        assert!(!Point::new(0, i32::MIN).in_range());
    }
}
