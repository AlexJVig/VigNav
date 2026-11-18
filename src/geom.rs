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
    fn exact_at_the_coordinate_limit() {
        let l = COORD_LIMIT;
        let (a, b, c) = (Point::new(-l, -l), Point::new(l, -l), Point::new(l, l));
        assert!(a.in_range() && b.in_range() && c.in_range());
        assert_eq!(cross(a, b, c), 1_i64 << 60);
        assert!(!Point::new(l + 1, 0).in_range());
        assert!(!Point::new(0, i32::MIN).in_range());
    }
}
