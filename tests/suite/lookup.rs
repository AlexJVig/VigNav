//! Finding where a point is on the mesh.

use super::support::*;

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
fn contains_includes_the_boundary() {
    let mesh = l_shape();
    assert!(mesh.contains(A, p(500, 500)));
    assert!(mesh.contains(A, p(M, 500)) && mesh.contains(B, p(M, 500)));
    assert!(mesh.contains(A, p(M, M)) && mesh.contains(C, p(M, M)));
    assert!(!mesh.contains(A, p(M + 1, 500)));
    assert!(!mesh.contains(C, p(500, 1500)));
}

#[test]
fn snapping_stops_at_its_reach() {
    let mesh = l_shape();
    let left_of_a = p(-300, 500);
    let snapped = Some(Location {
        poly: A,
        point: p(0, 500),
    });
    assert_eq!(mesh.nearest(left_of_a, 300), snapped);
    assert_eq!(mesh.nearest(left_of_a, 299), None);
    assert_eq!(mesh.nearest(left_of_a, -1), None);
    // From far outside the mesh, with a reach to match.
    assert_eq!(mesh.nearest(p(-900_000, 500), 1_000_000), snapped);
}

#[test]
fn a_point_snapped_to_a_slanted_edge_is_rounded_and_still_routes() {
    let mesh = NavMesh::from_polygons(&[p(0, 0), p(M, 0), p(0, M)], &[[0, 1, 2]]).unwrap();
    // The exact foot of (1001, 1000) on the slanted edge is (500.5, 499.5).
    let snapped = mesh.nearest(p(1001, 1000), M).unwrap();
    assert_eq!(
        snapped,
        Location {
            poly: PolyId(0),
            point: p(501, 500)
        }
    );
    // That is a fraction of a millimetre outside the triangle, and fine.
    assert!(!mesh.contains(snapped.poly, snapped.point));
    let path = mesh.find_path(p(1001, 1000), p(100, 100), M).unwrap();
    assert_eq!(path.points, [p(501, 500), p(100, 100)]);
}
