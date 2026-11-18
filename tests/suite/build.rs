//! Building a mesh: both constructors and every way they can refuse.

use super::support::*;

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
fn an_edge_used_three_times_is_refused() {
    // A and B share the edge between vertices 1 and 4 properly; a triangle
    // then runs along it again.
    let polygons: [&[u32]; 3] = [&[0, 1, 4, 3], &[1, 2, 5, 4], &[1, 5, 4]];
    assert!(matches!(
        NavMesh::from_polygons(&L_VERTICES, &polygons).unwrap_err(),
        MeshError::EdgeConflict { .. }
    ));
}

#[test]
fn given_links_are_checked_for_shape() {
    let none: [Option<u32>; 4] = [None; 4];
    // A polygon cannot be its own neighbour.
    let own = [[None, Some(0), None, None], none, none];
    assert_eq!(
        NavMesh::from_linked_polygons(&L_VERTICES, &L_POLYGONS, &own).unwrap_err(),
        MeshError::BadLink {
            polygon: 0,
            edge: 1
        }
    );
    // One list is an edge short.
    let short: [Vec<Option<u32>>; 3] = [vec![None; 4], vec![None; 3], vec![None; 4]];
    assert_eq!(
        NavMesh::from_linked_polygons(&L_VERTICES, &L_POLYGONS, &short).unwrap_err(),
        MeshError::LinkCountMismatch { polygon: 1 }
    );
}

#[test]
fn errors_say_where_the_problem_is() {
    assert_eq!(
        MeshError::NotConvex {
            polygon: 3,
            corner: 1
        }
        .to_string(),
        "polygon 3 is not convex at corner 1"
    );
    assert_eq!(
        MeshError::BadLink {
            polygon: 7,
            edge: 2
        }
        .to_string(),
        "polygon 7 edge 2 has a link that is not answered"
    );
    assert_eq!(
        PathError::Unreachable.to_string(),
        "the goal cannot be reached from the start"
    );
    // Both work wherever a boxed error is expected.
    let boxed: [Box<dyn std::error::Error>; 2] = [
        Box::new(MeshError::TooLarge),
        Box::new(PathError::StartOffMesh),
    ];
    assert!(boxed.iter().all(|error| !error.to_string().is_empty()));
}

// `MeshError::TooLarge` has no test of its own: it takes four billion
// vertices, polygons or edges to provoke.
