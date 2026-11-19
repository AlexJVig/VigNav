//! Rough timings of the calls a game makes every tick.
//!
//! ```text
//! cargo run --release --example timing
//! ```
//!
//! The mesh is a grid of 4 m squares with a fifth of them missing, about the
//! size of a large shipped map. The numbers are for comparing one change
//! with another on the same machine, not for quoting.

use std::time::Instant;

use vignav::{NavMesh, ObstacleKind, Point};

const COLS: usize = 64;
const ROWS: usize = 52;
const CELL: i32 = 4000;

struct Rng(u64);

impl Rng {
    fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 33) % n as u64) as usize
    }
}

fn main() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let vertices: Vec<Point> = (0..(COLS + 1) * (ROWS + 1))
        .map(|i| {
            Point::new(
                (i % (COLS + 1)) as i32 * CELL,
                (i / (COLS + 1)) as i32 * CELL,
            )
        })
        .collect();
    let corner = |x: usize, y: usize| (y * (COLS + 1) + x) as u32;
    let mut polygons = Vec::new();
    for y in 0..ROWS {
        for x in 0..COLS {
            if rng.below(5) != 0 {
                polygons.push([
                    corner(x, y),
                    corner(x + 1, y),
                    corner(x + 1, y + 1),
                    corner(x, y + 1),
                ]);
            }
        }
    }

    let began = Instant::now();
    let mut mesh = NavMesh::from_polygons(&vertices, &polygons).unwrap();
    println!(
        "{} polygons built in {:.2} ms",
        mesh.polygon_count(),
        began.elapsed().as_secs_f64() * 1e3
    );

    let anywhere = |rng: &mut Rng| {
        Point::new(
            rng.below(COLS * CELL as usize) as i32,
            rng.below(ROWS * CELL as usize) as i32,
        )
    };
    let nearby = |rng: &mut Rng, from: Point| {
        let step = |rng: &mut Rng| rng.below(60_000) as i32 - 30_000;
        Point::new(from.x + step(rng), from.y + step(rng))
    };
    let far: Vec<(Point, Point)> = (0..4000)
        .map(|_| (anywhere(&mut rng), anywhere(&mut rng)))
        .collect();
    let near: Vec<(Point, Point)> = (0..4000)
        .map(|_| {
            let from = anywhere(&mut rng);
            (from, nearby(&mut rng, from))
        })
        .collect();

    let time = |label: &str, mesh: &NavMesh, queries: &[(Point, Point)]| {
        let mut micros: Vec<f64> = queries
            .iter()
            .map(|&(start, goal)| {
                let best = (0..3).map(|_| {
                    let began = Instant::now();
                    std::hint::black_box(mesh.find_path(start, goal, 3000).ok());
                    began.elapsed().as_secs_f64() * 1e6
                });
                best.fold(f64::MAX, f64::min)
            })
            .collect();
        micros.sort_by(f64::total_cmp);
        let at = |q: usize| micros[micros.len() * q / 100];
        println!(
            "{label}: median {:.2} us, 95th percentile {:.2} us",
            at(50),
            at(95)
        );
    };
    time("far routes ", &mesh, &far);
    time("near routes", &mesh, &near);

    // Six hundred small obstacles at once, as when a town is loaded.
    let shapes: Vec<[Point; 4]> = (0..600)
        .map(|_| {
            let at = anywhere(&mut rng);
            let (w, h) = (500 + rng.below(3000) as i32, 500 + rng.below(3000) as i32);
            [
                at,
                Point::new(at.x + w, at.y),
                Point::new(at.x + w, at.y + h),
                Point::new(at.x, at.y + h),
            ]
        })
        .collect();
    let began = Instant::now();
    let mut edit = mesh.edit_obstacles();
    for shape in &shapes {
        edit.add(shape, ObstacleKind::Blocked).unwrap();
    }
    drop(edit);
    println!(
        "600 obstacles laid at once in {:.2} ms",
        began.elapsed().as_secs_f64() * 1e3
    );

    // One more, on and off again, as when a unit stops and starts.
    let mut micros = Vec::new();
    for shape in shapes.iter().take(200) {
        let moved = shape.map(|q| Point::new(q.x + 1700, q.y + 900));
        let began = Instant::now();
        let id = mesh.add_obstacle(&moved, ObstacleKind::Avoid).unwrap();
        mesh.remove_obstacle(id);
        micros.push(began.elapsed().as_secs_f64() * 1e6 / 2.0);
    }
    micros.sort_by(f64::total_cmp);
    println!(
        "one obstacle on or off: median {:.1} us, worst {:.1} us",
        micros[100], micros[199]
    );

    time("far routes  among obstacles", &mesh, &far);
    time("near routes among obstacles", &mesh, &near);
}
