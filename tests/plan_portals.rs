use hierarchical_pathfinding::prelude::*;

type Point = (usize, usize);

fn grid_cache(grid: [[isize; 8]; 8], chunk_size: usize) -> PathCache<ManhattanNeighborhood> {
    PathCache::new(
        (8, 8),
        move |(x, y): Point| grid[y][x],
        ManhattanNeighborhood::new(8, 8),
        PathCacheConfig::with_chunk_size(chunk_size),
    )
}

// Map for tests (b) and (c): 8x8, chunk size 4.
//
// A solid column at x=1 (rows 0..=3) plus a solid tile at (0, 3) seal the
// tiles {(0,0), (0,1), (0,2)} into a pocket inside the start chunk. The only
// portal of that chunk nearest to (0, 0) by position is (3, 0) — behind the
// wall as far as any local walk from the pocket is concerned.
fn pocket_grid() -> [[isize; 8]; 8] {
    let mut grid = [[1isize; 8]; 8];
    for y in 0..4 {
        grid[y][1] = -1;
    }
    grid[3][0] = -1;
    grid
}

// (b) The first portal is picked by nearest position WITHOUT checking that
// the pocket around `start` can actually reach it: (3, 0) is returned even
// though the straight (and any) local walk from (0, 0) is blocked by the
// wall at x=1 / (0, 3). This documents the behavior, it is not an error.
#[test]
fn portal_behind_wall_is_still_chosen() {
    let cache = grid_cache(pocket_grid(), 4);
    let portals = cache
        .plan_portals((0, 0), (7, 7))
        .expect("abstract path must be found");
    assert_eq!(portals[0], (0, 0), "list must start at start");
    assert_eq!(
        portals[1],
        (3, 0),
        "first portal must be the closest-by-position Node, behind the wall"
    );
    assert_eq!(*portals.last().unwrap(), (7, 7), "list must end at goal");
}

// (c) Same-chunk queries never touch the graph: exactly [start, goal].
#[test]
fn same_chunk_returns_endpoints_only() {
    let cache = grid_cache(pocket_grid(), 4);
    assert_eq!(
        cache.plan_portals((0, 0), (0, 1)),
        Some(vec![(0, 0), (0, 1)]),
        "same-chunk query must return exactly [start, goal]"
    );
    assert_eq!(
        cache.plan_portals((5, 6), (6, 5)).unwrap().len(),
        2,
        "same-chunk result must have length 2"
    );
}

// (d) Cave: the chunk x=4..=7, y=4..=7 is filled entirely with solid tiles,
// so no border gap around it exists and the chunk has zero Nodes. A start
// inside it must yield None.
#[test]
fn cave_chunk_without_nodes_gives_none() {
    let mut grid = [[1isize; 8]; 8];
    for y in 4..8 {
        for x in 4..8 {
            grid[y][x] = -1;
        }
    }
    let cache = grid_cache(grid, 4);
    assert_eq!(
        cache.plan_portals((5, 5), (0, 0)),
        None,
        "start in a Node-less (all-wall) chunk must be None"
    );
}

// Out-of-bounds points must give None, never panic.
#[test]
fn out_of_bounds_gives_none() {
    let cache = grid_cache(pocket_grid(), 4);
    assert_eq!(cache.plan_portals((8, 0), (7, 7)), None);
    assert_eq!(cache.plan_portals((0, 0), (0, 8)), None);
    assert_eq!(cache.plan_portals((9, 9), (0, 0)), None);
}

// (e) 20x20 map, chunk size 6, solid wall column at x=8 with its only gap at
// y=2. Query from the top-left chunk to the bottom-right corner. Every
// interior portal must sit on a chunk border, and the first interior portal
// must be the Node closest to (1, 2) by Manhattan heuristic: the chunk of
// (1, 2) has Nodes (0, 5), (5, 0) and (5, 5); heuristics 4, 6 and 8 — the
// unique closest is (0, 5).
#[test]
fn portals_on_chunk_borders() {
    let mut grid = [[1isize; 20]; 20];
    for y in 0..20 {
        if y != 2 {
            grid[y][8] = -1;
        }
    }
    let cache = PathCache::new(
        (20, 20),
        move |(x, y): Point| grid[y][x],
        ManhattanNeighborhood::new(20, 20),
        PathCacheConfig::with_chunk_size(6),
    );

    let start: Point = (1, 2);
    let goal: Point = (19, 19);
    let portals = cache
        .plan_portals(start, goal)
        .expect("path around the wall must be found");

    assert_eq!(portals[0], start);
    assert_eq!(*portals.last().unwrap(), goal);
    assert_eq!(
        portals[1],
        (0, 5),
        "first interior portal must be the closest Node of the start chunk"
    );

    for p in &portals[1..portals.len() - 1] {
        let on_border = p.0 % 6 == 0 || p.0 % 6 == 5 || p.1 % 6 == 0 || p.1 % 6 == 5;
        assert!(
            on_border,
            "interior portal {:?} must lie on a chunk border",
            p
        );
    }
}

// (a) The portal sequence of a deserialized cache must equal the original's.
// Only plan_portals is queried after the round-trip: no cost function needed.
#[cfg(feature = "serialize")]
#[test]
fn roundtrip_plan_portals_equivalence() {
    // same map as tests/serialize_roundtrip.rs: wall column x=3 with gap at
    // y=1, wall row y=4 with gap at x=6; chunk size 4.
    let mut grid = [[1isize; 8]; 8];
    for y in 0..8 {
        for x in 0..8 {
            if x == 3 && y != 1 {
                grid[y][x] = -1;
            }
            if y == 4 && x != 6 {
                grid[y][x] = -1;
            }
        }
    }
    let cache = PathCache::new(
        (8, 8),
        move |(x, y): Point| grid[y][x],
        ManhattanNeighborhood::new(8, 8),
        PathCacheConfig {
            chunk_size: 4,
            a_star_fallback: false,
            ..Default::default()
        },
    );

    let bytes = cache.to_bytes();
    let restored: PathCache<ManhattanNeighborhood> =
        PathCache::from_bytes(&bytes).expect("from_bytes must succeed");

    for (start, goal) in [((1, 1), (6, 6)), ((0, 0), (7, 7)), ((6, 1), (0, 0))] {
        let before = cache
            .plan_portals(start, goal)
            .expect("cross-chunk plan must exist");
        let after = restored
            .plan_portals(start, goal)
            .expect("deserialized cache must plan the same paths");
        assert_eq!(
            before, after,
            "plan_portals({:?}, {:?}) must survive to_bytes/from_bytes",
            start, goal
        );
        assert_eq!(before[0], start);
        assert_eq!(*before.last().unwrap(), goal);
    }
}
