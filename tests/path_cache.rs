use hierarchical_pathfinding::prelude::*;

#[test]
fn new() {
    let grid = [
        [0, 2, 0, 0, 0],
        [0, 2, 2, 2, 2],
        [0, 1, 0, 0, 0],
        [0, 1, 0, 2, 0],
        [0, 0, 0, 2, 0],
    ];
    let (width, height) = (grid[0].len(), grid.len());
    fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Fn((usize, usize)) -> isize {
        move |(x, y)| [1, 10, -1][grid[y][x]]
    }
    let pathfinding = PathCache::new(
        (width, height),
        cost_fn(&grid),
        ManhattanNeighborhood::new(width, height),
        PathCacheConfig::with_chunk_size(3),
    );
    let start = (0, 0);
    let goal = (4, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    let path = path.unwrap();
    let points: Vec<(usize, usize)> = path.collect();
    #[rustfmt::skip]
        assert_eq!(
            points,
            vec![(0, 1), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4), (2, 3), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4)],
        );
}

#[test]
fn update_path() {
    let mut grid = [
        [0, 2, 0, 0, 0],
        [0, 2, 2, 2, 2],
        [0, 1, 0, 0, 0],
        [0, 1, 0, 2, 0],
        [0, 0, 0, 2, 0],
    ];
    let (width, height) = (grid[0].len(), grid.len());
    fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Fn((usize, usize)) -> isize {
        move |(x, y)| [1, 10, -1][grid[y][x]]
    }

    let mut pathfinding = PathCache::new(
        (width, height),
        cost_fn(&grid),
        MooreNeighborhood::new(width, height),
        PathCacheConfig::with_chunk_size(3),
    );

    let start = (0, 0);
    let goal = (4, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    let path = path.unwrap();
    let points: Vec<(usize, usize)> = path.collect();
    #[rustfmt::skip]
        assert_eq!(
            points,
            vec![(0, 1), (0, 2), (0, 3), (1, 4), (2, 3), (3, 2), (4, 3), (4, 4)],
        );

    grid[2][1] = 0;
    let changed_tiles = [(1, 2)];

    pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

    let start = (0, 0);
    let goal = (4, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    let path = path.unwrap();
    let points: Vec<(usize, usize)> = path.collect();
    #[rustfmt::skip]
        assert_eq!(
            points,
            vec![(0, 1), (1, 2), (2, 2), (3, 2), (4, 3), (4, 4)],
        );

    // Add walls along chunk borders
    let start = (0, 0);
    let goal = (3, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_some());

    // Vertical wall
    let changed_tiles: Vec<_> = (0..grid.len()).map(|y| (2, y)).collect();
    grid.iter_mut().for_each(|row| row[2] = 2);
    pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_none());

    let goal = (2, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_some());

    // Horizontal wall
    let mut changed_tiles = Vec::new();
    {
        let y = 2;
        for x in 0..grid[y].len() {
            grid[y][x] = 2;
            changed_tiles.push((x, y));
        }
    }
    pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_none());
}

#[test]
fn skip_first_and_last() {
    let grid = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0]];
    // explanation:
    // grid:
    // . . .|#   // . = empty, # = wall
    // . S 0|1   // S = start
    // . G 2|#   // G = goal
    // -----+-   // chunk border
    // # # 4|5
    //
    //      ^ chunk border
    // numbers 0-3: nodes
    //
    // calculated path will be S -> 0 -> 2 -> G
    // but: old path optimizations would add the shortcut S->2 and 0->G, which would create
    //      an invalid path
    let (width, height) = (grid[0].len(), grid.len());
    fn cost_fn(grid: &[[usize; 4]]) -> impl '_ + Fn((usize, usize)) -> isize {
        move |(x, y)| [1, -1][grid[y][x]]
    }
    let pathfinding = PathCache::new(
        (width, height),
        cost_fn(&grid),
        ManhattanNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size: 3,
            a_star_fallback: false,
            ..Default::default()
        },
    );
    let start = (1, 1);
    let goal = (1, 2);
    let path = pathfinding.find_closest_goal(start, &[goal, (2, 3)], cost_fn(&grid));
    assert!(path.is_some());
    assert_eq!(path.unwrap().1.resolve(cost_fn(&grid)), vec![goal]);
}

#[test]
fn from_github_issue_7_example_1() {
    use hierarchical_pathfinding::prelude::*;

    type Grid = [[usize; 7]; 7];

    const COST_MAP: [isize; 3] = [1, 10, -1]; // now const for ownership reasons

    // only borrows the Grid when called
    fn cost_fn(grid: &Grid) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
        move |(x, y)| COST_MAP[grid[y][x]]
    }

    fn neighbors(x: usize, y: usize) -> [(usize, usize); 4] {
        [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    }

    fn main() {
        // 0 = empty, 1 = swamp, 2 = wall
        let grid: Grid = [
            [2, 2, 0, 0, 0, 0, 0],
            [2, 0, 0, 0, 0, 0, 0],
            [1, 0, 0, 2, 2, 0, 0],
            [0, 1, 0, 1, 2, 0, 0],
            [0, 0, 0, 2, 2, 0, 0],
            [0, 0, 0, 0, 0, 0, 2],
            [0, 0, 0, 0, 0, 2, 2],
        ];
        let (width, height) = (grid[0].len(), grid.len());

        let pathfinding = PathCache::new(
            (width, height),
            cost_fn(&grid),
            ManhattanNeighborhood::new(width, height),
            PathCacheConfig::with_chunk_size(3),
        );

        let path = pathfinding.find_closest_goal((1, 1), &neighbors(1, 3), cost_fn(&grid));
        assert!(path.is_some());
        let path = pathfinding.find_closest_goal((1, 3), &neighbors(1, 1), cost_fn(&grid));
        assert!(path.is_some());
    }

    main();
}

#[test]
fn from_github_issue_7_example_2() {
    use hierarchical_pathfinding::prelude::*;

    type Grid = [[usize; 7]; 7];

    const COST_MAP: [isize; 3] = [1, 10, -1]; // now const for ownership reasons

    // only borrows the Grid when called
    fn cost_fn(grid: &Grid) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
        move |(x, y)| COST_MAP[grid[y][x]]
    }

    fn neighbors(pos: (usize, usize)) -> [(isize, isize); 4] {
        [
            (pos.0 as isize + 1, pos.1 as isize),
            (pos.0 as isize - 1, pos.1 as isize),
            (pos.0 as isize, pos.1 as isize + 1),
            (pos.0 as isize, pos.1 as isize - 1),
        ]
    }

    fn main() {
        // 0 = empty, 1 = swamp, 2 = wall
        let grid: Grid = [
            [2, 2, 0, 0, 0, 0, 0],
            [2, 0, 0, 0, 0, 0, 0],
            [1, 0, 0, 2, 2, 0, 0],
            [0, 1, 0, 1, 2, 0, 0],
            [0, 0, 0, 2, 2, 0, 0],
            [0, 0, 0, 0, 0, 0, 2],
            [0, 0, 0, 0, 0, 2, 2],
        ];
        let (width, height) = (grid.len(), grid[0].len());

        let pathfinding = PathCache::new(
            (width, height),
            cost_fn(&grid),
            ManhattanNeighborhood::new(width, height),
            PathCacheConfig::with_chunk_size(3),
        );

        let path = pathfind((1, 1), (3, 1), &grid, &pathfinding);
        assert!(path.is_some());
        let len = path.unwrap().len();
        assert!(len <= 3, "len={}", len);
        let path = pathfind((3, 1), (5, 1), &grid, &pathfinding);
        assert!(path.is_some());
        let len = path.unwrap().len();
        assert!(len <= 3, "len={}", len);
        let path = pathfind((5, 1), (3, 1), &grid, &pathfinding);
        assert!(path.is_some());
        let len = path.unwrap().len();
        assert!(len <= 3, "len={}", len);
        let path = pathfind((3, 1), (1, 1), &grid, &pathfinding);
        assert!(path.is_some());
        let len = path.unwrap().len();
        assert!(len <= 3, "len={}", len);
    }

    fn pathfind(
        pos: (usize, usize),
        goal: (usize, usize),
        grid: &Grid,
        pathfinding: &PathCache<ManhattanNeighborhood>,
    ) -> Option<Vec<(usize, usize)>> {
        let valid_neighbors = neighbors(goal)
            .iter()
            .cloned()
            .filter(|n| n.0 >= 0 && n.1 >= 0 && cost_fn(&grid)((n.0 as usize, n.1 as usize)) != -1)
            .map(|n| (n.0 as usize, n.1 as usize))
            .collect::<Vec<_>>();
        println!("valid_neighbors: {:?}", valid_neighbors);
        let (_goal, path) =
            pathfinding.find_closest_goal(pos, &valid_neighbors.as_slice(), cost_fn(&grid))?;
        Some(path.resolve(cost_fn(&grid)))
    }

    main();
}

type Grid20 = [[usize; 20]; 20];

const COST_MAP20: [isize; 3] = [1, 10, -1];

fn cost_fn20(grid: &Grid20) -> impl '_ + Fn((usize, usize)) -> isize {
    move |(x, y)| COST_MAP20[grid[y][x]]
}

// 20x20 grid, all empty, one vertical wall in column 9 with a single gap, chunk_size 4.
fn multichunk_wall_fixture() -> (Grid20, PathCache<ManhattanNeighborhood>) {
    const WALL_X: usize = 9;
    const GAP_Y: usize = 10;
    let mut grid: Grid20 = [[0; 20]; 20];
    for row in grid.iter_mut() {
        row[WALL_X] = 2;
    }
    grid[GAP_Y][WALL_X] = 0;
    let (width, height) = (grid[0].len(), grid.len());
    let pathfinding = PathCache::new(
        (width, height),
        cost_fn20(&grid),
        ManhattanNeighborhood::new(width, height),
        PathCacheConfig::with_chunk_size(4),
    );
    (grid, pathfinding)
}

#[test]
fn portals_multichunk_endpoints() {
    let (grid, pathfinding) = multichunk_wall_fixture();
    let start = (1, 1);
    let goal = (18, 18);
    let (path, portals) = pathfinding
        .find_path_with_portals(start, goal, cost_fn20(&grid))
        .unwrap();
    assert_eq!(portals[0], start);
    assert_eq!(*portals.last().unwrap(), goal);
    assert!(
        portals.len() >= 3,
        "expected at least 3 portals, got {:?}",
        portals
    );
    let _ = path;
}

#[test]
fn portals_on_chunk_borders() {
    const CS: usize = 4;
    let (grid, pathfinding) = multichunk_wall_fixture();
    let start = (1, 1);
    let goal = (18, 18);
    let (_path, portals) = pathfinding
        .find_path_with_portals(start, goal, cost_fn20(&grid))
        .unwrap();
    assert!(portals.len() >= 3, "portals={:?}", portals);
    for portal in &portals[1..portals.len() - 1] {
        let (x, y) = *portal;
        assert!(
            x % CS == 0 || (x + 1) % CS == 0 || y % CS == 0 || (y + 1) % CS == 0,
            "portal {:?} is not on a chunk border",
            portal
        );
    }
}

#[test]
fn portals_same_chunk_no_portals() {
    let grid: Grid20 = [[0; 20]; 20];
    let (width, height) = (grid[0].len(), grid.len());
    let pathfinding = PathCache::new(
        (width, height),
        cost_fn20(&grid),
        ManhattanNeighborhood::new(width, height),
        PathCacheConfig::with_chunk_size(4),
    );
    let start = (1, 1);
    let goal = (2, 2);
    let (_path, portals) = pathfinding
        .find_path_with_portals(start, goal, cost_fn20(&grid))
        .unwrap();
    assert_eq!(portals, vec![start, goal]);
}

#[test]
fn portals_cost_matches_find_path() {
    let (grid, pathfinding) = multichunk_wall_fixture();
    let pairs = [
        ((1, 1), (18, 18)), // multichunk, across the wall
        ((1, 1), (2, 2)),   // same chunk, short route
        ((18, 1), (1, 18)), // multichunk, other direction
    ];
    for (start, goal) in pairs.iter() {
        let (path, _portals) = pathfinding
            .find_path_with_portals(*start, *goal, cost_fn20(&grid))
            .unwrap();
        let expected = pathfinding
            .find_path(*start, *goal, cost_fn20(&grid))
            .unwrap()
            .cost();
        assert_eq!(
            path.cost(),
            expected,
            "cost mismatch for {:?} -> {:?}",
            start,
            goal
        );
    }
}
