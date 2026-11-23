use hierarchical_pathfinding::prelude::*;

type Point = (usize, usize);

// 8x8 grid, row-major, cost 1 open / -1 solid.
//
// A wall column at x=3 with its only gap at y=1, and a wall row at y=4 with
// its only gap at x=6. Any path between the query pairs below must cross both
// chunk borders (chunk size 4 => borders at x=3/4 and y=3/4), so the portal
// sequences are non-trivial.
const SIZE: usize = 8;

fn cost_table() -> [[isize; SIZE]; SIZE] {
    let mut grid = [[1isize; SIZE]; SIZE];
    for y in 0..SIZE {
        for x in 0..SIZE {
            if x == 3 && y != 1 {
                grid[y][x] = -1;
            }
            if y == 4 && x != 6 {
                grid[y][x] = -1;
            }
        }
    }
    grid
}

fn cost_fn(grid: &'static [[isize; SIZE]; SIZE]) -> impl Sync + Fn(Point) -> isize {
    move |(x, y): Point| grid[y][x]
}

fn fallback_off() -> PathCacheConfig {
    PathCacheConfig {
        chunk_size: 4,
        a_star_fallback: false,
        ..Default::default()
    }
}

fn flat_costs() -> Vec<isize> {
    cost_table().iter().flatten().copied().collect()
}

fn query(
    cache: &PathCache<ManhattanNeighborhood>,
    get_cost: impl FnMut(Point) -> isize,
    start: Point,
    goal: Point,
) -> (usize, Vec<Point>) {
    let (path, portals) = cache
        .find_path_with_portals(start, goal, get_cost)
        .expect("path must be found");
    assert!(
        portals.len() > 2,
        "expected a non-trivial portal sequence crossing chunk borders, got {:?}",
        portals
    );
    assert_eq!(portals[0], start);
    assert_eq!(*portals.last().unwrap(), goal);
    (path.cost(), portals)
}

#[test]
fn roundtrip_manhattan() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        fallback_off(),
    );

    let start: Point = (1, 1);
    let goal: Point = (6, 6);
    let before = query(&cache, cost_fn(grid), start, goal);
    assert!(before.0 > 0, "cost should be positive, got {}", before.0);

    let bytes = bincode::serialize(&cache).expect("serialization must succeed");
    let back: PathCache<ManhattanNeighborhood> =
        bincode::deserialize(&bytes).expect("deserialization must succeed");

    let after = query(&back, cost_fn(grid), start, goal);
    assert_eq!(
        after.1, before.1,
        "portal sequence must survive the round-trip"
    );
    assert_eq!(after.0, before.0, "path cost must survive the round-trip");

    // second distinct pair: catches one-shot luck
    let start2: Point = (0, 0);
    let goal2: Point = (7, 7);
    let before2 = query(&cache, cost_fn(grid), start2, goal2);
    let after2 = query(&back, cost_fn(grid), start2, goal2);
    assert_eq!(
        after2.1, before2.1,
        "second portal sequence must survive the round-trip"
    );
    assert_eq!(
        after2.0, before2.0,
        "second path cost must survive the round-trip"
    );
}

#[test]
fn roundtrip_no_corner() {
    let costs = flat_costs();
    let hood = NoCornerNeighborhood::new(SIZE, SIZE, costs.clone());
    let cache = PathCache::new((SIZE, SIZE), hood.cost_fn(), hood.clone(), fallback_off());
    let get_cost = move |(x, y): Point| costs[y * SIZE + x];

    let start: Point = (1, 1);
    let goal: Point = (6, 6);
    let (path, portals) = cache
        .find_path_with_portals(start, goal, get_cost.clone())
        .expect("path must be found");
    assert!(
        portals.len() > 2,
        "expected a non-trivial portal sequence, got {:?}",
        portals
    );
    let before = (path.cost(), portals);

    let bytes = bincode::serialize(&cache).expect("serialization must succeed");
    let back: PathCache<NoCornerNeighborhood> =
        bincode::deserialize(&bytes).expect("deserialization must succeed");

    let (path, portals) = back
        .find_path_with_portals(start, goal, get_cost)
        .expect("path must be found after round-trip");
    assert_eq!(
        portals, before.1,
        "NoCorner portal sequence must survive the round-trip"
    );
    assert_eq!(
        path.cost(),
        before.0,
        "NoCorner path cost must survive the round-trip"
    );
}

#[test]
fn bincode_garbage_is_err() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        fallback_off(),
    );
    let bytes = bincode::serialize(&cache).expect("serialization must succeed");
    assert!(bytes.len() > 20, "payload should be non-trivial");

    let res = bincode::deserialize::<PathCache<ManhattanNeighborhood>>(&bytes[..bytes.len() / 2]);
    assert!(
        res.is_err(),
        "truncated payload must be an error, not a panic"
    );

    let res = bincode::deserialize::<PathCache<ManhattanNeighborhood>>(&bytes[..10]);
    assert!(res.is_err(), "heavily truncated payload must be an error");
}

#[test]
fn roundtrip_no_cached_paths() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        PathCacheConfig {
            cache_paths: false,
            ..fallback_off()
        },
    );

    let start: Point = (1, 1);
    let goal: Point = (6, 6);
    let before = query(&cache, cost_fn(grid), start, goal);

    let bytes = bincode::serialize(&cache).expect("serialization must succeed");
    let back: PathCache<ManhattanNeighborhood> =
        bincode::deserialize(&bytes).expect("deserialization must succeed");

    let after = query(&back, cost_fn(grid), start, goal);
    assert_eq!(
        after.1, before.1,
        "portal sequence must survive the round-trip with cache_paths=false"
    );
    assert_eq!(
        after.0, before.0,
        "path cost must survive the round-trip with cache_paths=false"
    );
}

#[test]
fn to_bytes_from_bytes_planning_equivalence() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        fallback_off(),
    );

    let start: Point = (1, 1);
    let goal: Point = (6, 6);
    let before1 = query(&cache, cost_fn(grid), start, goal);
    let start2: Point = (0, 0);
    let goal2: Point = (7, 7);
    let before2 = query(&cache, cost_fn(grid), start2, goal2);

    let bytes = cache.to_bytes();
    assert!(bytes.len() > 20, "blob should be non-trivial");
    assert_eq!(&bytes[..4], b"HPAG", "blob must start with the magic");
    assert_eq!(bytes[4], 1, "blob must carry format version 1");

    let back: PathCache<ManhattanNeighborhood> =
        PathCache::from_bytes(&bytes).expect("from_bytes must succeed");

    let after1 = query(&back, cost_fn(grid), start, goal);
    let after2 = query(&back, cost_fn(grid), start2, goal2);
    assert_eq!(
        after1, before1,
        "first path must survive to_bytes/from_bytes"
    );
    assert_eq!(
        after2, before2,
        "second path must survive to_bytes/from_bytes"
    );
}

#[test]
fn to_bytes_matches_bincode_payload() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        fallback_off(),
    );

    let bytes = cache.to_bytes();
    let raw = bincode::serialize(&cache).unwrap();

    assert_eq!(
        bytes.len(),
        raw.len() + 5,
        "header is exactly the 5 documented bytes"
    );
    assert_eq!(&bytes[5..], &raw[..], "payload must be plain bincode");

    // A raw bincode payload without the header must be rejected: 5+ bytes of
    // bincode data never start with `HPAG`.
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(&raw).err(),
        Some(HpaSerError::BadMagic),
        "headerless bincode must be rejected as BadMagic"
    );
}

#[test]
fn from_bytes_rejects_bad_headers() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        fallback_off(),
    );
    let bytes = cache.to_bytes();

    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(&[]).err(),
        Some(HpaSerError::TooShort),
        "empty input is too short for a header"
    );
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(&bytes[..4]).err(),
        Some(HpaSerError::TooShort),
        "magic without a version byte is too short"
    );
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(b"nope!").err(),
        Some(HpaSerError::BadMagic),
        "wrong magic is BadMagic"
    );
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(b"HPAG").err(),
        Some(HpaSerError::TooShort),
        "magic alone is too short"
    );
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(b"HXAG\x01").err(),
        Some(HpaSerError::BadMagic),
        "corrupted magic byte is BadMagic"
    );

    let mut bumped = bytes.clone();
    bumped[4] = 99;
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(&bumped).err(),
        Some(HpaSerError::UnsupportedVersion(99)),
        "unknown version carries the byte found"
    );
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(b"HPAG\x00").err(),
        Some(HpaSerError::UnsupportedVersion(0)),
        "version 0 is reported back"
    );
}

#[test]
fn from_bytes_rejects_corrupt_payload() {
    let grid = Box::leak(Box::new(cost_table()));
    let cache = PathCache::new(
        (SIZE, SIZE),
        cost_fn(grid),
        ManhattanNeighborhood::new(SIZE, SIZE),
        fallback_off(),
    );
    let bytes = cache.to_bytes();

    // Header intact, payload cut in half.
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(&bytes[..bytes.len() / 2]).err(),
        Some(HpaSerError::Corrupt),
        "truncated payload must be Corrupt"
    );

    // Header plus junk that is not a serialized PathCache.
    let mut junk = b"HPAG\x01".to_vec();
    junk.extend_from_slice(&[0xDEu8; 64]);
    assert_eq!(
        PathCache::<ManhattanNeighborhood>::from_bytes(&junk).err(),
        Some(HpaSerError::Corrupt),
        "header plus junk must be Corrupt"
    );

    // Flip a byte deep in the payload; the result must not be a valid cache.
    let mut flipped = bytes.clone();
    flipped[bytes.len() - 3] ^= 0xFF;
    let res = PathCache::<ManhattanNeighborhood>::from_bytes(&flipped);
    assert!(
        res.is_err(),
        "a flipped payload byte must not deserialize into a valid cache"
    );
}

#[test]
fn error_traits_are_implemented() {
    let variants = [
        HpaSerError::TooShort,
        HpaSerError::BadMagic,
        HpaSerError::UnsupportedVersion(7),
        HpaSerError::Corrupt,
    ];
    for err in variants {
        assert!(!err.to_string().is_empty(), "{:?} has no message", err);
        let boxed: Box<dyn std::error::Error> = Box::new(err);
        assert!(
            !boxed.to_string().is_empty(),
            "{:?} boxed message empty",
            err
        );
    }

    let _: Box<dyn std::error::Error> = Box::new(HpaSerError::Corrupt);

    assert!(
        HpaSerError::UnsupportedVersion(7).to_string().contains('7'),
        "UnsupportedVersion message must mention the offending version"
    );
    assert!(
        !format!("{:?}", HpaSerError::TooShort).is_empty(),
        "Debug must be non-empty"
    );
}
