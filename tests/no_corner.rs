use hierarchical_pathfinding::prelude::*;

// 6 wide x 3 tall, row-major. solid = -1, open = 1.
//
// y=0: [ 1,  1, -1, -1,  1,  1]
// y=1: [ 1,  1,  1,  1, -1,  1]
// y=2: [ 1,  1, -1, -1,  1,  1]
//
// the x=2 column is a wall with its only opening at (2,1)-(3,1). beyond (3,1)
// the pinch at (4,1)/(3,0)/(3,2) leaves only diagonals with both corners solid
// as a way into the goal region.
fn variant_a_map() -> Vec<isize> {
    vec![1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1]
}

fn variant_b_map() -> Vec<isize> {
    let mut costs = variant_a_map();
    costs[0 * 6 + 3] = 1; // (3, 0) open
    costs
}

#[test]
fn no_path_when_both_corners_blocked() {
    let costs = variant_a_map();
    let hood = NoCornerNeighborhood::new(6, 3, costs.clone());
    let pathfinding = PathCache::new(
        (6, 3),
        hood.cost_fn(),
        hood.clone(),
        PathCacheConfig::with_chunk_size(3),
    );
    assert!(
        pathfinding
            .find_path((0, 0), (5, 2), hood.cost_fn())
            .is_none(),
        "NoCorner must not slip through the both-corners-solid pinch"
    );

    let moore_cost_fn = move |(x, y)| costs[y * 6 + x];
    let moore = PathCache::new(
        (6, 3),
        moore_cost_fn.clone(),
        MooreNeighborhood::new(6, 3),
        PathCacheConfig::with_chunk_size(3),
    );
    assert!(
        moore.find_path((0, 0), (5, 2), moore_cost_fn).is_some(),
        "Moore on the identical map must find the corner-cutting path"
    );
}

#[test]
fn path_when_one_corner_free() {
    let costs = variant_b_map();
    let hood = NoCornerNeighborhood::new(6, 3, costs);
    let pathfinding = PathCache::new(
        (6, 3),
        hood.cost_fn(),
        hood.clone(),
        PathCacheConfig::with_chunk_size(3),
    );
    assert!(
        pathfinding
            .find_path((0, 0), (5, 2), hood.cost_fn())
            .is_some(),
        "opening (3,0) frees the corner of the (3,1)->(4,0) diagonal"
    );
}

#[test]
fn opening_corner_restores_path() {
    let costs = variant_a_map();
    let hood = NoCornerNeighborhood::new(6, 3, costs);
    let mut pathfinding = PathCache::new(
        (6, 3),
        hood.cost_fn(),
        hood.clone(),
        PathCacheConfig::with_chunk_size(3),
    );
    assert!(pathfinding
        .find_path((0, 0), (5, 2), hood.cost_fn())
        .is_none());

    hood.set_tile((3, 0), 1);
    pathfinding.tiles_changed(&[(3, 0)], hood.cost_fn());

    assert!(
        pathfinding
            .find_path((0, 0), (5, 2), hood.cost_fn())
            .is_some(),
        "set_tile before tiles_changed must make the path findable"
    );
}
