//! A crate with the most common Neighborhoods

use crate::Point;
use std::fmt::Debug;
use std::sync::{Arc, RwLock};

/// Defines how a Path can move along the Grid.
///
/// Different use cases may have different conditions for Paths.
/// For example if movement is restricted to the 4 cardinal directions, any Paths generated should
/// reflect that by only containing those steps.
///
/// This Trait is a generalized solution to that problem. It provides a function to query all
/// neighboring Points of an existing Point and a Heuristic for how long it might take to reach
/// a goal from a Point.
///
/// The most common implementations of this Trait are already provided by this Module:
/// - [`ManhattanNeighborhood`] for Agents that can move
/// up, down, left or right
/// - [`MooreNeighborhood`] for Agents that can move
/// up, down, left, right, as well as the 4 diagonals (up-right, ...)
///
/// When the cargo feature `serialize` is enabled, implementors outside this crate must also
/// derive or implement `Serialize` and `Deserialize` for their own type: `PathCache<N>` only
/// derives them under that feature, and `PathCache::to_bytes`/`PathCache::from_bytes` require them.
pub trait Neighborhood: Clone + Debug {
    /// Provides all the Neighbors of a Point.
    ///
    /// The Neighbors should be written into `target`
    ///
    /// Note that it is not necessary to check weather the Tile at a Point is solid or not.
    /// That check is done later.
    fn get_all_neighbors(&self, point: Point, target: &mut Vec<Point>);
    /// Gives a Heuristic for how long it takes to reach `goal` from `point`.
    ///
    /// This is usually the Distance between the two Points in the Metric of your Neighborhood.
    ///
    /// The returned value _**must be**_ less than or equal to the real cost of walking from
    /// `point` to `goal`. See [A*](https://en.wikipedia.org/wiki/A*_search_algorithm) for details.
    ///
    /// If there is no proper way of calculation how long it takes, simply return 0. This will
    /// increase the time it takes to calculate the Path, but at least it will always be correct.
    fn heuristic(&self, point: Point, goal: Point) -> usize;
}

/// A Neighborhood for Agents moving along the 4 cardinal directions.
///
/// Also known as [Von Neumann Neighborhood](https://en.wikipedia.org/wiki/Von_Neumann_neighborhood),
/// Manhattan Metric or [Taxicab Geometry](https://en.wikipedia.org/wiki/Taxicab_geometry).
///
/// ```no_code
/// A: Agent, o: reachable in one step
///   o
///   |
/// o-A-o
///   |
///   o
/// ```
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ManhattanNeighborhood {
    width: usize,
    height: usize,
}

impl ManhattanNeighborhood {
    /// Creates a new `ManhattanNeighborhood`.
    ///
    /// `width` and `height` are the size of the Grid to move on.
    pub fn new(width: usize, height: usize) -> ManhattanNeighborhood {
        ManhattanNeighborhood { width, height }
    }
}

impl Neighborhood for ManhattanNeighborhood {
    fn get_all_neighbors(&self, point: Point, target: &mut Vec<Point>) {
        let (width, height) = (self.width, self.height);

        #[rustfmt::skip]
        static ALL_DELTAS: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];

        for (dx, dy) in &ALL_DELTAS {
            let x = point.0 as isize + dx;
            let y = point.1 as isize + dy;
            if x >= 0 && x < width as isize && y >= 0 && y < height as isize {
                target.push((x as usize, y as usize))
            }
        }
    }
    fn heuristic(&self, point: Point, goal: Point) -> usize {
        let diff_0 = if goal.0 > point.0 {
            goal.0 - point.0
        } else {
            point.0 - goal.0
        };
        let diff_1 = if goal.1 > point.1 {
            goal.1 - point.1
        } else {
            point.1 - goal.1
        };
        diff_0 + diff_1
    }
}

/// A Neighborhood for Agents moving along the 4 cardinal directions and the 4 diagonals.
///
/// Also known as [Moore Neighborhood](https://en.wikipedia.org/wiki/Moore_neighborhood),
/// [Maximum Metric](https://en.wikipedia.org/wiki/Chebyshev_distance) or Chebyshev Metric.
///
/// ```no_code
/// A: Agent, o: reachable in one step
/// o o o
///  \|/
/// o-A-o
///  /|\
/// o o o
/// ```
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct MooreNeighborhood {
    width: usize,
    height: usize,
}

impl MooreNeighborhood {
    /// Creates a new `MooreNeighborhood`.
    ///
    /// `width` and `height` are the size of the Grid to move on.
    pub fn new(width: usize, height: usize) -> MooreNeighborhood {
        MooreNeighborhood { width, height }
    }
}

impl Neighborhood for MooreNeighborhood {
    fn get_all_neighbors(&self, point: Point, target: &mut Vec<Point>) {
        let (width, height) = (self.width, self.height);

        #[rustfmt::skip]
        static ALL_DELTAS: [(isize, isize); 8] = [(0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1)];

        for (dx, dy) in ALL_DELTAS.iter() {
            let x = point.0 as isize + dx;
            let y = point.1 as isize + dy;
            if x >= 0 && x < width as isize && y >= 0 && y < height as isize {
                target.push((x as usize, y as usize))
            }
        }
    }
    fn heuristic(&self, point: Point, goal: Point) -> usize {
        let diff_0 = if goal.0 > point.0 {
            goal.0 - point.0
        } else {
            point.0 - goal.0
        };
        let diff_1 = if goal.1 > point.1 {
            goal.1 - point.1
        } else {
            point.1 - goal.1
        };
        diff_0.max(diff_1)
    }
}

/// A Neighborhood for Agents moving in 8 directions, but without cutting corners.
///
/// Diagonal movement is allowed unless **both** orthogonal corners adjacent to the
/// move are solid. If at least one corner is walkable, the diagonal is allowed.
///
/// Unlike the other Neighborhoods, this one needs to know which tiles are solid,
/// so it owns the costmap and shares it with [`PathCache`](crate::PathCache) via
/// [`cost_fn`](NoCornerNeighborhood::cost_fn). Update the map with
/// [`set_tile`](NoCornerNeighborhood::set_tile) **before** calling
/// [`tiles_changed`](crate::PathCache::tiles_changed).
///
/// ```
/// use hierarchical_pathfinding::prelude::*;
///
/// let costs = vec![1; 25];
/// let hood = NoCornerNeighborhood::new(5, 5, costs);
/// let pathfinding = PathCache::new(
///     (5, 5),
///     hood.cost_fn(),
///     hood.clone(),
///     PathCacheConfig::with_chunk_size(3),
/// );
/// let path = pathfinding.find_path((0, 0), (4, 4), hood.cost_fn());
/// assert!(path.is_some());
/// ```
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct NoCornerNeighborhood {
    width: usize,
    height: usize,
    costs: Arc<RwLock<Vec<isize>>>,
}

impl NoCornerNeighborhood {
    /// Creates the Neighborhood from a flat row-major costmap (negative == solid).
    pub fn new(width: usize, height: usize, costs: Vec<isize>) -> Self {
        assert_eq!(
            costs.len(),
            width * height,
            "costmap length must equal width * height"
        );
        Self {
            width,
            height,
            costs: Arc::new(RwLock::new(costs)),
        }
    }

    /// Returns the cost callback sharing this Neighborhood's storage.
    /// Pass this (and `neighborhood.clone()`) to `PathCache::new` / `tiles_changed`.
    pub fn cost_fn(&self) -> impl Sync + Send + Fn((usize, usize)) -> isize {
        let costs = self.costs.clone();
        let width = self.width;
        let height = self.height;
        move |(x, y)| {
            if x >= width || y >= height {
                return -1;
            }
            costs.read().expect("costmap poisoned")[y * width + x]
        }
    }

    /// Updates a single tile. Call for every changed tile, then rebuild affected
    /// chunks with `PathCache::tiles_changed`.
    pub fn set_tile(&self, point: (usize, usize), cost: isize) {
        let (x, y) = point;
        if x < self.width && y < self.height {
            self.costs.write().expect("costmap poisoned")[y * self.width + x] = cost;
        }
    }

    /// Out-of-bounds counts as blocked (safer for corner checks).
    fn blocked(&self, x: isize, y: isize) -> bool {
        if x < 0 || y < 0 || x >= self.width as isize || y >= self.height as isize {
            return true;
        }
        self.costs.read().expect("costmap poisoned")[y as usize * self.width + x as usize] < 0
    }
}

impl Neighborhood for NoCornerNeighborhood {
    fn get_all_neighbors(&self, point: Point, target: &mut Vec<Point>) {
        let (px, py) = (point.0 as isize, point.1 as isize);

        #[rustfmt::skip]
        static ORTHO: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];
        #[rustfmt::skip]
        static DIAG: [(isize, isize); 4] = [(1, -1), (1, 1), (-1, 1), (-1, -1)];

        for &(dx, dy) in ORTHO.iter() {
            let (x, y) = (px + dx, py + dy);
            if x >= 0 && x < self.width as isize && y >= 0 && y < self.height as isize {
                target.push((x as usize, y as usize));
            }
        }
        for &(dx, dy) in DIAG.iter() {
            let (x, y) = (px + dx, py + dy);
            if x < 0 || x >= self.width as isize || y < 0 || y >= self.height as isize {
                continue;
            }
            // both corners solid => no diagonal through the gap
            if self.blocked(px + dx, py) && self.blocked(px, py + dy) {
                continue;
            }
            target.push((x as usize, y as usize));
        }
    }

    fn heuristic(&self, point: Point, goal: Point) -> usize {
        let diff_0 = if goal.0 > point.0 {
            goal.0 - point.0
        } else {
            point.0 - goal.0
        };
        let diff_1 = if goal.1 > point.1 {
            goal.1 - point.1
        } else {
            point.1 - goal.1
        };
        diff_0.max(diff_1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    mod manhattan {
        use super::*;
        #[test]
        fn get_all_neighbors() {
            let neighborhood = ManhattanNeighborhood::new(5, 5);
            let mut target = vec![];
            neighborhood.get_all_neighbors((0, 2), &mut target);
            assert_eq!(target, vec![(0, 1), (1, 2), (0, 3)],);
        }

        #[test]
        fn heuristic() {
            let neighborhood = ManhattanNeighborhood::new(5, 5);
            assert_eq!(neighborhood.heuristic((3, 1), (0, 0)), 3 + 1);
        }
    }
    mod moore {
        use super::*;

        #[test]
        fn get_all_neighbors() {
            let neighborhood = MooreNeighborhood::new(5, 5);
            let mut target = vec![];
            neighborhood.get_all_neighbors((0, 2), &mut target);
            assert_eq!(target, vec![(0, 1), (1, 1), (1, 2), (1, 3), (0, 3)],);
        }

        #[test]
        fn heuristic() {
            let neighborhood = MooreNeighborhood::new(5, 5);
            assert_eq!(neighborhood.heuristic((3, 1), (0, 0)), 3);
        }
    }
    mod no_corner {
        use super::*;

        #[test]
        fn open_map_matches_moore() {
            let neighborhood = NoCornerNeighborhood::new(5, 5, vec![1; 25]);
            let moore = MooreNeighborhood::new(5, 5);
            let mut target = vec![];
            neighborhood.get_all_neighbors((2, 2), &mut target);
            let mut moore_target = vec![];
            moore.get_all_neighbors((2, 2), &mut moore_target);
            target.sort_unstable();
            moore_target.sort_unstable();
            assert_eq!(target, moore_target);
        }

        #[test]
        fn both_corners_solid_blocks_diagonal() {
            let mut costs = vec![1; 25];
            costs[2 * 5 + 3] = -1; // (3, 2)
            costs[3 * 5 + 2] = -1; // (2, 3)
            let neighborhood = NoCornerNeighborhood::new(5, 5, costs);
            let mut target = vec![];
            neighborhood.get_all_neighbors((2, 2), &mut target);
            assert!(!target.contains(&(3, 3)));
            assert!(target.contains(&(3, 2)));
            assert!(target.contains(&(2, 3)));
        }

        #[test]
        fn one_corner_free_allows_diagonal() {
            let mut costs = vec![1; 25];
            costs[2 * 5 + 3] = -1; // (3, 2)
            let neighborhood = NoCornerNeighborhood::new(5, 5, costs);
            let mut target = vec![];
            neighborhood.get_all_neighbors((2, 2), &mut target);
            assert!(target.contains(&(3, 3)));
        }

        #[test]
        fn map_edge_diagonal_blocked() {
            let mut costs = vec![1; 4];
            costs[1] = -1; // (1, 0)
            costs[2] = -1; // (0, 1)
            let neighborhood = NoCornerNeighborhood::new(2, 2, costs);
            let mut target = vec![];
            neighborhood.get_all_neighbors((0, 0), &mut target);
            target.sort_unstable();
            assert_eq!(target, vec![(0, 1), (1, 0)]);
            // nothing out of bounds
            for &(x, y) in &target {
                assert!(x < 2 && y < 2);
            }
        }
    }
}
