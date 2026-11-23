use hierarchical_pathfinding::internals::CacheInspector;
use hierarchical_pathfinding::neighbors::Neighborhood;
use hierarchical_pathfinding::prelude::*;
use std::env;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::time::Instant;

type Point = (usize, usize);

fn traversable(map: &[Vec<char>], (x, y): Point) -> bool {
    y < map.len() && x < map[y].len() && map[y][x] == '.'
}

fn tile_cost(map: &[Vec<char>], point: Point) -> isize {
    if traversable(map, point) {
        1
    } else {
        -1
    }
}

#[derive(Clone, Debug)]
struct MapNeighborhood {
    width: usize,
    height: usize,
    map: Vec<Vec<char>>,
}

impl MapNeighborhood {
    fn new(map: &[Vec<char>]) -> Self {
        Self {
            height: map.len(),
            width: map.first().map_or(0, |row| row.len()),
            map: map.to_vec(),
        }
    }

    fn traversable(&self, point: Point) -> bool {
        point.0 < self.width && point.1 < self.height && self.map[point.1][point.0] == '.'
    }
}

impl Neighborhood for MapNeighborhood {
    fn get_all_neighbors(&self, point: Point, target: &mut Vec<Point>) {
        const DELTAS: [(isize, isize); 8] = [
            (0, -1),
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
        ];

        let (x, y) = (point.0 as isize, point.1 as isize);

        for (dx, dy) in DELTAS {
            let nx = x + dx;
            let ny = y + dy;

            if nx < 0 || ny < 0 || nx >= self.width as isize || ny >= self.height as isize {
                continue;
            }

            let next = (nx as usize, ny as usize);

            if !self.traversable(next) {
                continue;
            }

            if dx != 0 && dy != 0 {
                let side_a = (point.0, next.1);
                let side_b = (next.0, point.1);

                if !self.traversable(side_a) && !self.traversable(side_b) {
                    continue;
                }
            }

            target.push(next);
        }
    }

    fn heuristic(&self, point: Point, goal: Point) -> usize {
        let dx = point.0.abs_diff(goal.0);
        let dy = point.1.abs_diff(goal.1);
        dx.max(dy)
    }
}

fn export_graph(
    planner: &PathCache<impl hierarchical_pathfinding::neighbors::Neighborhood + Sync>,
    filename: &str,
    chunk_size: usize,
) -> std::io::Result<()> {
    let mut file = File::create(filename)?;

    writeln!(file, "graph HPA {{")?;
    writeln!(file, "  graph [overlap=false, splines=true];")?;
    writeln!(
        file,
        "  node [shape=circle, width=0.12, height=0.12, label=\"\"];"
    )?;
    writeln!(file, "  edge [color=gray70];")?;

    for node in planner.inspect_nodes() {
        let (x, y) = node.pos();
        let id = node.id();
        let chunk_x = x / chunk_size;
        let chunk_y = y / chunk_size;

        writeln!(
            file,
            "  n{} [pos=\"{},{}!\", tooltip=\"id={}, pos=({},{}), chunk=({}, {})\"];",
            id, x, y, id, x, y, chunk_x, chunk_y
        )?;

        for (neighbor, cost) in node.connected() {
            // Graf jest nieskierowany, więc zapisujemy każdą krawędź raz.
            if id < neighbor.id() {
                writeln!(
                    file,
                    "  n{} -- n{} [label=\"{}\"];",
                    id,
                    neighbor.id(),
                    cost
                )?;
            }
        }
    }

    writeln!(file, "}}")?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    if args.len() != 3 {
        eprintln!("Użycie: cargo run --release --example movingai_test -- MAP.scen MAP.map");
        std::process::exit(1);
    }

    // Wczytanie mapy Moving AI.
    let map_text = fs::read_to_string(&args[2])?;
    let lines: Vec<&str> = map_text.lines().collect();

    let height: usize = lines
        .iter()
        .find_map(|line| line.strip_prefix("height "))
        .ok_or("Brak pola height w mapie")?
        .trim()
        .parse()?;

    let width: usize = lines
        .iter()
        .find_map(|line| line.strip_prefix("width "))
        .ok_or("Brak pola width w mapie")?
        .trim()
        .parse()?;

    let map_start = lines
        .iter()
        .position(|line| line.trim() == "map")
        .ok_or("Brak nagłówka map")?;

    let map: Vec<Vec<char>> = lines[map_start + 1..]
        .iter()
        .take(height)
        .map(|line| line.chars().collect())
        .collect();

    if map.len() != height || map.iter().any(|row| row.len() != width) {
        return Err("Wymiary mapy nie zgadzają się z nagłówkiem".into());
    }

    println!("Mapa: {} x {}", width, height);

    // Wybierz najdalszą parę poprawnych punktów z pliku scenariuszy.
    let scen_text = fs::read_to_string(&args[1])?;
    let mut selected: Option<(f64, Point, Point, f64)> = None;

    for line in scen_text.lines().skip(1) {
        let fields: Vec<&str> = line.split_whitespace().collect();
        if fields.len() < 9 {
            continue;
        }

        let sx: usize = fields[4].parse()?;
        let sy: usize = fields[5].parse()?;
        let gx: usize = fields[6].parse()?;
        let gy: usize = fields[7].parse()?;
        let reference_cost: f64 = fields[8].parse()?;

        if sx >= width || gx >= width || sy >= height || gy >= height {
            continue;
        }

        let start = (sx, sy);
        let goal = (gx, gy);

        if !traversable(&map, start) || !traversable(&map, goal) {
            continue;
        }

        let dx = sx as f64 - gx as f64;
        let dy = sy as f64 - gy as f64;
        let distance = (dx * dx + dy * dy).sqrt();

        if selected.as_ref().is_none_or(|s| distance > s.0) {
            selected = Some((distance, start, goal, reference_cost));
        }
    }

    let (distance, start, goal, reference_cost) =
        selected.ok_or("Nie znaleziono poprawnej pary punktów")?;

    println!("Start: {:?}", start);
    println!("Cel: {:?}", goal);
    println!("Odległość w linii prostej: {:.2}", distance);
    println!("Koszt referencyjny Moving AI: {:.5}", reference_cost);

    // Budowa hierarchicznego cache.
    let build_start = Instant::now();

    let mut planner = PathCache::new(
        (width, height),
        |p| tile_cost(&map, p),
        MapNeighborhood::new(&map),
        PathCacheConfig::with_chunk_size(256),
    );

    let build_time = build_start.elapsed();
    println!("Czas budowy cache: {:.3?}", build_time);

    export_graph(&planner, "hierarchy3.dot", 256)?;
    println!("Zapisano graf abstrakcyjny: hierarchy.dot");

    // Wyszukiwanie trasy.
    let search_start = Instant::now();

    let path = planner.find_path(start, goal, |p| tile_cost(&map, p));

    let search_time = search_start.elapsed();
    println!("Czas wyszukiwania: {:.3?}", search_time);

    let Some(path) = path else {
        println!("WYNIK: nie znaleziono trasy.");
        return Ok(());
    };

    println!("Koniec abstrakcyjnej ścieżki: {:?}", path.clone().last());
    println!("Deklarowana liczba kroków: {}", path.length());
    println!("Deklarowany koszt: {}", path.cost());

    let path_length = path.length();
    let path_cost = path.cost();

    let points: Vec<Point> = path.resolve(|p| tile_cost(&map, p));

    println!("Długość według biblioteki: {}", path_length);
    println!("Koszt według biblioteki: {}", path_cost);
    println!("Liczba punktów po resolve(): {}", points.len());

    println!("WYNIK: znaleziono trasę.");
    println!("Długość według biblioteki: {}", path_length);
    println!("Koszt według biblioteki: {}", path_cost);
    println!("Liczba punktów zwróconych przez iterator: {}", points.len());

    // Kontrola poprawności ścieżki.
    let mut valid = true;
    let mut previous = start;

    for &point in &points {
        if !traversable(&map, point) {
            eprintln!("BŁĄD: ścieżka przechodzi przez przeszkodę: {:?}", point);
            valid = false;
            break;
        }

        let dx = previous.0.abs_diff(point.0);
        let dy = previous.1.abs_diff(point.1);

        if dx > 1 || dy > 1 || (dx == 0 && dy == 0) {
            eprintln!(
                "BŁĄD: nieprawidłowe przejście {:?} -> {:?}",
                previous, point
            );
            valid = false;
            break;
        }

        // Ruch diagonalny jest niedozwolony, jeśli przecina narożnik
        // pomiędzy dwiema przeszkodami.
        if dx == 1 && dy == 1 {
            let side_a = (previous.0, point.1);
            let side_b = (point.0, previous.1);

            if !traversable(&map, side_a) && !traversable(&map, side_b) {
                eprintln!(
                    "BŁĄD: przejście przez zablokowany narożnik {:?} -> {:?}",
                    previous, point
                );
                valid = false;
                break;
            }
        }

        previous = point;
    }

    if previous != goal {
        eprintln!(
            "UWAGA: ostatni punkt ścieżki to {:?}, oczekiwano {:?}",
            previous, goal
        );
        valid = false;
    }

    println!(
        "Kontrola geometrii ścieżki: {}",
        if valid { "OK" } else { "BŁĄD" }
    );

    // Zapis wizualizacji tekstowej.
    let mut output = map.clone();

    for &(x, y) in &points {
        output[y][x] = 'P';
    }

    output[start.1][start.0] = 'S';
    output[goal.1][goal.0] = 'G';

    let mut rendered = String::new();
    for row in &output {
        rendered.extend(row.iter());
        rendered.push('\n');
    }

    fs::write("path_overlay.txt", rendered)?;
    println!("Zapisano mapę z trasą: path_overlay.txt");

    Ok(())
}
