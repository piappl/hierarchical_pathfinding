// examples/barn_check.rs
// Walidacja hierarchical_pathfinding na dataset BARN, proces step-by-step:
//   STEP 1: ladowanie mapy i BUDOWA GRAFU HPA* (osobno)
//   STEP 2: WYZNACZENIE TRASY i zapis SAMEJ LISTY PORTALI do pliku (osobno)
// Uzyycie: cargo run --release --example barn_check <i>
//         cargo run --release --example barn_check -- --all
use hierarchical_pathfinding::prelude::*;
use std::path::Path;
use std::time::Instant;

type Pt = (usize, usize); // (x = kolumna, y = wiersz)

// ---- minimalny odczyt .npy (v1.0, descr '<i8', fortran_order False) ----
fn read_npy_i64(path: &Path) -> Result<(Vec<i64>, Vec<usize>), String> {
    let b = std::fs::read(path).map_err(|e| format!(" {}: {e}", path.display()))?;
    if b.len() < 12 || &b[..6] != b"\x93NUMPY" || b[6] != 1 {
        return Err(format!(
            "{}: nie obslugiwany npy (oczekiwano v1.0)",
            path.display()
        ));
    }
    let hlen = u16::from_le_bytes([b[8], b[9]]) as usize;
    let header = std::str::from_utf8(&b[10..10 + hlen]).map_err(|_| "zly naglowek")?;
    if !header.contains("'descr': '<i8'") {
        return Err(format!("nieobslugiwany descr w: {}", header.trim()));
    }
    if !header.contains("'fortran_order': False") {
        return Err("fortran_order=True nieobslugiwany".into());
    }
    let shape_s = header
        .split("'shape':")
        .nth(1)
        .and_then(|s| s.split(')').next())
        .ok_or("brak shape")?;
    let shape: Vec<usize> = shape_s
        .trim_start_matches(" (")
        .trim_end_matches(',')
        .split(',')
        .map(|x| x.trim().parse::<usize>().map_err(|_| "zly shape"))
        .collect::<Result<_, _>>()?;
    let n: usize = shape.iter().product();
    let data = &b[10 + hlen..];
    if data.len() < n * 8 {
        return Err("plik uciety".into());
    }
    let vals: Vec<i64> = (0..n)
        .map(|k| i64::from_le_bytes(data[k * 8..k * 8 + 8].try_into().unwrap()))
        .collect();
    Ok((vals, shape))
}

fn load_grid(i: usize) -> Result<(Vec<u8>, usize, usize), String> {
    let (v, shape) =
        read_npy_i64(&Path::new("dataset/BARN_dataset/grid_files").join(format!("grid_{i}.npy")))?;
    if shape != [30, 30] {
        return Err(format!("nieoczekiwany shape {shape:?}"));
    }
    Ok((
        v.into_iter().map(|x| if x != 0 { 1 } else { 0 }).collect(),
        30,
        30,
    ))
}

fn load_path_endpoints(i: usize) -> Result<(Pt, Pt, usize), String> {
    let (v, shape) =
        read_npy_i64(&Path::new("dataset/BARN_dataset/path_files").join(format!("path_{i}.npy")))?;
    if shape.len() != 2 || shape[1] != 2 || shape[0] < 2 {
        return Err("sciezka referencyjna pusta/lacza".into());
    }
    let row_col = |k: usize| (v[2 * k + 1] as usize, v[2 * k] as usize); // (col,row)
    Ok((row_col(0), row_col(shape[0] - 1), shape[0] - 1))
}

fn check(name: &str, cond: bool) {
    println!("    [{}] {name}", if cond { "OK  " } else { "FAIL" });
    if !cond {
        std::process::exit(1);
    }
}

// Zwroci Some((hpa_len, ref_len, lower_bound)) albo None gdy mapa nie do zapytania.
fn run_one(i: usize, quiet: bool) -> Option<(usize, usize, usize)> {
    let t0 = Instant::now();

    // ============ STEP 1: mapa -> GRAF ============
    if !quiet {
        println!("=== MAPA {i} ===");
        print!("STEP 1: budowa grafu HPA*... ");
    }
    let (grid, w, h) = load_grid(i).ok()?;
    let cost_fn = {
        let g = grid.clone();
        move |p: Pt| -> isize {
            match g.get(p.1 * w + p.0) {
                Some(1) | None => -1,
                _ => 1,
            }
        }
    };
    let ((sx, sy), (gx, gy), ref_len) = load_path_endpoints(i).ok()?;
    if cost_fn((sx, sy)) < 0 || cost_fn((gx, gy)) < 0 {
        return None;
    }
    let pf = PathCache::new(
        (w, h),
        cost_fn.clone(),
        ManhattanNeighborhood::new(w, h),
        PathCacheConfig::with_chunk_size(6),
    );
    let n_nodes = pf.inspect_nodes().count();
    let t_graph = t0.elapsed();
    if !quiet {
        println!("OK ({} portali/westlow, czas {:?})", n_nodes, t_graph);
    }

    // ============ STEP 2: graf -> TRASA -> TYLKO PORTALE ============
    let t1 = Instant::now();
    if !quiet {
        print!(
            "STEP 2: trasa {:?} -> {:?} i zapis portali... ",
            (sx, sy),
            (gx, gy)
        );
    }
    let Some((path, portals)) = pf.find_path_with_portals((sx, sy), (gx, gy), cost_fn.clone())
    else {
        if !quiet {
            println!("brak sciezki");
        }
        return None;
    };
    let t_plan = t1.elapsed();
    let mut pts: Vec<Pt> = vec![(sx, sy)];
    pts.extend(path.clone().collect::<Vec<Pt>>()); // iterator AbstractPath pomija start

    if !quiet {
        println!("OK (czas {:?})", t_plan);
        println!(
            "  koszt={}, dl. sciezki={} punktow (referencja {} punktow)",
            path.cost(),
            pts.len(),
            ref_len + 1
        );
        println!("  lista portali ({}): {:?}", portals.len(), portals);
        std::fs::create_dir_all("out").ok();
        let f = format!("out/portals_{i}.txt");
        let body = portals
            .iter()
            .map(|p| format!("{} {}", p.0, p.1))
            .collect::<Vec<_>>()
            .join("\n");
        std::fs::write(&f, format!("{body}\n")).expect("zapis out/");
        println!("  zapisano TYLKO portale -> {f}");
    }

    // walidacja
    check(
        "sciezka zaczyna sie w starcie",
        pts.first() == Some(&(sx, sy)),
    );
    check("sciezka konczy sie w celu", pts.last() == Some(&(gx, gy)));
    check(
        "wszystkie pola wolne",
        pts.iter().all(|p| p.0 < w && p.1 < h && cost_fn(*p) > 0),
    );
    check(
        "ruchy 4-sasiednie",
        pts.windows(2)
            .all(|ab| ab[0].0.abs_diff(ab[1].0) + ab[0].1.abs_diff(ab[1].1) == 1),
    );
    check(
        "portale [start,...,goal]",
        portals.first() == Some(&(sx, sy)) && portals.last() == Some(&(gx, gy)),
    );
    let lower = sx.abs_diff(gx) + sy.abs_diff(gy) + 1; // manhattan+1 = dolne ograniczenie dla 4-sasiednich
    check(
        "HPA* nie krotsza niz dolne ograniczenie 4-sasiedzkie",
        pts.len() >= lower,
    );
    if !quiet {
        println!(
            "  (info) referencja 8-sasiedna ma {} punktow; HPA* 4-sasiednie: {} (dolne ogr. {})",
            ref_len + 1,
            pts.len(),
            lower
        );
    }
    Some((pts.len(), ref_len + 1, lower))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if !Path::new("dataset/BARN_dataset").exists() {
        eprintln!("Brak dataset/BARN_dataset — uruchom z katalogu glownego repo.");
        std::process::exit(1);
    }
    if args.first().map(|a| a == "--all").unwrap_or(false) {
        let (mut ok, mut skip, mut sum) = (0u32, 0u32, 0i64);
        let mut worse = Vec::new();
        for i in 0..300 {
            match run_one(i, true) {
                Some((hpa, r, lb)) => {
                    ok += 1;
                    sum += (hpa - r) as i64;
                    if hpa < lb {
                        worse.push(i);
                    }
                }
                None => skip += 1,
            }
        }
        println!("\n== PODSUMOWANIE: {ok} map zwalidowanych, {skip} pominietych ==");
        println!(
            "srednia delta dlugosci HPA* vs referencja: {:.2} punktow",
            sum as f64 / ok.max(1) as f64
        );
        println!("indeksy z wynikiem PONIZEJ dolnego ograniczenia (ma byc puste): {worse:?}");
        if !worse.is_empty() {
            std::process::exit(1);
        }
    } else {
        let i: usize = args.first().and_then(|a| a.parse().ok()).unwrap_or(0);
        if run_one(i, false).is_none() {
            println!("mapa {i}: nie zbudowano zapytania");
        }
    }
}
