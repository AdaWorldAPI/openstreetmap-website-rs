//! `building_probe` — **P4**, the closed-way template census.
//!
//! ```text
//! building_probe <input.osm.pbf>
//! ```
//!
//! `osm-chain-encoding-v1` §2(c) proposes encoding a building footprint as an
//! anchor cell + one orientation + one turn bit and a length per corner. §4
//! makes P4 the gate. Three columns, all answerable on the extract with no bake:
//!
//! 1. **Refcount by referrer class** — P1 measured 34.66 % of referenced nodes
//!    at refcount ≥ 2 and called them junctions. That conflates road topology
//!    with **building adjacency**: terraced blocks share boundary nodes between
//!    two `building` ways, which are CAD adjacencies and not routing nodes. This
//!    splits them.
//! 2. **Rectilinear fraction** — what share of `building=*` closed ways is
//!    actually rectangular, or rectilinear (every corner a right angle) at the
//!    P2 threshold. The falsifier: low → §2(c) is dead and buildings stay rows.
//! 3. **Shared-wall incidence** — how many building ways share a whole edge with
//!    a neighbour. Falsifier: high *together with* a high (2) → the template
//!    needs a shared-edge form before it ships, or duplication must be priced.
//!
//! # Angles are measured in cell space, and that is not a shortcut
//!
//! Mercator is **conformal**: it preserves angles locally. So a right angle on
//! the ground is a right angle in the projected grid, and measuring corners in
//! cell coordinates is measuring them where the encoding would actually live —
//! integer vectors, no geodesy, no datum. Lengths are converted back to metres
//! only for reporting, using the cell's ground size at the footprint's own
//! latitude (`40075017 / 2^32 · cos φ`, ≈ 5.67 mm at Berlin — matching `tms.rs`).
//!
//! # What "rectilinear" means here, precisely
//!
//! A footprint's dominant orientation θ is the bearing of its longest edge,
//! taken mod 90°. Each edge's deviation is its angular distance to the nearest
//! `θ + k·90°`, and the **displacement** that deviation causes is
//! `edge_length · sin(deviation)` — how far the far end of that edge sits from
//! where a perfectly rectilinear footprint would put it.
//!
//! Reporting displacement rather than angle is deliberate: a 2° error on a 40 m
//! wall moves a corner 1.4 m, the same 2° on a 2 m wall moves it 7 cm, and only
//! one of those matters. The threshold is P2's — one z=24 cell, 0.27–1.69 m,
//! GNSS-fix order — because that is the tolerance the template contract is
//! written against, and demanding a z=32 cell (1.13 mm) would falsify every
//! footprint against survey noise rather than against shape.

use std::collections::{HashMap, HashSet};

use osm_soa_bake::tms;
use osmpbf::{Element, ElementReader};

/// Earth's equatorial circumference, metres — for cell-size-to-metres only.
const EQUATORIAL_M: f64 = 40_075_017.0;

/// P2's threshold, in metres: one z=24 cell at its coarse end.
const Z24_CELL_M: f64 = 1.69;

/// Referrer classes a node can be pulled into.
const CLASS_BUILDING: u8 = 1;
const CLASS_HIGHWAY: u8 = 2;
const CLASS_OTHER: u8 = 4;

/// Ground size of one z=32 cell at latitude `lat`, in metres.
fn cell_metres(lat: f64) -> f64 {
    EQUATORIAL_M / 4_294_967_296.0 * lat.to_radians().cos()
}

/// Quantile of a sorted slice.
fn q(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

/// One footprint's worst corner displacement, its vertex count, and its size.
struct Fit {
    /// Distinct vertices (the repeated closing node dropped).
    corners: usize,
    /// Worst per-edge displacement from a perfectly rectilinear footprint.
    worst_m: f64,
    /// Longest edge, metres — the size the verdict must be stratified by.
    ///
    /// The threshold is a DISPLACEMENT, so a given angular error passes more
    /// easily on a small footprint: 5° of skew moves a 2 m wall 17 cm and a
    /// 40 m wall 3.5 m. A single headline percentage would therefore be carried
    /// by garden sheds. Reporting per size band is what keeps it honest.
    longest_m: f64,
}

/// Measure how far a closed way is from rectilinear, in metres.
///
/// `None` for anything with fewer than three usable corners — a degenerate ring
/// has no shape to fit and counting it either way would be fiction.
fn rectilinear_fit(pts: &[(f64, f64)]) -> Option<Fit> {
    // Drop the repeated closing vertex, then any zero-length edge.
    let ring = &pts[..pts.len() - 1];
    if ring.len() < 3 {
        return None;
    }
    let lat = ring.iter().map(|p| p.1).sum::<f64>() / ring.len() as f64;
    let m_per_cell = cell_metres(lat);

    // Vertices in cell space, where Mercator's conformality makes the angles
    // the same ones a ground observer would measure.
    let cells: Vec<(f64, f64)> = ring
        .iter()
        .map(|&(lon, lat)| {
            let c = tms::point_to_cell(lon, lat);
            (f64::from(c.x), f64::from(c.y_xyz))
        })
        .collect();

    let mut edges: Vec<(f64, f64)> = Vec::with_capacity(cells.len());
    for i in 0..cells.len() {
        let a = cells[i];
        let b = cells[(i + 1) % cells.len()];
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len = dx.hypot(dy);
        if len > 0.0 {
            edges.push((dy.atan2(dx), len));
        }
    }
    if edges.len() < 3 {
        return None;
    }

    // Dominant orientation: the longest edge's bearing, mod 90°. The longest
    // edge is the one a mis-set θ would penalise most, so anchoring on it is
    // the conservative choice rather than a flattering one.
    let quarter = std::f64::consts::FRAC_PI_2;
    let theta = edges
        .iter()
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|&(ang, _)| ang.rem_euclid(quarter))
        .unwrap_or(0.0);

    let mut worst = 0.0f64;
    for &(ang, len) in &edges {
        // Angular distance to the nearest θ + k·90°.
        let d = (ang - theta).rem_euclid(quarter);
        let dev = d.min(quarter - d);
        worst = worst.max(len * dev.sin() * m_per_cell);
    }
    Some(Fit {
        corners: ring.len(),
        worst_m: worst,
        longest_m: edges.iter().map(|&(_, l)| l).fold(0.0, f64::max) * m_per_cell,
    })
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: building_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    // ── Pass 1: node coordinates. ──
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(8_000_000);
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| match el {
            Element::Node(n) => {
                coords.insert(n.id(), (n.lon(), n.lat()));
            }
            Element::DenseNode(n) => {
                coords.insert(n.id(), (n.lon(), n.lat()));
            }
            _ => {}
        })
        .expect("pass 1");

    // ── Pass 2: ways. ──
    let mut refs: HashMap<i64, u32> = HashMap::with_capacity(8_000_000);
    let mut classes: HashMap<i64, u8> = HashMap::with_capacity(8_000_000);
    // Undirected edge → how many BUILDING ways use it.
    let mut wall_use: HashMap<(i64, i64), u32> = HashMap::with_capacity(3_000_000);
    // Per building way, its edge list, so column (iii) can be counted after.
    let mut building_edges: Vec<Vec<(i64, i64)>> = Vec::with_capacity(600_000);

    let mut buildings = 0u64;
    let mut buildings_closed = 0u64;
    let mut fits: Vec<f64> = Vec::with_capacity(600_000);
    // (longest edge, worst displacement) so the verdict can be stratified.
    let mut sized: Vec<(f64, f64)> = Vec::with_capacity(600_000);
    let mut rectangles = 0u64;
    let mut degenerate = 0u64;

    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Way(w) = el else { return };
            let is_building = w
                .tags()
                .any(|(k, _)| k == "building" || k == "building:part");
            let is_highway = w.tags().any(|(k, _)| k == "highway");
            let class = if is_building {
                CLASS_BUILDING
            } else if is_highway {
                CLASS_HIGHWAY
            } else {
                CLASS_OTHER
            };

            let ids: Vec<i64> = w.refs().collect();
            for &id in &ids {
                *refs.entry(id).or_insert(0) += 1;
                *classes.entry(id).or_insert(0) |= class;
            }

            if !is_building {
                return;
            }
            buildings += 1;
            let closed = ids.len() >= 4 && ids.first() == ids.last();
            if !closed {
                return;
            }
            buildings_closed += 1;

            // (iii) shared walls: every undirected edge of this footprint.
            let mut mine: Vec<(i64, i64)> = Vec::with_capacity(ids.len());
            for pair in ids.windows(2) {
                let e = (pair[0].min(pair[1]), pair[0].max(pair[1]));
                *wall_use.entry(e).or_insert(0) += 1;
                mine.push(e);
            }
            building_edges.push(mine);

            // (ii) rectilinear fit.
            let pts: Vec<(f64, f64)> = ids
                .iter()
                .filter_map(|id| coords.get(id).copied())
                .collect();
            if pts.len() != ids.len() {
                // Clipped out of the extract; not a shape failure.
                degenerate += 1;
                return;
            }
            match rectilinear_fit(&pts) {
                Some(f) => {
                    if f.corners == 4 && f.worst_m < Z24_CELL_M {
                        rectangles += 1;
                    }
                    fits.push(f.worst_m);
                    sized.push((f.longest_m, f.worst_m));
                }
                None => degenerate += 1,
            }
        })
        .expect("pass 2");

    // ── Column (i): refcount ≥ 2, split by referrer class. ──
    let mut junction_total = 0u64;
    let mut junction_building_only = 0u64;
    let mut junction_highway = 0u64;
    let mut junction_mixed = 0u64;
    let mut junction_other_only = 0u64;
    for (&id, &c) in &refs {
        if c < 2 {
            continue;
        }
        junction_total += 1;
        let m = classes.get(&id).copied().unwrap_or(0);
        let b = m & CLASS_BUILDING != 0;
        let h = m & CLASS_HIGHWAY != 0;
        if b && !h {
            junction_building_only += 1;
        } else if h && !b {
            junction_highway += 1;
        } else if b && h {
            junction_mixed += 1;
        } else {
            junction_other_only += 1;
        }
    }

    let pct = |n: u64, d: u64| 100.0 * n as f64 / d.max(1) as f64;
    println!("── P4(i): refcount ≥ 2, by referrer class ──");
    println!("junction-ish nodes    {junction_total:>12}");
    println!(
        "  building ways only  {junction_building_only:>12}  ({:.2}%)  CAD adjacency, NOT routing",
        pct(junction_building_only, junction_total)
    );
    println!(
        "  highway, no building{junction_highway:>12}  ({:.2}%)  the real routing graph",
        pct(junction_highway, junction_total)
    );
    println!(
        "  both                {junction_mixed:>12}  ({:.2}%)",
        pct(junction_mixed, junction_total)
    );
    println!(
        "  neither             {junction_other_only:>12}  ({:.2}%)",
        pct(junction_other_only, junction_total)
    );
    println!();

    fits.sort_by(f64::total_cmp);
    let under = fits.iter().filter(|&&m| m < Z24_CELL_M).count() as u64;
    println!("── P4(ii): rectilinear fraction of building footprints ──");
    println!("building ways         {buildings:>12}");
    println!("  closed rings        {buildings_closed:>12}");
    println!("  measured            {:>12}", fits.len());
    println!("  degenerate/clipped  {degenerate:>12}");
    println!(
        "RECTILINEAR (<{Z24_CELL_M} m) {under:>12}  ({:.2}% of measured)",
        pct(under, fits.len() as u64)
    );
    println!(
        "  of which 4-corner   {rectangles:>12}  ({:.2}% of measured)",
        pct(rectangles, fits.len() as u64)
    );
    println!("worst-corner displacement from rectilinear:");
    println!("  median              {:>12.4} m", q(&fits, 0.5));
    println!("  p75                 {:>12.4} m", q(&fits, 0.75));
    println!("  p95                 {:>12.4} m", q(&fits, 0.95));
    println!("  p99                 {:>12.4} m", q(&fits, 0.99));
    println!(
        "  max                 {:>12.4} m",
        fits.last().copied().unwrap_or(0.0)
    );
    // Stratified by size — the anti-vacuity half. A headline percentage that
    // rests on small footprints would read as "buildings are orthogonal" when it
    // really said "small things move little".
    println!("rectilinear by footprint size (longest edge):");
    for (lo, hi, label) in [
        (0.0, 5.0, "     < 5 m"),
        (5.0, 10.0, "  5 –  10 m"),
        (10.0, 20.0, " 10 –  20 m"),
        (20.0, 40.0, " 20 –  40 m"),
        (40.0, f64::INFINITY, "    ≥ 40 m"),
    ] {
        let band: Vec<f64> = sized
            .iter()
            .filter(|&&(l, _)| l >= lo && l < hi)
            .map(|&(_, w)| w)
            .collect();
        let ok = band.iter().filter(|&&m| m < Z24_CELL_M).count() as u64;
        println!(
            "  {label}  {:>9}  rectilinear {:>6.2}%",
            band.len(),
            pct(ok, band.len() as u64)
        );
    }
    println!();

    // ── Column (iii): shared walls. ──
    let shared_edges: HashSet<(i64, i64)> = wall_use
        .iter()
        .filter(|(_, &n)| n >= 2)
        .map(|(&e, _)| e)
        .collect();
    let with_shared = building_edges
        .iter()
        .filter(|es| es.iter().any(|e| shared_edges.contains(e)))
        .count() as u64;
    let total_edges: u64 = building_edges.iter().map(|e| e.len() as u64).sum();
    let distinct = wall_use.len() as u64;
    let duplicate_instances = total_edges - distinct;
    println!("── P4(iii): shared-wall incidence ──");
    println!("footprint edge slots  {total_edges:>12}");
    println!("  distinct edges      {distinct:>12}");
    println!(
        "  shared by ≥2 ways   {:>12}  ({:.2}% of distinct)",
        shared_edges.len(),
        pct(shared_edges.len() as u64, distinct)
    );
    println!(
        "footprints w/ a shared edge {with_shared:>6}  ({:.2}% of closed rings)",
        pct(with_shared, buildings_closed)
    );
    // The falsifier says a high incidence means the template needs a shared-edge
    // form OR duplication must be priced. This is the price: incidence counts
    // FOOTPRINTS, cost counts EDGES, and a shared wall is one varint — not one
    // footprint. The two numbers can differ by an order of magnitude.
    println!(
        "PRICE OF DUPLICATION  {duplicate_instances:>12}  ({:.2}% of edge slots — what a \
shared-edge form would save)",
        pct(duplicate_instances, total_edges)
    );
}
