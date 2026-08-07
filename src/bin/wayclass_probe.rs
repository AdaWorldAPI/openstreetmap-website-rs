//! `wayclass_probe` — **P5**, the way-class census: what the encoding is
//! actually asked to carry, beyond the drivable road.
//!
//! ```text
//! wayclass_probe <input.osm.pbf>
//! ```
//!
//! P4 classified a node's referrers as `building` / `highway` / other, and
//! `street.rs` quotes a **"drivable network"** of 96,340 named ways. Both
//! readings collapse every pedestrian, cycle and access way into one bucket or
//! drop it, so neither can answer the question that matters for a map people
//! walk on: **is the footpath between two houses in the corpus, and does it
//! survive the model?**
//!
//! OSM does have way classes, and more of them than a consumer expects — but
//! the interesting part is not the list. It is that **the same physical thing is
//! mapped two incompatible ways**, and a reader that knows only one of them
//! silently loses the other. Five columns:
//!
//! 1. **The `highway=*` census** — every value, by ways / nodes / metres. The
//!    ground truth the other columns are read against.
//! 2. **Separate geometry vs attribute-on-the-road** — a Bürgersteig is either
//!    its own `highway=footway` + `footway=sidewalk` way *or* a `sidewalk=*` tag
//!    on the carriageway with no geometry of its own. Same for cycling
//!    (`highway=cycleway` vs `cycleway[:side]=*`). Counting one form is
//!    undercounting; the falsifier is that both forms are non-trivial.
//! 3. **Zufahrten** — `highway=service` split by `service=*`, because a driveway,
//!    a parking aisle and an alley are one `highway` value and three different
//!    things to route on.
//! 4. **The literal case** — foot ways flanked by buildings on **both** sides,
//!    at two radii, against a road control. This is the column that can lie, so
//!    it carries its own falsifier (below).
//! 5. **Survival in our model** — the share of each class that is **unnamed**,
//!    because [`street::edge_name`] treats `NAME_NONE` as absent: an unnamed way
//!    is read and baked, but it is not a member of any street projection.
//!
//! [`street::edge_name`]: osm_soa_bake::street::edge_name
//!
//! # Column 4 is a shape test, not a density test — and it says so
//!
//! "A path with a building on each side" is satisfied by an ordinary residential
//! street: houses line both sides of it too. Measured at one radius the column
//! would report a large number and mean nothing.
//!
//! So it is measured at **two** radii and against **road controls**. A Fußweg
//! between two houses is a 2–6 m gap, so its flanks sit within ~8 m; a
//! residential street is 8–20 m building-to-building, so it fails at 8 m and
//! passes at 25 m. If the foot classes and the road controls come out alike at
//! **8 m**, this column is measuring built-up density and must be discarded —
//! that is its kill condition, and the control is printed next to the result
//! rather than left for the reader to request.
//!
//! # Method
//!
//! Proximity is computed in a **local equirectangular frame** anchored at the
//! extract's own mean position, with the true meridional and normal radii of
//! curvature at that latitude (the same radii [`geodesy::segment_metres`] uses).
//! Over a city extract's ~50 km span the frame's scale error is well under
//! 0.1 %, which is nothing against an 8 m threshold; it would not be acceptable
//! for a country, and the frame is rebuilt from the data rather than hardcoded
//! so the error scales with the extract rather than silently persisting.
//!
//! Distance is **point-to-segment**, not point-to-midpoint: a 40 m path beside a
//! 6 m house is flanked along part of its length, and midpoint-only would score
//! it by whatever happens to sit at its middle. Side is the sign of the 2-D
//! cross product, so a node exactly on the centreline counts as neither side and
//! cannot flank a path by itself — which is what makes a footway *attached* to a
//! building (a shared node, an entrance link) not self-flanking.
//!
//! [`geodesy::segment_metres`]: osm_soa_bake::geodesy::segment_metres
//!
//! # Exclusions, stated rather than buried
//!
//! Buildings are represented by their **nodes**, not their filled area, so a
//! path beside a very long blank wall whose corners are far away is scored by
//! the corners. Node density is the shape of the error and it is
//! anti-conservative for large buildings only. Relation (multipolygon) buildings
//! contribute through their member ways, which carry the nodes; a building
//! mapped as a relation with no way member would be missed, and Berlin has none
//! that matter at this threshold.

use std::collections::{HashMap, HashSet};

use osm_soa_bake::geodesy::{meridional_radius, normal_radius, polyline_metres};
use osmpbf::{Element, ElementReader};

/// Narrow-passage radius: a Fußweg between two houses is a 2–6 m gap.
const R_NARROW_M: f64 = 8.0;

/// Built-up-corridor radius: a residential street is 8–20 m building-to-building.
const R_CORRIDOR_M: f64 = 25.0;

/// Grid cell for the building-node index, metres. One cell ≥ `R_CORRIDOR_M` so
/// a query never needs more than the cells its own bbox touches.
const GRID_M: f64 = 32.0;

/// `highway=*` values that carry people on foot as their primary purpose.
const FOOT: &[&str] = &[
    "footway",
    "path",
    "steps",
    "pedestrian",
    "living_street",
    "corridor",
    "track",
    "bridleway",
];

/// `highway=*` values that are bicycle infrastructure in their own right.
const CYCLE: &[&str] = &["cycleway"];

/// Road controls for column 4 — the classes that MUST behave differently at
/// `R_NARROW_M` if the column measures shape rather than density.
const CONTROL: &[&str] = &["residential", "secondary", "primary", "tertiary"];

/// Tag values that mean "this side has none", as opposed to "not surveyed".
///
/// `separate` is deliberately NOT here: it means the sidewalk exists and is
/// mapped as its own way, so it is evidence *for* the double mapping in column
/// 2, not against it.
const ABSENT: &[&str] = &["no", "none"];

/// A local equirectangular frame — metres from an anchor, for proximity only.
struct Frame {
    lat0: f64,
    lon0: f64,
    m_per_deg_lat: f64,
    m_per_deg_lon: f64,
}

impl Frame {
    fn new(lat0: f64, lon0: f64) -> Self {
        let phi = lat0.to_radians();
        Self {
            lat0,
            lon0,
            m_per_deg_lat: meridional_radius(phi) * std::f64::consts::PI / 180.0,
            m_per_deg_lon: normal_radius(phi) * phi.cos() * std::f64::consts::PI / 180.0,
        }
    }

    fn xy(&self, lat: f64, lon: f64) -> (f64, f64) {
        (
            (lon - self.lon0) * self.m_per_deg_lon,
            (lat - self.lat0) * self.m_per_deg_lat,
        )
    }
}

/// Building-node positions, bucketed so a segment query touches a few cells.
#[derive(Default)]
struct Grid {
    cells: HashMap<(i32, i32), Vec<(f32, f32)>>,
}

impl Grid {
    fn key(x: f64, y: f64) -> (i32, i32) {
        ((x / GRID_M).floor() as i32, (y / GRID_M).floor() as i32)
    }

    fn insert(&mut self, x: f64, y: f64) {
        self.cells
            .entry(Self::key(x, y))
            .or_default()
            .push((x as f32, y as f32));
    }

    /// Closest building node on each side of segment `a`→`b`, in metres.
    ///
    /// Returns `(left, right)`; either is `f64::INFINITY` when that side is
    /// empty within `R_CORRIDOR_M`. Side is the sign of the cross product, so a
    /// node on the centreline is on neither side.
    fn flanks(&self, a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let (mut left, mut right) = (f64::INFINITY, f64::INFINITY);

        let (x0, x1) = (a.0.min(b.0) - R_CORRIDOR_M, a.0.max(b.0) + R_CORRIDOR_M);
        let (y0, y1) = (a.1.min(b.1) - R_CORRIDOR_M, a.1.max(b.1) + R_CORRIDOR_M);
        let (cx0, cy0) = Self::key(x0, y0);
        let (cx1, cy1) = Self::key(x1, y1);

        for cx in cx0..=cx1 {
            for cy in cy0..=cy1 {
                let Some(pts) = self.cells.get(&(cx, cy)) else {
                    continue;
                };
                for &(px, py) in pts {
                    let (px, py) = (px as f64, py as f64);
                    // Point-to-SEGMENT distance: project, clamp to the segment.
                    let t = if len2 > 0.0 {
                        (((px - a.0) * dx + (py - a.1) * dy) / len2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
                    let d = ((px - qx).powi(2) + (py - qy).powi(2)).sqrt();
                    if d > R_CORRIDOR_M {
                        continue;
                    }
                    let cross = dx * (py - a.1) - dy * (px - a.0);
                    if cross > 0.0 {
                        left = left.min(d);
                    } else if cross < 0.0 {
                        right = right.min(d);
                    }
                }
            }
        }
        (left, right)
    }
}

/// Per-`highway`-value totals.
#[derive(Default, Clone)]
struct ClassStat {
    ways: u64,
    nodes: u64,
    metres: f64,
    unnamed: u64,
}

/// A way held back for column 4, in projected metres.
struct Candidate {
    class: String,
    pts: Vec<(f64, f64)>,
}

/// Column-4 result for one class group.
#[derive(Default)]
struct Flanked {
    ways: u64,
    narrow_ways: u64,
    corridor_ways: u64,
    metres: f64,
    narrow_m: f64,
    corridor_m: f64,
}

fn pct(n: u64, d: u64) -> f64 {
    if d == 0 {
        0.0
    } else {
        n as f64 * 100.0 / d as f64
    }
}

fn pct_f(n: f64, d: f64) -> f64 {
    if d == 0.0 {
        0.0
    } else {
        n * 100.0 / d
    }
}

/// The value of `key`, if the way carries it.
fn tag<'a>(tags: &[(&'a str, &'a str)], key: &str) -> Option<&'a str> {
    tags.iter().find(|(k, _)| *k == key).map(|(_, v)| *v)
}

/// Whether any key in `keys` is present with a value that is not an absence.
fn present_any(tags: &[(&str, &str)], keys: &[&str]) -> bool {
    keys.iter().any(|k| {
        tag(tags, k).is_some_and(|v| !ABSENT.contains(&v) && v != "separate" && !v.is_empty())
    })
}

/// Whether any key in `keys` carries the cross-reference value `separate`.
fn says_separate(tags: &[(&str, &str)], keys: &[&str]) -> bool {
    keys.iter().any(|k| tag(tags, k) == Some("separate"))
}

const SIDEWALK_KEYS: &[&str] = &[
    "sidewalk",
    "sidewalk:both",
    "sidewalk:left",
    "sidewalk:right",
];

const CYCLEWAY_KEYS: &[&str] = &[
    "cycleway",
    "cycleway:both",
    "cycleway:left",
    "cycleway:right",
];

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: wayclass_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    // ── Pass 1: node coordinates, and the frame anchor. ──
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(8_000_000);
    let (mut sum_lat, mut sum_lon) = (0.0f64, 0.0f64);
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| match el {
            Element::Node(n) => {
                coords.insert(n.id(), (n.lat(), n.lon()));
                sum_lat += n.lat();
                sum_lon += n.lon();
            }
            Element::DenseNode(n) => {
                coords.insert(n.id(), (n.lat(), n.lon()));
                sum_lat += n.lat();
                sum_lon += n.lon();
            }
            _ => {}
        })
        .expect("pass 1");
    let n = coords.len().max(1) as f64;
    let frame = Frame::new(sum_lat / n, sum_lon / n);
    eprintln!(
        "pass 1: {} nodes, frame anchored at {:.4}, {:.4}",
        coords.len(),
        frame.lat0,
        frame.lon0
    );

    // ── Pass 2: ways. ──
    let mut census: HashMap<String, ClassStat> = HashMap::new();
    let mut nonhighway: HashMap<String, ClassStat> = HashMap::new();
    let mut service_kind: HashMap<String, ClassStat> = HashMap::new();

    // Column 2 counters.
    let (mut sep_sidewalk_ways, mut sep_sidewalk_m) = (0u64, 0.0f64);
    let (mut attr_sidewalk_ways, mut attr_sidewalk_m) = (0u64, 0.0f64);
    let mut xref_sidewalk_ways = 0u64;
    let (mut sep_cycle_ways, mut sep_cycle_m) = (0u64, 0.0f64);
    let (mut attr_cycle_ways, mut attr_cycle_m) = (0u64, 0.0f64);
    let mut xref_cycle_ways = 0u64;
    let mut roads_considered = 0u64;

    let mut building_nodes: HashSet<i64> = HashSet::with_capacity(4_000_000);
    let mut candidates: Vec<Candidate> = Vec::with_capacity(200_000);
    // Foot ways that share a node with a building — attached, not merely near.
    let mut foot_way_nodes: Vec<(String, Vec<i64>)> = Vec::with_capacity(200_000);

    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Way(w) = el else { return };
            let tags: Vec<(&str, &str)> = w.tags().collect();
            if tags.is_empty() {
                return;
            }
            let ids: Vec<i64> = w.refs().collect();
            let pts: Vec<(f64, f64)> = ids.iter().filter_map(|i| coords.get(i).copied()).collect();
            let metres = polyline_metres(&pts);
            let named = tag(&tags, "name").is_some();

            let is_building = tags
                .iter()
                .any(|(k, _)| *k == "building" || *k == "building:part");
            if is_building {
                building_nodes.extend(ids.iter().copied());
            }

            let Some(hw) = tag(&tags, "highway") else {
                // (i-b) what the non-highway, non-building ways actually are —
                // P4's unexamined "neither" bucket.
                if !is_building {
                    let key = [
                        "railway", "waterway", "landuse", "natural", "barrier", "leisure",
                        "amenity", "power", "man_made", "boundary",
                    ]
                    .iter()
                    .find(|k| tag(&tags, k).is_some())
                    .map_or("(other)", |k| *k);
                    let e = nonhighway.entry(key.to_string()).or_default();
                    e.ways += 1;
                    e.nodes += ids.len() as u64;
                    e.metres += metres;
                }
                return;
            };

            let e = census.entry(hw.to_string()).or_default();
            e.ways += 1;
            e.nodes += ids.len() as u64;
            e.metres += metres;
            if !named {
                e.unnamed += 1;
            }

            // (iii) Zufahrten: one `highway` value, several different things.
            if hw == "service" {
                let kind = tag(&tags, "service").unwrap_or("(unspecified)");
                let s = service_kind.entry(kind.to_string()).or_default();
                s.ways += 1;
                s.nodes += ids.len() as u64;
                s.metres += metres;
                if !named {
                    s.unnamed += 1;
                }
            }

            // (ii) the two forms of the same physical thing.
            if hw == "footway" && tag(&tags, "footway") == Some("sidewalk")
                || hw == "path" && tag(&tags, "path") == Some("sidewalk")
            {
                sep_sidewalk_ways += 1;
                sep_sidewalk_m += metres;
            }
            if hw == "cycleway" || (hw == "path" && tag(&tags, "bicycle") == Some("designated")) {
                sep_cycle_ways += 1;
                sep_cycle_m += metres;
            }
            // The attribute form only makes sense on a carriageway.
            let is_road = !FOOT.contains(&hw) && !CYCLE.contains(&hw) && hw != "construction";
            if is_road {
                roads_considered += 1;
                if present_any(&tags, SIDEWALK_KEYS) {
                    attr_sidewalk_ways += 1;
                    attr_sidewalk_m += metres;
                }
                if says_separate(&tags, SIDEWALK_KEYS) {
                    xref_sidewalk_ways += 1;
                }
                if present_any(&tags, CYCLEWAY_KEYS) {
                    attr_cycle_ways += 1;
                    attr_cycle_m += metres;
                }
                if says_separate(&tags, CYCLEWAY_KEYS) {
                    xref_cycle_ways += 1;
                }
            }

            // (iv) hold the foot classes and the road controls for the flank test.
            if (FOOT.contains(&hw) || CYCLE.contains(&hw) || CONTROL.contains(&hw))
                && pts.len() == ids.len()
                && pts.len() >= 2
            {
                candidates.push(Candidate {
                    class: hw.to_string(),
                    pts: pts.iter().map(|&(la, lo)| frame.xy(la, lo)).collect(),
                });
                if FOOT.contains(&hw) || CYCLE.contains(&hw) {
                    foot_way_nodes.push((hw.to_string(), ids.clone()));
                }
            }
        })
        .expect("pass 2");

    // ── Build the building-node grid. ──
    let mut grid = Grid::default();
    for id in &building_nodes {
        if let Some(&(lat, lon)) = coords.get(id) {
            let (x, y) = frame.xy(lat, lon);
            grid.insert(x, y);
        }
    }
    eprintln!(
        "pass 2: {} building nodes gridded, {} candidate ways",
        building_nodes.len(),
        candidates.len()
    );

    // ── Column 4: flanking. ──
    let mut flanked: HashMap<String, Flanked> = HashMap::new();
    for c in &candidates {
        let f = flanked.entry(c.class.clone()).or_default();
        f.ways += 1;
        let (mut any_narrow, mut any_corridor) = (false, false);
        for w in c.pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let seg = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            f.metres += seg;
            let (l, r) = grid.flanks(a, b);
            let worst = l.max(r);
            if worst <= R_NARROW_M {
                f.narrow_m += seg;
                any_narrow = true;
            }
            if worst <= R_CORRIDOR_M {
                f.corridor_m += seg;
                any_corridor = true;
            }
        }
        if any_narrow {
            f.narrow_ways += 1;
        }
        if any_corridor {
            f.corridor_ways += 1;
        }
    }

    // Foot ways physically attached to a building (a shared node).
    let mut attached = 0u64;
    for (_, ids) in &foot_way_nodes {
        if ids.iter().any(|i| building_nodes.contains(i)) {
            attached += 1;
        }
    }

    // ── Report. ──
    let mut rows: Vec<(&String, &ClassStat)> = census.iter().collect();
    rows.sort_by(|a, b| {
        b.1.metres
            .partial_cmp(&a.1.metres)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.0.cmp(b.0))
    });
    let total_ways: u64 = census.values().map(|s| s.ways).sum();
    let total_m: f64 = census.values().map(|s| s.metres).sum();

    println!(
        "\n(i) highway=* census — {total_ways} ways, {:.0} km",
        total_m / 1000.0
    );
    println!(
        "{:<20} {:>9} {:>11} {:>12} {:>8} {:>9}",
        "value", "ways", "nodes", "km", "%km", "unnamed"
    );
    for (v, s) in &rows {
        println!(
            "{:<20} {:>9} {:>11} {:>12.1} {:>7.2}% {:>8.1}%",
            v,
            s.ways,
            s.nodes,
            s.metres / 1000.0,
            pct_f(s.metres, total_m),
            pct(s.unnamed, s.ways),
        );
    }

    let foot_m: f64 = rows
        .iter()
        .filter(|(v, _)| FOOT.contains(&v.as_str()) || CYCLE.contains(&v.as_str()))
        .map(|(_, s)| s.metres)
        .sum();
    println!(
        "\n  foot + cycle classes are {:.2}% of highway kilometres — {:.0} km",
        pct_f(foot_m, total_m),
        foot_m / 1000.0
    );

    let mut nh: Vec<(&String, &ClassStat)> = nonhighway.iter().collect();
    nh.sort_by(|a, b| b.1.ways.cmp(&a.1.ways).then_with(|| a.0.cmp(b.0)));
    println!("\n(i-b) P4's \"neither\" bucket — non-highway, non-building ways");
    println!(
        "{:<20} {:>9} {:>11} {:>12}",
        "primary key", "ways", "nodes", "km"
    );
    for (k, s) in nh.iter().take(12) {
        println!(
            "{:<20} {:>9} {:>11} {:>12.1}",
            k,
            s.ways,
            s.nodes,
            s.metres / 1000.0
        );
    }

    println!("\n(ii) the same thing, mapped two ways — {roads_considered} carriageways considered");
    println!("{:<26} {:>10} {:>12}", "", "ways", "km");
    println!(
        "{:<26} {:>10} {:>12.1}",
        "sidewalk, own geometry",
        sep_sidewalk_ways,
        sep_sidewalk_m / 1000.0
    );
    println!(
        "{:<26} {:>10} {:>12.1}",
        "sidewalk, road attribute",
        attr_sidewalk_ways,
        attr_sidewalk_m / 1000.0
    );
    println!(
        "{:<26} {:>10}          — says sidewalk=separate (cross-reference)",
        "", xref_sidewalk_ways
    );
    println!(
        "{:<26} {:>10} {:>12.1}",
        "cycleway, own geometry",
        sep_cycle_ways,
        sep_cycle_m / 1000.0
    );
    println!(
        "{:<26} {:>10} {:>12.1}",
        "cycleway, road attribute",
        attr_cycle_ways,
        attr_cycle_m / 1000.0
    );
    println!(
        "{:<26} {:>10}          — says cycleway=separate (cross-reference)",
        "", xref_cycle_ways
    );

    let mut sk: Vec<(&String, &ClassStat)> = service_kind.iter().collect();
    sk.sort_by(|a, b| b.1.ways.cmp(&a.1.ways).then_with(|| a.0.cmp(b.0)));
    println!("\n(iii) Zufahrten — highway=service by service=*");
    println!(
        "{:<22} {:>9} {:>12} {:>9}",
        "service", "ways", "km", "unnamed"
    );
    for (k, s) in &sk {
        println!(
            "{:<22} {:>9} {:>12.1} {:>8.1}%",
            k,
            s.ways,
            s.metres / 1000.0,
            pct(s.unnamed, s.ways)
        );
    }

    println!(
        "\n(iv) flanked by buildings on BOTH sides — narrow {R_NARROW_M:.0} m / corridor {R_CORRIDOR_M:.0} m"
    );
    println!(
        "{:<18} {:>9} {:>10} {:>10} {:>10} {:>10}",
        "class", "ways", "narrow%", "corr%", "narrow km", "km"
    );
    let mut fl: Vec<(&String, &Flanked)> = flanked.iter().collect();
    fl.sort_by(|a, b| {
        b.1.metres
            .partial_cmp(&a.1.metres)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (k, f) in &fl {
        let tag = if CONTROL.contains(&k.as_str()) {
            "  <- control"
        } else {
            ""
        };
        println!(
            "{:<18} {:>9} {:>9.1}% {:>9.1}% {:>10.1} {:>10.1}{}",
            k,
            f.ways,
            pct(f.narrow_ways, f.ways),
            pct(f.corridor_ways, f.ways),
            f.narrow_m / 1000.0,
            f.metres / 1000.0,
            tag
        );
    }
    println!(
        "\n  foot/cycle ways sharing a node with a building (attached): {attached} of {}",
        foot_way_nodes.len()
    );

    println!("\n(v) survival in the model — unnamed share by class");
    println!("  an unnamed way is READ and BAKED, but street::edge_name sees NAME_NONE,");
    println!("  so it is absent from every street projection.");
    let mut unnamed_foot = (0u64, 0u64);
    let mut unnamed_road = (0u64, 0u64);
    for (v, s) in &rows {
        if FOOT.contains(&v.as_str()) || CYCLE.contains(&v.as_str()) {
            unnamed_foot.0 += s.unnamed;
            unnamed_foot.1 += s.ways;
        } else {
            unnamed_road.0 += s.unnamed;
            unnamed_road.1 += s.ways;
        }
    }
    println!(
        "  foot + cycle   {:>9} of {:>9} unnamed  ({:.1}%)",
        unnamed_foot.0,
        unnamed_foot.1,
        pct(unnamed_foot.0, unnamed_foot.1)
    );
    println!(
        "  everything else{:>9} of {:>9} unnamed  ({:.1}%)",
        unnamed_road.0,
        unnamed_road.1,
        pct(unnamed_road.0, unnamed_road.1)
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grid holding exactly the given points, in metres.
    fn grid_of(pts: &[(f64, f64)]) -> Grid {
        let mut g = Grid::default();
        for &(x, y) in pts {
            g.insert(x, y);
        }
        g
    }

    #[test]
    fn flanking_needs_a_node_on_each_side_and_says_so_both_ways() {
        // The whole column-4 claim in miniature. A segment along +x with one
        // building 3 m to each side is flanked; move both to the SAME side and
        // it must not be — otherwise the column counts "near buildings", which
        // is a density measure and the doc says it is not.
        let seg = ((0.0, 0.0), (20.0, 0.0));
        let both = grid_of(&[(10.0, 3.0), (10.0, -3.0)]);
        let (l, r) = both.flanks(seg.0, seg.1);
        assert!(l.is_finite() && r.is_finite(), "one node per side flanks");
        assert!(l.max(r) <= R_NARROW_M, "3 m each side is a narrow passage");

        let one_side = grid_of(&[(8.0, 3.0), (12.0, 4.0)]);
        let (l, r) = one_side.flanks(seg.0, seg.1);
        assert!(
            l.is_infinite() || r.is_infinite(),
            "two nodes on one side must leave the other side empty"
        );
    }

    #[test]
    fn a_node_on_the_centreline_flanks_neither_side() {
        // Load-bearing for the doc's claim that a footway ATTACHED to a
        // building (a shared node, an entrance link) is not self-flanking: a
        // shared node sits exactly on the way, cross product zero, no side.
        let g = grid_of(&[(10.0, 0.0), (0.0, 0.0)]);
        let (l, r) = g.flanks((0.0, 0.0), (20.0, 0.0));
        assert!(
            l.is_infinite() && r.is_infinite(),
            "a node on the segment belongs to no side"
        );
    }

    #[test]
    fn distance_is_to_the_segment_not_to_its_midpoint() {
        // A 60 m path with houses 4 m away at its FAR END. Point-to-midpoint
        // would score them ~30 m and report the path unflanked at both radii;
        // point-to-segment scores them 4 m. The doc claims the segment form, so
        // a regression to midpoint distance has to fail here.
        let seg = ((0.0, 0.0), (60.0, 0.0));
        let g = grid_of(&[(58.0, 4.0), (58.0, -4.0)]);
        let (l, r) = g.flanks(seg.0, seg.1);
        assert!(
            l.max(r) <= R_NARROW_M,
            "houses 4 m off the far end are 4 m away, not 30 m (got {l}, {r})"
        );
    }

    #[test]
    fn a_node_beyond_the_end_is_measured_from_the_endpoint() {
        // The clamp's other half: past the end, distance grows along the axis
        // rather than staying perpendicular. 30 m beyond a 20 m segment is out
        // of range entirely, so it must not flank at the corridor radius.
        let g = grid_of(&[(50.0, 1.0), (50.0, -1.0)]);
        let (l, r) = g.flanks((0.0, 0.0), (20.0, 0.0));
        assert!(
            l.is_infinite() && r.is_infinite(),
            "30 m past the end is outside {R_CORRIDOR_M} m regardless of offset"
        );
    }

    #[test]
    fn absence_values_are_not_infrastructure_but_separate_is_a_cross_reference() {
        // Column (ii)'s tag rule, and the trap inside it. `sidewalk=no` means
        // there is none. `sidewalk=separate` means there IS one and it is
        // mapped as its own way — so it must NOT be counted as the attribute
        // form (that would double-count against the geometry form), and it must
        // still be visible as the cross-reference that links the two mappings.
        let none: Vec<(&str, &str)> = vec![("highway", "residential"), ("sidewalk", "no")];
        let sep: Vec<(&str, &str)> = vec![("highway", "residential"), ("sidewalk", "separate")];
        let real: Vec<(&str, &str)> = vec![("highway", "residential"), ("sidewalk:left", "yes")];

        assert!(!present_any(&none, SIDEWALK_KEYS), "no is an absence");
        assert!(
            !present_any(&sep, SIDEWALK_KEYS),
            "separate is geometry elsewhere, not an attribute sidewalk"
        );
        assert!(present_any(&real, SIDEWALK_KEYS), "a side key counts");

        assert!(says_separate(&sep, SIDEWALK_KEYS), "the cross-reference");
        assert!(!says_separate(&none, SIDEWALK_KEYS));
        assert!(!says_separate(&real, SIDEWALK_KEYS));
    }
}
