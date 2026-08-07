//! `housenum_probe` — **P11**, the address template: parity, side, turn-around.
//!
//! ```text
//! housenum_probe <input.osm.pbf>
//! ```
//!
//! P10 measured what an address *is*: a street name as text, a house number as
//! text, no reference to anything. This measures whether the numbers along a
//! street follow a **template**, in which case they need not be stored one by
//! one — the same argument `helix`'s curve ruler makes for geometry ("do not
//! compress the points — recognise they lie on a template").
//!
//! Two proposals, measured rather than assumed:
//!
//! 1. **Obscure numbers as a decimal place.** `12a` → `12.01`, `12b` → `12.02`,
//!    so the whole part is the position and the fraction is the sub-position.
//!    P10 measured 79.5 % `Plain` + 19.2 % `Suffixed` in Berlin, so this covers
//!    98.7 % there and 99.4 % in Iceland — the question is only whether the
//!    encoding preserves ORDER, which is the one thing it exists for.
//! 2. **Streets are parity-split, or one-way-up-and-back.** Either odd numbers
//!    run one side and even the other, both ascending together; or the numbers
//!    ascend along one side and descend back along the other, with a
//!    turn-around at the end. If a street is one of those two, the addresses on
//!    it reduce to a range, a parity and a side.
//!
//! # How "along the street" is computed, and where that is wrong
//!
//! Addresses are projected onto the **principal axis** of their street
//! component — the first PCA direction of the addresses themselves. It is
//! cheap, needs no ordered traversal of a possibly-branching component, and is
//! right for a street that runs roughly one way.
//!
//! **It is wrong for a street that doubles back**, an L-shape or a loop: two
//! genuinely ordered houses can project to the same coordinate or invert. Such
//! streets will read as **irregular**, so the error direction is conservative —
//! the template's share is under-stated, never over-stated. A component whose
//! addresses have no dominant direction (a cul-de-sac ring) is reported
//! separately rather than folded into the failures.
//!
//! # Side of the street
//!
//! The sign of the cross product between the nearest segment's direction and
//! the vector to the address. That is exactly "left or right of the way as
//! drawn", and OSM way direction is arbitrary — which does not matter here,
//! because the question is only whether the two sides *differ*, never which is
//! which.

use std::collections::HashMap;

use osm_soa_bake::curve::point_in_ring;
use osm_soa_bake::geodesy::{meridional_radius, normal_radius};
use osmpbf::{Element, ElementReader};

/// A house is within this distance of its own street.
const NEAR_M: f64 = 100.0;

/// Grid cell for the street-segment index, metres.
const CELL_M: f64 = 128.0;

/// Fewest addresses on one side before its ordering means anything.
///
/// Two points are monotone in both directions, so a template fitted to them is
/// a tautology. Three is the smallest number that can fail.
const MIN_SIDE: usize = 3;

/// Share of pairs that must be ordered before a side counts as monotone.
///
/// Not 1.0: one mis-tagged or newly-built house should not condemn a street
/// that is otherwise a clean sequence. Reported alongside the strict figure so
/// the allowance is visible rather than assumed.
const MONOTONE_TOL: f64 = 0.9;

/// Share of one side that must share a parity before the split counts.
const PARITY_TOL: f64 = 0.9;

/// A closed areal ring: what it is, its points, and its bbox.
type Ring = (&'static str, Vec<(f64, f64)>, (f64, f64), (f64, f64));

/// One street segment in the metre frame.
type Seg = ((f64, f64), (f64, f64));

/// How far off the street centreline to probe what occupies the sparse side.
///
/// Far enough to clear the carriageway and the verge, near enough to still be
/// the parcel fronting this street rather than the next block.
const PROBE_OFFSET_M: f64 = 25.0;

/// What a sparse side turns out to hold. A one-sided street is only a finding
/// once this says WHY.
fn areal_kind(tags: &[(&str, &str)]) -> Option<&'static str> {
    let get = |k: &str| tags.iter().find(|(t, _)| *t == k).map(|(_, v)| *v);
    if let Some(l) = get("landuse") {
        return match l {
            "industrial" | "commercial" | "retail" | "port" | "quarry" => Some("industry"),
            "forest" | "forestry" => Some("wood"),
            "meadow" | "grass" | "village_green" | "farmland" | "orchard" | "allotments"
            | "cemetery" | "recreation_ground" => Some("green"),
            "railway" => Some("railway"),
            "reservoir" | "basin" => Some("water"),
            _ => None,
        };
    }
    if let Some(n) = get("natural") {
        return match n {
            "wood" | "scrub" => Some("wood"),
            "water" | "wetland" => Some("water"),
            "heath" | "grassland" | "sand" | "beach" => Some("green"),
            _ => None,
        };
    }
    if let Some(l) = get("leisure") {
        return match l {
            "park" | "garden" | "nature_reserve" | "pitch" | "sports_centre" | "golf_course" => {
                Some("green")
            }
            _ => None,
        };
    }
    if get("railway").is_some() || get("aeroway").is_some() {
        return Some("railway");
    }
    if get("man_made") == Some("works") || get("amenity") == Some("school") {
        return Some("industry");
    }
    None
}

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

struct Dsu {
    parent: Vec<usize>,
}

impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[ra] = rb;
        }
    }
}

/// A house number as a sortable decimal: whole part is the position, fraction
/// the sub-position.
///
/// `12` → 12.0, `12a` → 12.01, `12b` → 12.02. The ONLY property that matters is
/// that it preserves the order a human reads off the doors, which is what the
/// test pins. A range (`12-14`) has no single position and returns `None`
/// rather than a fabricated one — silently taking the lower bound would place
/// two houses on one point, which is the defect the encoding exists to avoid.
fn to_decimal(s: &str) -> Option<f64> {
    let t = s.trim();
    if t.is_empty() {
        return None;
    }
    if t.contains('-') || t.contains('/') || t.contains(',') || t.contains(';') {
        return None;
    }
    let digits: String = t.chars().take_while(char::is_ascii_digit).collect();
    if digits.is_empty() {
        return None;
    }
    let whole: f64 = digits.parse().ok()?;
    let rest = t[digits.len()..].trim();
    if rest.is_empty() {
        return Some(whole);
    }
    // A short alphabetic suffix becomes hundredths, in alphabet order.
    if rest.chars().all(char::is_alphabetic) && rest.chars().count() <= 2 {
        let mut v = 0u32;
        for c in rest.to_ascii_lowercase().chars() {
            let idx = u32::from(c as u8).saturating_sub(u32::from(b'a')) + 1;
            v = v * 26 + idx.min(26);
        }
        return Some(whole + f64::from(v) / 100.0);
    }
    None
}

fn point_seg_t(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
    let d = ((p.0 - qx).powi(2) + (p.1 - qy).powi(2)).sqrt();
    // Side: sign of the cross product of the segment direction and (p - a).
    let cross = dx * (p.1 - a.1) - dy * (p.0 - a.0);
    (d, cross)
}

fn pct(n: u64, d: u64) -> f64 {
    if d == 0 {
        0.0
    } else {
        n as f64 * 100.0 / d as f64
    }
}

/// Share of ordered pairs that agree with `ascending`, over points already
/// sorted by their along-street coordinate.
fn monotone_share(nums: &[f64], ascending: bool) -> f64 {
    if nums.len() < 2 {
        return 1.0;
    }
    let mut ok = 0usize;
    let mut total = 0usize;
    for w in nums.windows(2) {
        total += 1;
        let d = w[1] - w[0];
        if (ascending && d >= 0.0) || (!ascending && d <= 0.0) {
            ok += 1;
        }
    }
    ok as f64 / total as f64
}

/// One side's verdict: monotone, and in which direction.
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
enum Side {
    Up,
    Down,
    Irregular,
    TooFew,
}

fn side_verdict(mut pts: Vec<(f64, f64)>) -> Side {
    if pts.len() < MIN_SIDE {
        return Side::TooFew;
    }
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let nums: Vec<f64> = pts.iter().map(|p| p.1).collect();
    let up = monotone_share(&nums, true);
    let down = monotone_share(&nums, false);
    if up >= MONOTONE_TOL && up >= down {
        Side::Up
    } else if down >= MONOTONE_TOL {
        Side::Down
    } else {
        Side::Irregular
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: housenum_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(8_000_000);
    let (mut slat, mut slon) = (0.0f64, 0.0f64);
    let mut node_addrs: Vec<(i64, String, String)> = Vec::new();
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let (id, lat, lon, tags): (i64, f64, f64, Vec<(&str, &str)>) = match el {
                Element::Node(n) => (n.id(), n.lat(), n.lon(), n.tags().collect()),
                Element::DenseNode(n) => (n.id(), n.lat(), n.lon(), n.tags().collect()),
                _ => return,
            };
            coords.insert(id, (lat, lon));
            slat += lat;
            slon += lon;
            let get = |k: &str| tags.iter().find(|(t, _)| *t == k).map(|(_, v)| *v);
            if let (Some(num), Some(st)) = (get("addr:housenumber"), get("addr:street")) {
                node_addrs.push((id, st.to_string(), num.to_string()));
            }
        })
        .expect("pass 1");
    let nn = coords.len().max(1) as f64;
    let frame = Frame::new(slat / nn, slon / nn);

    let mut street_names: Vec<String> = Vec::new();
    let mut street_nodes: Vec<Vec<i64>> = Vec::new();
    let mut addrs: Vec<((f64, f64), String, String)> = Vec::new();
    // Closed areal rings, for explaining a sparse side.
    let mut rings: Vec<Ring> = Vec::new();
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Way(w) = el else { return };
            let tags: Vec<(&str, &str)> = w.tags().collect();
            if tags.is_empty() {
                return;
            }
            let get = |k: &str| tags.iter().find(|(t, _)| *t == k).map(|(_, v)| *v);
            let ids: Vec<i64> = w.refs().collect();
            if get("highway").is_some() {
                if let Some(n) = get("name") {
                    street_names.push(n.to_string());
                    street_nodes.push(ids.clone());
                }
            }
            if let Some(kind) = areal_kind(&tags) {
                if ids.len() >= 4 && ids.first() == ids.last() {
                    let pts: Vec<(f64, f64)> = ids
                        .iter()
                        .filter_map(|i| coords.get(i).copied())
                        .map(|(la, lo)| frame.xy(la, lo))
                        .collect();
                    if pts.len() == ids.len() {
                        let mut lo = (f64::MAX, f64::MAX);
                        let mut hi = (f64::MIN, f64::MIN);
                        for &(x, y) in &pts {
                            lo = (lo.0.min(x), lo.1.min(y));
                            hi = (hi.0.max(x), hi.1.max(y));
                        }
                        rings.push((kind, pts, lo, hi));
                    }
                }
            }
            if let (Some(num), Some(st)) = (get("addr:housenumber"), get("addr:street")) {
                let pts: Vec<(f64, f64)> =
                    ids.iter().filter_map(|i| coords.get(i).copied()).collect();
                if pts.is_empty() {
                    return;
                }
                let (mut sx, mut sy) = (0.0, 0.0);
                for &(la, lo) in &pts {
                    let (x, y) = frame.xy(la, lo);
                    sx += x;
                    sy += y;
                }
                let k = pts.len() as f64;
                addrs.push(((sx / k, sy / k), st.to_string(), num.to_string()));
            }
        })
        .expect("pass 2");
    for (id, st, num) in node_addrs {
        if let Some(&(la, lo)) = coords.get(&id) {
            addrs.push((frame.xy(la, lo), st, num));
        }
    }

    // Same-named ways -> connected components (a name is not a road: P8).
    let mut by_name: HashMap<&str, Vec<usize>> = HashMap::new();
    for (i, n) in street_names.iter().enumerate() {
        by_name.entry(n.as_str()).or_default().push(i);
    }
    let mut comp_of: Vec<usize> = vec![0; street_names.len()];
    let mut next_comp = 0usize;
    for members in by_name.values() {
        let mut dsu = Dsu::new(members.len());
        let mut owner: HashMap<i64, usize> = HashMap::new();
        for (li, &wi) in members.iter().enumerate() {
            for &n in &street_nodes[wi] {
                if let Some(&prev) = owner.get(&n) {
                    dsu.union(prev, li);
                } else {
                    owner.insert(n, li);
                }
            }
        }
        let mut local: HashMap<usize, usize> = HashMap::new();
        for (li, &wi) in members.iter().enumerate() {
            let root = dsu.find(li);
            let c = *local.entry(root).or_insert_with(|| {
                let c = next_comp;
                next_comp += 1;
                c
            });
            comp_of[wi] = c;
        }
    }

    let mut grid: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (wi, nodes) in street_nodes.iter().enumerate() {
        for si in 0..nodes.len().saturating_sub(1) {
            let (Some(&a), Some(&b)) = (coords.get(&nodes[si]), coords.get(&nodes[si + 1])) else {
                continue;
            };
            let p = frame.xy(a.0, a.1);
            let q = frame.xy(b.0, b.1);
            let cx0 = ((p.0.min(q.0) - NEAR_M) / CELL_M).floor() as i32;
            let cx1 = ((p.0.max(q.0) + NEAR_M) / CELL_M).floor() as i32;
            let cy0 = ((p.1.min(q.1) - NEAR_M) / CELL_M).floor() as i32;
            let cy1 = ((p.1.max(q.1) + NEAR_M) / CELL_M).floor() as i32;
            for cx in cx0..=cx1 {
                for cy in cy0..=cy1 {
                    grid.entry((cx, cy)).or_default().push((wi, si));
                }
            }
        }
    }

    // Ring index, and the street segments of each component — the probe point
    // is offset from the CENTRELINE, not derived from the addresses, or it
    // would only ever find what is already next to a house.
    let mut ring_grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (ri, (_, _, lo, hi)) in rings.iter().enumerate() {
        let cx0 = (lo.0 / CELL_M).floor() as i32;
        let cx1 = (hi.0 / CELL_M).floor() as i32;
        let cy0 = (lo.1 / CELL_M).floor() as i32;
        let cy1 = (hi.1 / CELL_M).floor() as i32;
        for cx in cx0..=cx1 {
            for cy in cy0..=cy1 {
                ring_grid.entry((cx, cy)).or_default().push(ri);
            }
        }
    }
    let mut comp_segs: HashMap<usize, Vec<Seg>> = HashMap::new();
    for (wi, nodes) in street_nodes.iter().enumerate() {
        for si in 0..nodes.len().saturating_sub(1) {
            let (Some(&a), Some(&b)) = (coords.get(&nodes[si]), coords.get(&nodes[si + 1])) else {
                continue;
            };
            comp_segs
                .entry(comp_of[wi])
                .or_default()
                .push((frame.xy(a.0, a.1), frame.xy(b.0, b.1)));
        }
    }

    // Assign each address to (component, side) with its decimal number.
    // (position, side, decimal number) for one address.
    type Placed = ((f64, f64), i8, f64);
    let mut per_comp: HashMap<usize, Vec<Placed>> = HashMap::new();
    let (mut decimal_ok, mut decimal_no) = (0u64, 0u64);
    for (at, name, num) in &addrs {
        let Some(dec) = to_decimal(num) else {
            decimal_no += 1;
            continue;
        };
        decimal_ok += 1;
        let cx = (at.0 / CELL_M).floor() as i32;
        let cy = (at.1 / CELL_M).floor() as i32;
        let mut best = (f64::INFINITY, 0usize, 0.0f64);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(b) = grid.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &(wi, si) in b {
                    if street_names[wi] != *name {
                        continue;
                    }
                    let (Some(&p), Some(&q)) = (
                        coords.get(&street_nodes[wi][si]),
                        coords.get(&street_nodes[wi][si + 1]),
                    ) else {
                        continue;
                    };
                    let (d, cross) = point_seg_t(*at, frame.xy(p.0, p.1), frame.xy(q.0, q.1));
                    if d < best.0 {
                        best = (d, comp_of[wi], cross);
                    }
                }
            }
        }
        if best.0 > NEAR_M {
            continue;
        }
        let side = if best.2 >= 0.0 { 1i8 } else { -1i8 };
        per_comp.entry(best.1).or_default().push((*at, side, dec));
    }

    // Classify each component.
    let (mut parallel, mut horseshoe, mut one_sided, mut irregular, mut too_few, mut no_axis) =
        (0u64, 0u64, 0u64, 0u64, 0u64, 0u64);
    // A street with an industrial complex, a park or a railway on one side has
    // ONE numbered side and a near-empty other. That is a one-sided template,
    // not a failure — the first version's match arm folded it in with "the
    // other side is scrambled", which are opposite verdicts. Split, and the
    // sparse side's size is reported so the reader can see which case it is.
    let mut half_irregular = 0u64;
    let mut sparse_side_size: HashMap<usize, u64> = HashMap::new();
    let (mut parity_split, mut parity_checked) = (0u64, 0u64);
    let mut explain: Vec<(usize, i8)> = Vec::new();
    for (&comp, members) in &per_comp {
        if members.len() < MIN_SIDE * 2 {
            too_few += 1;
            continue;
        }
        // Principal axis of the addresses themselves.
        let n = members.len() as f64;
        let (mut mx, mut my) = (0.0, 0.0);
        for (p, _, _) in members {
            mx += p.0;
            my += p.1;
        }
        let (mx, my) = (mx / n, my / n);
        let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
        for (p, _, _) in members {
            let (u, v) = (p.0 - mx, p.1 - my);
            sxx += u * u;
            sxy += u * v;
            syy += v * v;
        }
        // Dominant eigenvector of the 2x2 covariance.
        let tr = sxx + syy;
        let det = sxx * syy - sxy * sxy;
        let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
        let l1 = tr / 2.0 + disc;
        let l2 = tr / 2.0 - disc;
        if l1 <= 0.0 || l2 / l1 > 0.5 {
            // No dominant direction: a ring or a blob, where projecting onto an
            // axis orders nothing. Reported separately rather than counted as a
            // template failure it never had a chance at.
            no_axis += 1;
            continue;
        }
        let axis = if sxy.abs() > 1e-9 {
            let v = (l1 - syy, sxy);
            let l = (v.0 * v.0 + v.1 * v.1).sqrt();
            (v.0 / l, v.1 / l)
        } else if sxx >= syy {
            (1.0, 0.0)
        } else {
            (0.0, 1.0)
        };

        let mut left: Vec<(f64, f64)> = Vec::new();
        let mut right: Vec<(f64, f64)> = Vec::new();
        for (p, side, dec) in members {
            let along = (p.0 - mx) * axis.0 + (p.1 - my) * axis.1;
            if *side >= 0 {
                left.push((along, *dec));
            } else {
                right.push((along, *dec));
            }
        }

        // Parity: does each side hold mostly one parity, and do the two differ?
        if left.len() >= MIN_SIDE && right.len() >= MIN_SIDE {
            parity_checked += 1;
            let even = |v: &Vec<(f64, f64)>| {
                v.iter().filter(|(_, d)| (*d as i64) % 2 == 0).count() as f64 / v.len() as f64
            };
            let (el, er) = (even(&left), even(&right));
            let pure_l = el >= PARITY_TOL || el <= 1.0 - PARITY_TOL;
            let pure_r = er >= PARITY_TOL || er <= 1.0 - PARITY_TOL;
            if pure_l && pure_r && (el >= PARITY_TOL) != (er >= PARITY_TOL) {
                parity_split += 1;
            }
        }

        let (nl, nr) = (left.len(), right.len());
        match (side_verdict(left), side_verdict(right)) {
            (Side::Up, Side::Up) | (Side::Down, Side::Down) => parallel += 1,
            (Side::Up, Side::Down) | (Side::Down, Side::Up) => horseshoe += 1,
            // Ordered on one side, too few to judge on the other: the
            // industrial-complex case. A template with one side.
            (Side::Up | Side::Down, Side::TooFew) => {
                one_sided += 1;
                *sparse_side_size.entry(nr).or_insert(0) += 1;
                explain.push((comp, -1i8));
            }
            (Side::TooFew, Side::Up | Side::Down) => {
                one_sided += 1;
                *sparse_side_size.entry(nl).or_insert(0) += 1;
                explain.push((comp, 1i8));
            }
            // Ordered on one side and genuinely SCRAMBLED on the other. This is
            // the partial failure the previous version conflated with the above.
            (Side::Up | Side::Down, Side::Irregular) | (Side::Irregular, Side::Up | Side::Down) => {
                half_irregular += 1;
            }
            (Side::TooFew, Side::TooFew) => too_few += 1,
            _ => irregular += 1,
        }
    }

    let total_comp =
        parallel + horseshoe + one_sided + half_irregular + irregular + too_few + no_axis;
    println!("\n(1) the decimal encoding — does an obscure number get a position?");
    println!(
        "  representable as a decimal {decimal_ok:>9}  ({:.2}%)",
        pct(decimal_ok, decimal_ok + decimal_no)
    );
    println!(
        "  NOT (a range, or unparsable) {decimal_no:>7}  ({:.2}%)",
        pct(decimal_no, decimal_ok + decimal_no)
    );
    println!("  a range gets NO position rather than a fabricated one.");

    println!("\n(2) is the street a template? — components with addresses on both sides");
    println!(
        "  parallel  (both sides ascend together) {parallel:>8}  ({:.1}%)",
        pct(parallel, total_comp)
    );
    println!(
        "  horseshoe (up one side, back the other) {horseshoe:>7}  ({:.1}%)",
        pct(horseshoe, total_comp)
    );
    println!(
        "  one-sided template (other side sparse) {one_sided:>8}  ({:.1}%)",
        pct(one_sided, total_comp)
    );
    println!(
        "  half irregular (other side SCRAMBLED)  {half_irregular:>8}  ({:.1}%)",
        pct(half_irregular, total_comp)
    );
    println!(
        "  irregular                              {irregular:>8}  ({:.1}%)",
        pct(irregular, total_comp)
    );
    println!(
        "  too few addresses to judge             {too_few:>8}  ({:.1}%)",
        pct(too_few, total_comp)
    );
    println!(
        "  no dominant axis (ring/blob)           {no_axis:>8}  ({:.1}%)",
        pct(no_axis, total_comp)
    );
    println!("  total components with addresses        {total_comp:>8}");
    let mut ss: Vec<(&usize, &u64)> = sparse_side_size.iter().collect();
    ss.sort_by_key(|(k, _)| **k);
    let sparse_desc: String = ss
        .iter()
        .take(3)
        .map(|(k, v)| format!("{k} addr: {v}"))
        .collect::<Vec<_>>()
        .join(", ");
    println!("    the sparse side holds — {sparse_desc}");
    let judgeable = parallel + horseshoe + one_sided + half_irregular + irregular;
    println!(
        "\n  TEMPLATE-SHAPED, two-sided only  {:.1}% of judgeable",
        pct(parallel + horseshoe, judgeable)
    );
    println!(
        "  TEMPLATE-SHAPED incl. one-sided  {:.1}% of judgeable  <- the honest figure",
        pct(parallel + horseshoe + one_sided, judgeable)
    );
    println!("  (a street with an industrial complex on one side has ONE numbered side;");
    println!("   counting that as a failure was an error in the first version of this probe.)");

    // ── Why is the sparse side sparse? ──
    let mut why: HashMap<&'static str, u64> = HashMap::new();
    let mut explained = 0u64;
    for (comp, sparse_sign) in &explain {
        let Some(segs) = comp_segs.get(comp) else {
            *why.entry("(no street geometry)").or_insert(0) += 1;
            continue;
        };
        let mut hits: HashMap<&'static str, u64> = HashMap::new();
        for (a, b) in segs.iter().take(40) {
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l = (dx * dx + dy * dy).sqrt();
            if l <= 0.0 {
                continue;
            }
            // Unit normal, flipped to the sparse side. The side convention is
            // the same cross product the addresses were classified with, so
            // "sparse" here means the same thing it meant there.
            let n = (
                -dy / l * f64::from(*sparse_sign),
                dx / l * f64::from(*sparse_sign),
            );
            let mid = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
            let p = (mid.0 + n.0 * PROBE_OFFSET_M, mid.1 + n.1 * PROBE_OFFSET_M);
            let key = ((p.0 / CELL_M).floor() as i32, (p.1 / CELL_M).floor() as i32);
            let Some(cands) = ring_grid.get(&key) else {
                continue;
            };
            for &ri in cands {
                let (kind, ring, lo, hi) = &rings[ri];
                if p.0 < lo.0 || p.0 > hi.0 || p.1 < lo.1 || p.1 > hi.1 {
                    continue;
                }
                if point_in_ring(p, ring) {
                    *hits.entry(kind).or_insert(0) += 1;
                }
            }
        }
        if let Some((k, _)) = hits.iter().max_by_key(|(_, v)| **v) {
            *why.entry(k).or_insert(0) += 1;
            explained += 1;
        } else {
            *why.entry("nothing mapped there").or_insert(0) += 1;
        }
    }
    println!(
        "\n(3) WHY is the sparse side sparse? — probing {PROBE_OFFSET_M:.0} m off the centreline"
    );
    let mut ws: Vec<(&&str, &u64)> = why.iter().collect();
    ws.sort_by(|a, b| b.1.cmp(a.1));
    for (k, v) in ws {
        println!("  {k:<22} {v:>7}  ({:.1}%)", pct(*v, one_sided));
    }
    println!(
        "  explained by something mapped: {explained} of {one_sided}  ({:.1}%)",
        pct(explained, one_sided)
    );

    println!("\n(4) parity — odd one side, even the other");
    println!(
        "  clean split {parity_split:>8} of {parity_checked} checked  ({:.1}%)",
        pct(parity_split, parity_checked)
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sparse_other_side_is_a_one_sided_template_not_a_failure() {
        // The industrial-complex case, and the defect it exposed. A street with
        // one numbered side and a factory, park or railway opposite has an
        // ORDERED side and a near-empty one. The first version of the classifier
        // put that in the same bucket as "the other side is scrambled", which is
        // the opposite verdict, and under-stated the template share.
        //
        // Here the distinction is made at the level the classifier uses: an
        // empty or one-address side is TooFew, a scrambled one is Irregular, and
        // those must not be the same value.
        let ordered = vec![(0.0, 1.0), (1.0, 3.0), (2.0, 5.0), (3.0, 7.0)];
        assert_eq!(side_verdict(ordered.clone()), Side::Up);

        // The factory side: nothing, or a single address for the whole complex.
        assert_eq!(side_verdict(vec![]), Side::TooFew);
        assert_eq!(side_verdict(vec![(1.5, 40.0)]), Side::TooFew);
        assert_eq!(side_verdict(vec![(1.0, 40.0), (2.0, 42.0)]), Side::TooFew);

        // …and a genuinely scrambled other side is a DIFFERENT verdict, or the
        // split the report now makes would be a distinction without a
        // difference.
        let scrambled = vec![(0.0, 5.0), (1.0, 1.0), (2.0, 9.0), (3.0, 3.0)];
        assert_ne!(side_verdict(scrambled.clone()), Side::TooFew);
        assert_eq!(side_verdict(scrambled), Side::Irregular);
    }

    #[test]
    fn the_decimal_preserves_the_order_a_human_reads_off_the_doors() {
        // The only property the encoding exists for. If 12a did not sort between
        // 12 and 13, the whole scheme would be a renaming rather than an
        // ordering, and every template verdict built on it would be noise.
        let v = |s: &str| to_decimal(s).unwrap();
        assert!(v("12") < v("12a"));
        assert!(v("12a") < v("12b"));
        assert!(v("12b") < v("13"));
        assert!(v("9") < v("10"), "and it is numeric, not lexical");
        assert_eq!(v("12"), 12.0);
        // Case must not change the position.
        assert_eq!(v("12A"), v("12a"));
    }

    #[test]
    fn a_range_gets_no_position_rather_than_a_fabricated_one() {
        // Taking the lower bound of "12-14" would put two houses on one point —
        // the exact defect the encoding is meant to remove. Better to refuse.
        assert!(to_decimal("12-14").is_none());
        assert!(to_decimal("12/14").is_none());
        assert!(to_decimal("12,14").is_none());
        assert!(to_decimal("").is_none());
        assert!(to_decimal("no-number").is_none());
        // …while the ordinary cases still work, or the refusal would be a
        // blanket one and the coverage figure meaningless.
        assert!(to_decimal("7").is_some());
        assert!(to_decimal("7c").is_some());
    }

    #[test]
    fn a_side_is_monotone_up_down_or_neither_and_two_points_are_never_evidence() {
        // Two points are monotone in BOTH directions, so a template fitted to
        // them is a tautology — MIN_SIDE exists for that, and it must show.
        assert_eq!(side_verdict(vec![(0.0, 1.0), (1.0, 3.0)]), Side::TooFew);
        assert_eq!(
            side_verdict(vec![(0.0, 1.0), (1.0, 3.0), (2.0, 5.0)]),
            Side::Up
        );
        assert_eq!(
            side_verdict(vec![(0.0, 5.0), (1.0, 3.0), (2.0, 1.0)]),
            Side::Down
        );
        // Genuinely scrambled: neither direction reaches the tolerance.
        assert_eq!(
            side_verdict(vec![(0.0, 5.0), (1.0, 1.0), (2.0, 9.0), (3.0, 3.0)]),
            Side::Irregular
        );
    }

    #[test]
    fn the_verdict_sorts_by_position_rather_than_trusting_input_order() {
        // Addresses arrive in file order, not along the street. A version that
        // skipped the sort would read a perfectly ordered street as irregular
        // and under-state the template everywhere.
        let scrambled = vec![(2.0, 5.0), (0.0, 1.0), (1.0, 3.0)];
        assert_eq!(side_verdict(scrambled), Side::Up);
    }

    #[test]
    fn one_outlier_does_not_condemn_an_otherwise_clean_street() {
        // MONOTONE_TOL made visible: a single infill house numbered out of
        // sequence is normal, and a strict rule would classify most real
        // streets as irregular.
        //
        // The arithmetic matters and I had it wrong: N points give N-1 WINDOWS,
        // so ten points with one inversion is 8/9 = 88.9 %, BELOW the bar.
        // Eleven points give ten windows, so one inversion is exactly the 90 %
        // the constant allows.
        let mut v: Vec<(f64, f64)> = (0..11)
            .map(|i| (f64::from(i), f64::from(i) * 2.0))
            .collect();
        v[10] = (10.0, 1.0);
        assert_eq!(side_verdict(v), Side::Up);
        // …but two inversions in the same eleven is 8/10 = 80 % and must fail,
        // or the allowance would be unbounded.
        let mut w: Vec<(f64, f64)> = (0..11)
            .map(|i| (f64::from(i), f64::from(i) * 2.0))
            .collect();
        w[5] = (5.0, 0.0);
        w[10] = (10.0, 1.0);
        assert_eq!(side_verdict(w), Side::Irregular);
    }
}
