//! `addr_probe` — **P10**, addresses: the reference that is a string.
//!
//! ```text
//! addr_probe <input.osm.pbf>
//! ```
//!
//! An OSM address does not point at a street. It carries `addr:street` as
//! **text** and `addr:housenumber` as **text**, and nothing links either to a
//! way id. Resolving "Rosenweg 12" therefore means matching a string against
//! nearby street names — and P8 measured that "Rosenweg" is **83 separate
//! streets** in Berlin, with a quarter of Berlin's named groups and half of
//! Iceland's not being one connected road at all.
//!
//! So the address layer resolves by **proximity to a name**, which is exactly
//! the join the routing gate forbids for connectivity. The difference is that
//! here there is no alternative in the data: the identity simply is not
//! recorded. This probe measures how often that bites.
//!
//! Four columns:
//!
//! 1. **Where addresses live** — on a building way, on a standalone node, or as
//!    an `addr:interpolation` range. Three encodings of one concept, and a
//!    consumer that knows one of them silently misses the others.
//! 2. **Does the street name resolve?** For each address with `addr:street`,
//!    is there a way of that exact name nearby — and is there exactly **one**
//!    connected road of that name nearby, or several? The second question is
//!    the one that decides whether proximity is doing real work or guessing.
//! 3. **House numbers are not numbers.** `12a`, `12-14`, `1/3`, `12 1/2`.
//!    Anything that sorts or ranges numerically has to prove it first.
//! 4. **Codebook pressure.** `tags.rs` records that the value codebook's growth
//!    driver is distinct `name` / `addr:housenumber` text, which grows with
//!    coverage instead of saturating, against a `u24` ceiling of 16,777,215.
//!    This measures the address layer's share of that.
//!
//! # What this probe does NOT claim
//!
//! It does not resolve addresses correctly — it measures how often a correct
//! resolution is *underdetermined by the data*. An address whose name matches
//! two disconnected roads within the radius may still be resolvable by
//! `addr:postcode`, `addr:suburb`, or by which side of which segment the
//! building sits on. Those are heuristics with their own error rates; the point
//! of the column is that a heuristic is **required**, not that it always fails.

use std::collections::{HashMap, HashSet};

use osm_soa_bake::geodesy::{meridional_radius, normal_radius};
use osmpbf::{Element, ElementReader};

/// How far a building may sit from its own street before the match is doubtful.
///
/// A house fronts its street; 100 m is generous for a driveway or a set-back
/// block, and tight enough that a same-named road in the next district does not
/// silently qualify. Reported alongside a 30 m figure so the choice is visible.
const NEAR_M: f64 = 100.0;
const TIGHT_M: f64 = 30.0;

/// Grid cell for the street-segment index, metres.
const CELL_M: f64 = 128.0;

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

/// Union-find, for grouping same-named ways into connected roads.
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

/// How a house number is shaped, which decides whether it can be ordered.
#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
enum NumKind {
    /// `12` — orders and ranges numerically.
    Plain,
    /// `12a`, `12 B` — orders only if the letter is treated as a minor key.
    Suffixed,
    /// `12-14`, `12/14`, `12,14` — a RANGE or a list, not a position.
    Range,
    /// `12 1/2`, `1½`, and anything else that survives none of the above.
    Other,
}

fn classify_number(s: &str) -> NumKind {
    let t = s.trim();
    if t.is_empty() {
        return NumKind::Other;
    }
    if t.chars().all(|c| c.is_ascii_digit()) {
        return NumKind::Plain;
    }
    // A separator anywhere means several numbers, not one position.
    if t.contains('-') || t.contains('/') || t.contains(',') || t.contains(';') {
        return NumKind::Range;
    }
    // digits then a short alphabetic suffix.
    let digits: String = t.chars().take_while(char::is_ascii_digit).collect();
    if !digits.is_empty() {
        let rest = t[digits.len()..].trim();
        if !rest.is_empty() && rest.chars().all(|c| c.is_alphabetic()) && rest.chars().count() <= 2
        {
            return NumKind::Suffixed;
        }
    }
    NumKind::Other
}

/// Distance from a point to a segment, metres.
fn point_seg(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
    ((p.0 - qx).powi(2) + (p.1 - qy).powi(2)).sqrt()
}

fn pct(n: u64, d: u64) -> f64 {
    if d == 0 {
        0.0
    } else {
        n as f64 * 100.0 / d as f64
    }
}

/// One address, already projected.
struct Addr {
    at: (f64, f64),
    street: Option<String>,
    number: String,
    /// The proposed disambiguator: (street, postcode) where street alone is not
    /// unique. Measured rather than assumed to work.
    postcode: Option<String>,
    /// Which of the three encodings carried it.
    form: &'static str,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: addr_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    // ── Pass 1: coordinates, frame anchor, and address NODES. ──
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(8_000_000);
    let (mut slat, mut slon) = (0.0f64, 0.0f64);
    type NodeAddr = (i64, Option<String>, String, Option<String>);
    let mut node_addrs: Vec<NodeAddr> = Vec::new();
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
            if let Some(num) = get("addr:housenumber") {
                node_addrs.push((
                    id,
                    get("addr:street").map(std::string::ToString::to_string),
                    num.to_string(),
                    get("addr:postcode").map(std::string::ToString::to_string),
                ));
            }
        })
        .expect("pass 1");
    let nn = coords.len().max(1) as f64;
    let frame = Frame::new(slat / nn, slon / nn);

    // ── Pass 2: ways — named streets, address ways, interpolation ways. ──
    let mut street_names: Vec<String> = Vec::new();
    let mut street_nodes: Vec<Vec<i64>> = Vec::new();
    let mut addrs: Vec<Addr> = Vec::new();
    let mut interpolation = 0u64;
    let mut postal_boundaries = 0u64;

    let mut distinct_numbers: HashSet<String> = HashSet::new();
    let mut distinct_streets: HashSet<String> = HashSet::new();

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
            if get("addr:interpolation").is_some() {
                interpolation += 1;
            }
            if get("boundary") == Some("postal_code") {
                postal_boundaries += 1;
            }
            let _ = &postal_boundaries;
            if let Some(num) = get("addr:housenumber") {
                // Centroid of the way, which is what a consumer would place.
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
                addrs.push(Addr {
                    at: (sx / k, sy / k),
                    street: get("addr:street").map(std::string::ToString::to_string),
                    number: num.to_string(),
                    postcode: get("addr:postcode").map(std::string::ToString::to_string),
                    form: if get("building").is_some() {
                        "building way"
                    } else {
                        "other way"
                    },
                });
            }
        })
        .expect("pass 2");

    for (id, street, number, postcode) in node_addrs {
        if let Some(&(la, lo)) = coords.get(&id) {
            addrs.push(Addr {
                at: frame.xy(la, lo),
                street,
                number,
                postcode,
                form: "node",
            });
        }
    }
    for a in &addrs {
        distinct_numbers.insert(a.number.clone());
        if let Some(s) = &a.street {
            distinct_streets.insert(s.clone());
        }
    }
    eprintln!(
        "{} addresses, {} named street ways",
        addrs.len(),
        street_names.len()
    );

    // ── Group same-named street ways into CONNECTED roads. ──
    //
    // A name is not a road (P8). So the question an address faces is not "is
    // there a way of this name nearby" but "is there exactly ONE connected road
    // of this name nearby".
    let mut by_name: HashMap<&str, Vec<usize>> = HashMap::new();
    for (i, n) in street_names.iter().enumerate() {
        by_name.entry(n.as_str()).or_default().push(i);
    }
    // component id per street way, unique across names.
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

    // ── Spatial index over street segments, keyed by cell. ──
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

    // ── Resolve each address by name + proximity. ──
    let (mut no_street_tag, mut name_absent, mut resolved_one, mut ambiguous, mut too_far) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut ambiguous_tight = 0u64;
    // Postcode columns. `comp_pc` is built from the UNAMBIGUOUS addresses only,
    // so it is not circular: a component's postcodes come from houses that
    // already resolved to it by name and proximity alone.
    let mut comp_pc: HashMap<usize, HashSet<String>> = HashMap::new();
    let mut ambiguous_cases: Vec<(Option<String>, Vec<usize>)> = Vec::new();
    let mut with_postcode = 0u64;
    let mut worst = (0usize, String::new());
    for a in &addrs {
        let Some(name) = &a.street else {
            no_street_tag += 1;
            continue;
        };
        if !by_name.contains_key(name.as_str()) {
            name_absent += 1;
            continue;
        }
        let cx = (a.at.0 / CELL_M).floor() as i32;
        let cy = (a.at.1 / CELL_M).floor() as i32;
        let mut near: HashSet<usize> = HashSet::new();
        let mut tight: HashSet<usize> = HashSet::new();
        let mut best = f64::INFINITY;
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
                    let d = point_seg(a.at, frame.xy(p.0, p.1), frame.xy(q.0, q.1));
                    best = best.min(d);
                    if d <= NEAR_M {
                        near.insert(comp_of[wi]);
                    }
                    if d <= TIGHT_M {
                        tight.insert(comp_of[wi]);
                    }
                }
            }
        }
        if near.is_empty() {
            too_far += 1;
            continue;
        }
        if a.postcode.is_some() {
            with_postcode += 1;
        }
        if near.len() == 1 {
            resolved_one += 1;
            if let Some(pc) = &a.postcode {
                comp_pc
                    .entry(*near.iter().next().unwrap())
                    .or_default()
                    .insert(pc.clone());
            }
        } else {
            ambiguous += 1;
            ambiguous_cases.push((a.postcode.clone(), near.iter().copied().collect()));
            if near.len() > worst.0 {
                worst = (near.len(), name.clone());
            }
        }
        if tight.len() > 1 {
            ambiguous_tight += 1;
        }
    }

    // ── Report. ──
    let total = addrs.len() as u64;
    let mut forms: HashMap<&str, u64> = HashMap::new();
    for a in &addrs {
        *forms.entry(a.form).or_insert(0) += 1;
    }
    println!("\n(1) where an address lives — three encodings of one concept");
    let mut fs: Vec<(&&str, &u64)> = forms.iter().collect();
    fs.sort_by(|a, b| b.1.cmp(a.1));
    for (f, c) in fs {
        println!("  {f:<16} {c:>9}  ({:.1}%)", pct(*c, total));
    }
    println!("  addr:interpolation ways {interpolation:>9}  (a RANGE, not a position)");
    println!("  total addresses         {total:>9}");

    println!("\n(2) does the street NAME resolve to one road?");
    println!(
        "  no addr:street at all      {no_street_tag:>9}  ({:.1}%)",
        pct(no_street_tag, total)
    );
    println!(
        "  name matches no way at all {name_absent:>9}  ({:.1}%)",
        pct(name_absent, total)
    );
    println!(
        "  name exists, none within {NEAR_M:.0} m {too_far:>6}  ({:.1}%)",
        pct(too_far, total)
    );
    println!(
        "  exactly ONE connected road {resolved_one:>9}  ({:.1}%)",
        pct(resolved_one, total)
    );
    println!(
        "  SEVERAL disconnected roads {ambiguous:>9}  ({:.1}%)  <- proximity is guessing",
        pct(ambiguous, total)
    );
    println!(
        "  still several within {TIGHT_M:.0} m     {ambiguous_tight:>9}  ({:.1}%)",
        pct(ambiguous_tight, total)
    );
    if worst.0 > 0 {
        println!(
            "  worst: \"{}\" — {} disconnected roads of that name in reach",
            worst.1, worst.0
        );
    }

    // ── Postcode as the disambiguator, and the Berlin objection to it. ──
    // SECOND ROUND. The first pass learns a component's postcodes only from
    // addresses that already resolved by name and proximity alone — which keeps
    // it non-circular, but leaves thinly-covered components with NO postcode at
    // all. An address then "matches no candidate" for want of information
    // rather than because of a genuine mismatch, and reporting those together
    // overstates the failure.
    //
    // So: resolve once, feed what that settled back in, resolve again. One
    // round only — iterating to a fixed point would let a guess become
    // evidence for the next guess.
    for (pc, cands) in &ambiguous_cases {
        let (Some(pc), true) = (pc, cands.len() > 1) else {
            continue;
        };
        let hits: Vec<usize> = cands
            .iter()
            .copied()
            .filter(|c| comp_pc.get(c).is_some_and(|s| s.contains(pc)))
            .collect();
        if hits.len() == 1 {
            comp_pc.entry(hits[0]).or_default().insert(pc.clone());
        }
    }

    let (mut pc_unique, mut pc_still_several, mut pc_none_matched, mut pc_missing) =
        (0u64, 0u64, 0u64, 0u64);
    let mut pc_no_data = 0u64;
    for (pc, cands) in &ambiguous_cases {
        let Some(pc) = pc else {
            pc_missing += 1;
            continue;
        };
        let known = cands.iter().filter(|c| comp_pc.contains_key(*c)).count();
        let hits = cands
            .iter()
            .filter(|c| comp_pc.get(*c).is_some_and(|s| s.contains(pc)))
            .count();
        match hits {
            // No candidate has ANY postcode on record: the pair could not be
            // applied, which is not the same as the pair failing.
            0 if known == 0 => pc_no_data += 1,
            0 => pc_none_matched += 1,
            1 => pc_unique += 1,
            _ => pc_still_several += 1,
        }
    }
    let multi_pc = comp_pc.values().filter(|s| s.len() > 1).count() as u64;
    let widest = comp_pc.values().map(HashSet::len).max().unwrap_or(0);

    println!("\n(3) does (street, POSTCODE) settle what the name alone could not?");
    println!(
        "  addresses carrying addr:postcode  {with_postcode:>9}  ({:.1}%)",
        pct(with_postcode, total)
    );
    println!("  of the {ambiguous} ambiguous cases:");
    println!(
        "    resolved to exactly ONE         {pc_unique:>9}  ({:.1}%)",
        pct(pc_unique, ambiguous)
    );
    println!(
        "    still several candidates        {pc_still_several:>9}  ({:.1}%)",
        pct(pc_still_several, ambiguous)
    );
    println!(
        "    postcode matched NO candidate   {pc_none_matched:>9}  ({:.1}%)  <- genuine mismatch",
        pct(pc_none_matched, ambiguous)
    );
    println!(
        "    no candidate HAS a postcode     {pc_no_data:>9}  ({:.1}%)  <- missing data, not a failure",
        pct(pc_no_data, ambiguous)
    );
    println!(
        "    address carried no postcode     {pc_missing:>9}  ({:.1}%)",
        pct(pc_missing, ambiguous)
    );

    println!("\n(4) …but a street can SPAN postcodes, so it is not a street property");
    println!(
        "  components whose addresses carry >1 postcode: {multi_pc} of {} ({:.1}%)",
        comp_pc.len(),
        pct(multi_pc, comp_pc.len() as u64)
    );
    println!("  widest: one connected street carrying {widest} distinct postcodes");
    println!("  where that happens, (name, postcode) names a STRETCH, not the street.");

    println!("\n(5) house numbers are not numbers");
    let mut kinds: HashMap<NumKind, u64> = HashMap::new();
    for a in &addrs {
        *kinds.entry(classify_number(&a.number)).or_insert(0) += 1;
    }
    let mut ks: Vec<(&NumKind, &u64)> = kinds.iter().collect();
    ks.sort_by(|a, b| b.1.cmp(a.1));
    for (k, c) in ks {
        println!("  {k:<10?} {c:>9}  ({:.1}%)", pct(*c, total));
    }

    println!("\n(6) codebook pressure — the u24 ceiling is 16,777,215");
    println!("  distinct house numbers {:>9}", distinct_numbers.len());
    println!("  distinct street names   {:>9}", distinct_streets.len());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_range_is_not_a_position_and_a_suffix_is_not_a_range() {
        // Column 3's whole point: anything that sorts or interpolates has to
        // prove it first. Collapsing Range into Plain would let a consumer
        // interpolate "12-14" as the number 12 and place two houses on one.
        assert_eq!(classify_number("12"), NumKind::Plain);
        assert_eq!(classify_number("12a"), NumKind::Suffixed);
        assert_eq!(classify_number("12 B"), NumKind::Suffixed);
        assert_eq!(classify_number("12-14"), NumKind::Range);
        assert_eq!(classify_number("12/14"), NumKind::Range);
        assert_eq!(classify_number("12,14"), NumKind::Range);
        // A separator wins over a suffix: "12a-14b" is a range, not a suffix.
        assert_eq!(classify_number("12a-14b"), NumKind::Range);
        assert_eq!(classify_number(""), NumKind::Other);
        assert_eq!(classify_number("1½"), NumKind::Other);
    }

    #[test]
    fn the_point_to_segment_distance_is_to_the_segment_not_its_ends() {
        // A house opposite the MIDDLE of its street must measure to the middle.
        // Measuring to endpoints would inflate every distance and push
        // addresses past the radius, turning resolvable ones into "too far".
        let d = point_seg((50.0, 10.0), (0.0, 0.0), (100.0, 0.0));
        assert!((d - 10.0).abs() < 1e-9, "got {d}");
        let past = point_seg((150.0, 0.0), (0.0, 0.0), (100.0, 0.0));
        assert!(
            (past - 50.0).abs() < 1e-9,
            "beyond the end measures to the end"
        );
    }

    #[test]
    fn same_named_ways_split_into_components_only_when_disconnected() {
        // The claim inherited from P8, re-checked here because column 2 rests on
        // it: a name is not a road. Two ways sharing a node are ONE road; two
        // sharing nothing are TWO, and an address between them cannot tell which
        // it belongs to from the name alone.
        let mut d = Dsu::new(2);
        assert_ne!(d.find(0), d.find(1), "no shared node, two roads");
        d.union(0, 1);
        assert_eq!(d.find(0), d.find(1), "shared node, one road");
    }
}
