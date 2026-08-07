//! `areal_probe` — **P6**, the natural layer: water, Wald, Wiesen.
//!
//! ```text
//! areal_probe <input.osm.pbf>
//! ```
//!
//! P4 priced the building template: a footprint as anchor + orientation + one
//! turn bit and a length per corner, and it survived because **94.73 %** of
//! footprints are rectilinear. P5 then showed the drivable road is a minority of
//! the network. This probe asks the next obvious question and the one P4's
//! result makes tempting: **does the same template carry the natural layer?**
//!
//! It should not, and the interesting part is *how* it fails. A building is
//! rectilinear because a mason built it; a lake shore, a forest edge and a
//! meadow boundary are surveyed traces of something nobody squared off. Two
//! different shapes want two different encodings, and guessing which is which
//! from the building result is exactly the extrapolation this measures instead.
//!
//! # The amortization question, stated precisely
//!
//! P4(iii) found buildings share **8.58 %** of their edge slots with a
//! neighbour and concluded the branch *resolves toward duplicating* — a shared
//! wall is one varint, and a cross-footprint reference costs more than it saves
//! in a format whose point is that a slot reads alone.
//!
//! The natural layer is where that call could go the other way. A wood does not
//! merely sit *near* a meadow, it is bounded *by* it: the same surveyed line is
//! the forest's edge and the field's edge, and a lake shore is frequently also a
//! landuse boundary. If cross-feature sharing here is materially above 8.58 %,
//! the duplication verdict was a statement about **buildings**, not about the
//! corpus — and that is a correction to P4's scope, not a refutation of it.
//!
//! # Columns
//!
//! 1. **Census** — the areal layer by class, plus linear waterways, which are
//!    chains and belong with the roads rather than with the polygons.
//! 2. **Turn-angle distribution** — the discriminator, with buildings measured
//!    in the **same run by the same code** so the comparison is not across
//!    probes. A turn-bit template is a bet that turns cluster at ±90°.
//! 3. **Rectilinear share** at P2's threshold — the direct §2(c) reuse test.
//! 4. **Encoding price** over the real distribution: raw ids, the PBF's own
//!    delta-varints, §2(c)'s turn bits, and an angle-delta chain — the last
//!    reported **with the drift it accumulates**, because a chain of headings
//!    is not free and quoting only its byte count would hide the cost.
//! 5. **Shared boundary** — the amortization question above, by partner class.
//! 6. **May you walk across it?** — OSM *can* say so (`foot=` / `access=` on the
//!    area), but whether it *does* is empirical. This column reads the tag and
//!    the geometry against each other: does the area declare foot access, and
//!    does a footpath actually run **inside** it (ray-cast, not bbox). The cell
//!    that matters is **silent but walked** — the area a router would refuse to
//!    cross while people demonstrably do.
//!
//! # Method
//!
//! Geometry is computed in a **local equirectangular frame** anchored on the
//! extract's own mean position, with the true meridional and normal radii of
//! curvature there — the same construction P5 uses, and the same reason: over a
//! city span its scale error is far below the thresholds being tested.
//!
//! Angles are measured in that metre frame rather than in cell space. P4 could
//! use cell space because Mercator is conformal and it only needed *relative*
//! right angles; here the **lengths** enter the encoding directly, so the frame
//! has to be metric rather than merely angle-preserving.
//!
//! This probe does not reuse `building_probe`'s `rectilinear_fit`. That function
//! answers a yes/no at one threshold; column 2 needs the whole distribution, and
//! rectilinearity falls out of it as a special case. Reusing the narrower answer
//! would have meant asking the data twice.
//!
//! # Exclusions, stated rather than buried
//!
//! **Multipolygon relations are counted but not reassembled.** A lake with an
//! island is a relation over member ways; this probe measures those member ways
//! as the chains they are, which is what an encoding would store, but it does
//! not stitch outer and inner rings into one polygon — so the census reports
//! ring counts and boundary length, never a corrected surface area for
//! relations. Berlin also has **no coastline**, so `natural=coastline` — the one
//! areal boundary that is genuinely planet-scale — is untested here and any
//! result must not be extrapolated to it.
//!
//! **Column 6's containment test covers closed ways only**, for the same reason:
//! a ray cast needs a ring, and a relation's ring is assembled from members this
//! probe deliberately does not stitch. Large forests are the most likely to be
//! relations, so the walked-share it reports is a floor, not an estimate. It is
//! also **containment, not connectivity**: a path inside a wood says people
//! cross it, not that the router may enter — an access tag on the path itself
//! could still forbid it, and that is the path's business, not the area's.

use std::collections::HashMap;

use osm_soa_bake::geodesy::{meridional_radius, normal_radius};
use osmpbf::{Element, ElementReader};

/// P2's threshold, in metres: one z=24 cell at its coarse end. Same bar P4 used,
/// so the two probes' rectilinear shares are comparable.
const Z24_CELL_M: f64 = 1.69;

/// "Straight" and "square" tolerance for the turn histogram, degrees.
const NEAR_DEG: f64 = 5.0;

/// Heading quantum for the angle-delta chain, degrees.
const ANGLE_QUANTUM_DEG: f64 = 0.5;

/// Class bits, for the shared-boundary attribution.
const C_WATER: u8 = 1 << 0;
const C_WOOD: u8 = 1 << 1;
const C_GREEN: u8 = 1 << 2;
const C_BUILDING: u8 = 1 << 3;
const C_LINEAR_WATER: u8 = 1 << 4;
const C_OTHER: u8 = 1 << 5;

/// A local equirectangular frame — metres from an anchor.
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

/// Which class a way's tags put it in, or `None` if it is not of interest.
fn classify(tags: &[(&str, &str)]) -> Option<(u8, &'static str)> {
    let get = |k: &str| tags.iter().find(|(t, _)| *t == k).map(|(_, v)| *v);

    if tags
        .iter()
        .any(|(k, _)| *k == "building" || *k == "building:part")
    {
        return Some((C_BUILDING, "building"));
    }
    if let Some(w) = get("waterway") {
        return match w {
            "riverbank" => Some((C_WATER, "waterway=riverbank")),
            "river" | "stream" | "canal" | "ditch" | "drain" | "brook" => {
                Some((C_LINEAR_WATER, "waterway (linear)"))
            }
            _ => Some((C_OTHER, "waterway (other)")),
        };
    }
    if let Some(n) = get("natural") {
        return match n {
            "water" => Some((C_WATER, "natural=water")),
            "wetland" => Some((C_WATER, "natural=wetland")),
            "wood" => Some((C_WOOD, "natural=wood")),
            "scrub" | "heath" | "grassland" => Some((C_GREEN, "natural=scrub/heath/grassland")),
            "coastline" => Some((C_WATER, "natural=coastline")),
            _ => Some((C_OTHER, "natural (other)")),
        };
    }
    if let Some(l) = get("landuse") {
        return match l {
            "forest" => Some((C_WOOD, "landuse=forest")),
            "reservoir" | "basin" => Some((C_WATER, "landuse=reservoir/basin")),
            "meadow" | "grass" | "village_green" | "greenfield" => {
                Some((C_GREEN, "landuse=meadow/grass"))
            }
            "farmland" | "orchard" | "vineyard" | "allotments" | "farmyard" => {
                Some((C_GREEN, "landuse=farm/allotment"))
            }
            "forestry" => Some((C_WOOD, "landuse=forestry")),
            _ => Some((C_OTHER, "landuse (other)")),
        };
    }
    if let Some(l) = get("leisure") {
        return match l {
            // Split deliberately: a `leisure=garden` is frequently a PRIVATE
            // garden carrying `access=private`, and lumping it with public
            // parks makes column 6 report "37 % of parks forbid walking" —
            // a labelling artefact, not a fact about parks.
            "park" | "nature_reserve" | "common" => Some((C_GREEN, "leisure=park/reserve")),
            "garden" => Some((C_GREEN, "leisure=garden")),
            _ => None,
        };
    }
    None
}

/// `highway=*` values a pedestrian actually uses, for column 6's geometry side.
const FOOT_HIGHWAY: &[&str] = &[
    "footway",
    "path",
    "steps",
    "pedestrian",
    "living_street",
    "track",
    "bridleway",
    "cycleway",
];

/// What an area's own tags say about crossing it on foot.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Foot {
    /// `foot`/`access` says yes, permissive, designated, destination.
    Allowed,
    /// `foot`/`access` says no or private.
    Denied,
    /// Neither key is present — the case that makes column 6 worth measuring.
    Silent,
}

/// Read the access question off an area's tags.
///
/// `foot` wins over `access` when both are present: it is the specific key, and
/// a `access=private` + `foot=yes` wood is open to walkers and closed to cars.
fn foot_access(tags: &[(&str, &str)]) -> Foot {
    let get = |k: &str| tags.iter().find(|(t, _)| *t == k).map(|(_, v)| *v);
    for key in ["foot", "access"] {
        let Some(v) = get(key) else { continue };
        return match v {
            "yes" | "permissive" | "designated" | "official" | "destination" | "customers"
            | "public" => Foot::Allowed,
            "no" | "private" | "forestry" | "agricultural" | "delivery" => Foot::Denied,
            _ => Foot::Silent,
        };
    }
    Foot::Silent
}

/// Is `p` strictly inside the closed ring `ring`? Crossing-number ray cast.
///
/// The half-open rule on the y comparison (`>` on one end, `<=` on the other) is
/// what makes a vertex exactly at the ray's height count once rather than twice
/// — without it a point level with a vertex flips to the wrong answer.
fn point_in_ring(p: (f64, f64), ring: &[(f64, f64)]) -> bool {
    let mut inside = false;
    for w in ring.windows(2) {
        let (a, b) = (w[0], w[1]);
        if (a.1 > p.1) != (b.1 > p.1) {
            let t = (p.1 - a.1) / (b.1 - a.1);
            if p.0 < a.0 + t * (b.0 - a.0) {
                inside = !inside;
            }
        }
    }
    inside
}

/// A closed areal ring held back for column 6.
struct Ring {
    label: &'static str,
    pts: Vec<(f64, f64)>,
    foot: Foot,
    min: (f64, f64),
    max: (f64, f64),
}

/// Column-6 tally for one tag.
#[derive(Default)]
struct Access {
    rings: u64,
    allowed: u64,
    denied: u64,
    silent: u64,
    walked: u64,
    /// The cell that matters: a path runs through it and nothing says you may.
    silent_walked: u64,
    /// The other side of it: the tag forbids what the geometry shows.
    denied_walked: u64,
}

fn class_name(bit: u8) -> &'static str {
    match bit {
        C_WATER => "water",
        C_WOOD => "wood",
        C_GREEN => "green",
        C_BUILDING => "building",
        C_LINEAR_WATER => "linear water",
        _ => "other",
    }
}

/// Bytes an LEB128 varint of `v` occupies.
fn varint_len(v: u64) -> usize {
    let mut n = 1;
    let mut v = v >> 7;
    while v > 0 {
        n += 1;
        v >>= 7;
    }
    n
}

/// Zig-zag, so a signed delta costs by magnitude rather than by sign.
fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}

/// Wrap to `(-π, π]`.
fn wrap_pi(mut a: f64) -> f64 {
    while a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    while a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

fn q(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

/// Per-class accumulators.
#[derive(Default)]
struct Acc {
    ways: u64,
    nodes: u64,
    boundary_m: f64,
    area_m2: f64,
    closed: u64,
    /// Turn magnitudes at every interior vertex, degrees.
    turns: Vec<f64>,
    near_straight: u64,
    near_square: u64,
    /// Closed ways passing the P2 rectilinear bar.
    rectilinear: u64,
    /// (raw ids, pbf delta-varint, turn-bit, angle-chain) bytes.
    b_raw: u64,
    b_pbf: u64,
    b_turnbit: u64,
    b_angle: u64,
    /// Worst reconstruction error of the angle chain, metres, per way.
    drift: Vec<f64>,
    /// Worst reconstruction error of the **turn-bit** form, metres, per way.
    ///
    /// Priced separately because a turn bit cannot express a 17° turn at all.
    /// Without this the byte column reads as though §2(c) were cheapest for
    /// every class, when for a curve it is cheap the way an empty box is light.
    turnbit_drift: Vec<f64>,
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

/// Measure one chain: turns, area, and what four encodings would spend on it.
#[allow(clippy::too_many_lines)]
fn measure(acc: &mut Acc, ids: &[i64], pts: &[(f64, f64)], ll: &[(f64, f64)]) {
    let n = pts.len();
    acc.ways += 1;
    acc.nodes += n as u64;
    let closed = n >= 4 && ids.first() == ids.last();
    if closed {
        acc.closed += 1;
    }

    // Segment headings and lengths, in the metre frame.
    let mut head = Vec::with_capacity(n.saturating_sub(1));
    let mut len = Vec::with_capacity(n.saturating_sub(1));
    for w in pts.windows(2) {
        let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
        head.push(dy.atan2(dx));
        let l = (dx * dx + dy * dy).sqrt();
        len.push(l);
        acc.boundary_m += l;
    }
    if head.is_empty() {
        return;
    }

    // Shoelace, in metres, for closed rings only.
    if closed {
        let mut s = 0.0;
        for w in pts.windows(2) {
            s += w[0].0 * w[1].1 - w[1].0 * w[0].1;
        }
        acc.area_m2 += (s / 2.0).abs();
    }

    // ── Column 2: the turn distribution. ──
    for w in head.windows(2) {
        let t = wrap_pi(w[1] - w[0]).to_degrees();
        let a = t.abs();
        acc.turns.push(a);
        if a <= NEAR_DEG {
            acc.near_straight += 1;
        }
        if (a - 90.0).abs() <= NEAR_DEG {
            acc.near_square += 1;
        }
    }

    // ── Column 3: rectilinearity, at P2's bar, as a displacement. ──
    //
    // The dominant orientation is the longest edge's bearing mod 90°; an edge's
    // displacement is `length · sin(deviation)` — the same definition P4 used,
    // so the two numbers are comparable rather than merely similar.
    if closed {
        let longest = len
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);
        let theta = head[longest].rem_euclid(std::f64::consts::FRAC_PI_2);
        let mut worst: f64 = 0.0;
        for (h, l) in head.iter().zip(len.iter()) {
            let dev = wrap_pi(h - theta).rem_euclid(std::f64::consts::FRAC_PI_2);
            let dev = dev.min(std::f64::consts::FRAC_PI_2 - dev);
            worst = worst.max(l * dev.sin());
        }
        if worst < Z24_CELL_M {
            acc.rectilinear += 1;
        }
    }

    // ── Column 4: four prices for the same chain. ──

    // A. Raw id list — what a row-based store pays.
    acc.b_raw += n as u64 * 8;

    // B. The PBF's own delta-varints. The honest baseline: the format being
    //    replaced is not naive.
    let (mut pid, mut plat, mut plon) = (0i64, 0i64, 0i64);
    for (i, &id) in ids.iter().enumerate() {
        let (lat, lon) = ll[i];
        let (la, lo) = ((lat * 1e7).round() as i64, (lon * 1e7).round() as i64);
        acc.b_pbf += varint_len(zigzag(id - pid)) as u64
            + varint_len(zigzag(la - plat)) as u64
            + varint_len(zigzag(lo - plon)) as u64;
        pid = id;
        plat = la;
        plon = lo;
    }

    // C. §2(c)'s turn-bit form: θ as u16, one turn bit per corner, one length
    //    varint per edge. On a rectilinear footprint the bit IS the turn; on a
    //    curve it is a lie, and this column exists to price that lie rather
    //    than to refuse to compute it.
    acc.b_turnbit += 2
        + (n as u64).div_ceil(8)
        + len
            .iter()
            .map(|l| varint_len((l * 100.0) as u64) as u64)
            .sum::<u64>();

    // …and what a turn BIT reconstructs. The bit can only say "left" or
    // "right", so the decoder must turn a full 90° each corner. On a footprint
    // that is exactly right; on a shore it is the encoding claiming a shape the
    // data does not have, and the error is the size of that claim.
    {
        let mut h = head[0];
        let (mut x, mut y) = pts[0];
        let mut worst: f64 = 0.0;
        for (i, l) in len.iter().enumerate() {
            if i > 0 {
                let turn = wrap_pi(head[i] - head[i - 1]);
                h += if turn >= 0.0 {
                    std::f64::consts::FRAC_PI_2
                } else {
                    -std::f64::consts::FRAC_PI_2
                };
            }
            x += l * h.cos();
            y += l * h.sin();
            let (tx, ty) = pts[i + 1];
            worst = worst.max(((x - tx).powi(2) + (y - ty).powi(2)).sqrt());
        }
        acc.turnbit_drift.push(worst);
    }

    // D. Angle-delta chain: θ0 as u16, then per vertex a zigzag varint of the
    //    quantised heading change and a length varint.
    let mut b = 2u64;
    for l in &len {
        b += varint_len((l * 100.0) as u64) as u64;
    }
    let mut qd: Vec<i64> = Vec::with_capacity(head.len());
    for w in head.windows(2) {
        let d = wrap_pi(w[1] - w[0]).to_degrees();
        let k = (d / ANGLE_QUANTUM_DEG).round() as i64;
        qd.push(k);
        b += varint_len(zigzag(k)) as u64;
    }
    acc.b_angle += b;

    // …and what that costs in position. A heading chain integrates its own
    // quantisation error, so the deviation GROWS along the way — the reason
    // this column reports drift next to bytes instead of bytes alone.
    let q0 = (head[0].to_degrees() / ANGLE_QUANTUM_DEG).round() * ANGLE_QUANTUM_DEG;
    let mut h = q0.to_radians();
    let (mut x, mut y) = pts[0];
    let mut worst: f64 = 0.0;
    for (i, l) in len.iter().enumerate() {
        if i > 0 {
            h += (qd[i - 1] as f64 * ANGLE_QUANTUM_DEG).to_radians();
        }
        // Length is stored in centimetres, so quantise it here too.
        let lq = ((l * 100.0).round()) / 100.0;
        x += lq * h.cos();
        y += lq * h.sin();
        let (tx, ty) = pts[i + 1];
        worst = worst.max(((x - tx).powi(2) + (y - ty).powi(2)).sqrt());
    }
    acc.drift.push(worst);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: areal_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    // ── Pass 1: node coordinates and the frame anchor. ──
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
    let nn = coords.len().max(1) as f64;
    let frame = Frame::new(sum_lat / nn, sum_lon / nn);
    eprintln!("pass 1: {} nodes", coords.len());

    // ── Pass 2: ways. ──
    let mut by_tag: HashMap<&'static str, Acc> = HashMap::new();
    let mut by_class: HashMap<u8, Acc> = HashMap::new();
    // Undirected edge → the class bits of every way that uses it.
    let mut edge_use: HashMap<(i64, i64), u8> = HashMap::with_capacity(6_000_000);
    // Per areal way: its class and its edges, so column 5 can attribute after.
    let mut areal_edges: Vec<(u8, Vec<(i64, i64)>)> = Vec::with_capacity(100_000);
    let mut clipped = 0u64;
    // Column 6: every pedestrian node, and every closed wood/green ring.
    let mut foot_pts: Vec<(f64, f64)> = Vec::with_capacity(1_200_000);
    let mut rings: Vec<Ring> = Vec::with_capacity(60_000);

    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Way(w) = el else { return };
            let tags: Vec<(&str, &str)> = w.tags().collect();
            if tags.is_empty() {
                return;
            }
            if let Some(hw) = tags.iter().find(|(k, _)| *k == "highway").map(|(_, v)| *v) {
                if FOOT_HIGHWAY.contains(&hw) {
                    for id in w.refs() {
                        if let Some(&(la, lo)) = coords.get(&id) {
                            foot_pts.push(frame.xy(la, lo));
                        }
                    }
                }
                return;
            }
            let Some((bit, label)) = classify(&tags) else {
                return;
            };
            let ids: Vec<i64> = w.refs().collect();
            if ids.len() < 2 {
                return;
            }

            for pair in ids.windows(2) {
                let e = (pair[0].min(pair[1]), pair[0].max(pair[1]));
                *edge_use.entry(e).or_insert(0) |= bit;
            }
            if bit & (C_WATER | C_WOOD | C_GREEN) != 0 {
                areal_edges.push((
                    bit,
                    ids.windows(2)
                        .map(|p| (p[0].min(p[1]), p[0].max(p[1])))
                        .collect(),
                ));
            }
            if bit == C_OTHER {
                return;
            }

            let ll: Vec<(f64, f64)> = ids.iter().filter_map(|i| coords.get(i).copied()).collect();
            if ll.len() != ids.len() {
                clipped += 1;
                return;
            }
            let pts: Vec<(f64, f64)> = ll.iter().map(|&(la, lo)| frame.xy(la, lo)).collect();

            measure(by_tag.entry(label).or_default(), &ids, &pts, &ll);
            measure(by_class.entry(bit).or_default(), &ids, &pts, &ll);

            // Column 6 takes closed wood/green rings only — see the exclusions.
            if bit & (C_WOOD | C_GREEN) != 0 && pts.len() >= 4 && ids.first() == ids.last() {
                let mut min = (f64::MAX, f64::MAX);
                let mut max = (f64::MIN, f64::MIN);
                for &(x, y) in &pts {
                    min = (min.0.min(x), min.1.min(y));
                    max = (max.0.max(x), max.1.max(y));
                }
                rings.push(Ring {
                    label,
                    pts,
                    foot: foot_access(&tags),
                    min,
                    max,
                });
            }
        })
        .expect("pass 2");

    // ── Pass 3: multipolygon relations, counted not reassembled. ──
    let mut rel_rings: HashMap<&'static str, (u64, u64)> = HashMap::new();
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Relation(r) = el else { return };
            let tags: Vec<(&str, &str)> = r.tags().collect();
            if tags.is_empty() {
                return;
            }
            let Some((bit, label)) = classify(&tags) else {
                return;
            };
            if bit & (C_WATER | C_WOOD | C_GREEN) == 0 {
                return;
            }
            let members = r.members().count() as u64;
            let e = rel_rings.entry(label).or_insert((0, 0));
            e.0 += 1;
            e.1 += members;
        })
        .expect("pass 3");

    // ── Column 5: shared boundary. ──
    let mut shared: HashMap<u8, (u64, u64, HashMap<u8, u64>)> = HashMap::new();
    for (bit, edges) in &areal_edges {
        let entry = shared.entry(*bit).or_insert((0, 0, HashMap::new()));
        for e in edges {
            entry.0 += 1;
            let Some(&mask) = edge_use.get(e) else {
                continue;
            };
            // Shared means: some OTHER class also uses this edge, or the same
            // class uses it twice — the latter is invisible in a bitmask, so
            // only cross-class sharing is claimed here.
            let others = mask & !*bit;
            if others != 0 {
                entry.1 += 1;
                for b in [
                    C_WATER,
                    C_WOOD,
                    C_GREEN,
                    C_BUILDING,
                    C_LINEAR_WATER,
                    C_OTHER,
                ] {
                    if others & b != 0 {
                        *entry.2.entry(b).or_insert(0) += 1;
                    }
                }
            }
        }
    }

    // ── Column 6: does a footpath actually run inside? ──
    //
    // A grid over pedestrian nodes so a ring tests its own bbox rather than the
    // whole extract; the ray cast itself is exact, the grid only narrows the
    // candidate set.
    const FOOT_CELL_M: f64 = 64.0;
    let mut fgrid: HashMap<(i32, i32), Vec<(f32, f32)>> = HashMap::new();
    for &(x, y) in &foot_pts {
        fgrid
            .entry((
                (x / FOOT_CELL_M).floor() as i32,
                (y / FOOT_CELL_M).floor() as i32,
            ))
            .or_default()
            .push((x as f32, y as f32));
    }
    eprintln!(
        "pass 2: {} pedestrian nodes, {} closed wood/green rings",
        foot_pts.len(),
        rings.len()
    );

    let mut access: HashMap<&'static str, Access> = HashMap::new();
    for r in &rings {
        let a = access.entry(r.label).or_default();
        a.rings += 1;
        match r.foot {
            Foot::Allowed => a.allowed += 1,
            Foot::Denied => a.denied += 1,
            Foot::Silent => a.silent += 1,
        }
        let (cx0, cy0) = (
            (r.min.0 / FOOT_CELL_M).floor() as i32,
            (r.min.1 / FOOT_CELL_M).floor() as i32,
        );
        let (cx1, cy1) = (
            (r.max.0 / FOOT_CELL_M).floor() as i32,
            (r.max.1 / FOOT_CELL_M).floor() as i32,
        );
        let mut walked = false;
        'outer: for cx in cx0..=cx1 {
            for cy in cy0..=cy1 {
                let Some(pts) = fgrid.get(&(cx, cy)) else {
                    continue;
                };
                for &(px, py) in pts {
                    let p = (px as f64, py as f64);
                    if p.0 < r.min.0 || p.0 > r.max.0 || p.1 < r.min.1 || p.1 > r.max.1 {
                        continue;
                    }
                    if point_in_ring(p, &r.pts) {
                        walked = true;
                        break 'outer;
                    }
                }
            }
        }
        if walked {
            a.walked += 1;
            match r.foot {
                Foot::Silent => a.silent_walked += 1,
                Foot::Denied => a.denied_walked += 1,
                Foot::Allowed => {}
            }
        }
    }

    // ── Report. ──
    println!("\n(i) the areal layer — census ({clipped} ways clipped out of the extract)");
    println!(
        "{:<30} {:>8} {:>10} {:>11} {:>11} {:>8}",
        "tag", "ways", "nodes", "boundary km", "area km2", "closed"
    );
    let mut rows: Vec<(&&str, &Acc)> = by_tag.iter().collect();
    rows.sort_by(|a, b| {
        b.1.boundary_m
            .partial_cmp(&a.1.boundary_m)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (t, a) in &rows {
        println!(
            "{:<30} {:>8} {:>10} {:>11.1} {:>11.2} {:>7.0}%",
            t,
            a.ways,
            a.nodes,
            a.boundary_m / 1000.0,
            a.area_m2 / 1e6,
            pct(a.closed, a.ways)
        );
    }
    if !rel_rings.is_empty() {
        println!("\n  multipolygon relations (counted, not reassembled — see module docs)");
        let mut rr: Vec<(&&str, &(u64, u64))> = rel_rings.iter().collect();
        rr.sort_by_key(|r| std::cmp::Reverse(r.1 .0));
        for (t, (n, m)) in &rr {
            println!("  {t:<28} {n:>8} relations {m:>9} member rings");
        }
    }

    println!("\n(ii) turn-angle distribution — the discriminator");
    println!("  a turn-bit template is a bet that turns cluster at +/-90 deg.");
    println!(
        "{:<16} {:>10} {:>12} {:>12} {:>10} {:>10}",
        "class", "vertices", "straight<5", "square+-5", "median", "p95"
    );
    let order = [C_BUILDING, C_WATER, C_WOOD, C_GREEN, C_LINEAR_WATER];
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        let mut t = a.turns.clone();
        t.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        println!(
            "{:<16} {:>10} {:>11.1}% {:>11.1}% {:>9.1}d {:>9.1}d",
            class_name(bit),
            t.len(),
            pct(a.near_straight, t.len() as u64),
            pct(a.near_square, t.len() as u64),
            q(&t, 0.5),
            q(&t, 0.95)
        );
    }

    println!("\n(iii) rectilinear share of closed ways, at P2's 1.69 m bar");
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        if a.closed == 0 {
            continue;
        }
        println!(
            "  {:<14} {:>8} of {:>8} closed  ({:.2}%)",
            class_name(bit),
            a.rectilinear,
            a.closed,
            pct(a.rectilinear, a.closed)
        );
    }

    println!("\n(iv) four prices for the same chains, MB — and what the chain costs in position");
    println!("  bytes are only half the price — a reconstruction error column sits beside each");
    println!("  lossy form, because a form that cannot express the shape is cheap for no reason.");
    println!(
        "{:<14} {:>8} {:>8} {:>9} {:>10} {:>8} {:>10}",
        "class", "raw ids", "pbf", "turn-bit", "  err p95", "angle", "  err p95"
    );
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        let mut d = a.drift.clone();
        d.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let mut tb = a.turnbit_drift.clone();
        tb.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let mb = |v: u64| v as f64 / 1e6;
        println!(
            "{:<14} {:>8.2} {:>8.2} {:>9.2} {:>9.1}m {:>8.2} {:>9.2}m",
            class_name(bit),
            mb(a.b_raw),
            mb(a.b_pbf),
            mb(a.b_turnbit),
            q(&tb, 0.95),
            mb(a.b_angle),
            q(&d, 0.95)
        );
    }

    println!("\n(v) shared boundary — P4 measured buildings at 8.58% of edge slots");
    for bit in [C_WATER, C_WOOD, C_GREEN] {
        let Some((slots, sh, partners)) = shared.get(&bit) else {
            continue;
        };
        println!(
            "  {:<8} {:>9} edge slots, {:>9} shared with another class  ({:.2}%)",
            class_name(bit),
            slots,
            sh,
            pct_f(*sh as f64, *slots as f64)
        );
        let mut ps: Vec<(&u8, &u64)> = partners.iter().collect();
        ps.sort_by(|a, b| b.1.cmp(a.1));
        for (p, c) in ps.iter().take(5) {
            println!("      with {:<14} {:>9}", class_name(**p), c);
        }
    }

    println!("\n(vi) may you walk across it? — the tag, against the geometry");
    println!(
        "  closed wood/green rings only; \"walked\" = a pedestrian node lies INSIDE the ring."
    );
    println!(
        "{:<30} {:>7} {:>8} {:>8} {:>8} {:>8} {:>14}",
        "tag", "rings", "allowed", "denied", "silent", "walked", "silent+walked"
    );
    let mut ac: Vec<(&&str, &Access)> = access.iter().collect();
    ac.sort_by_key(|a| std::cmp::Reverse(a.1.rings));
    let (mut t_rings, mut t_silent, mut t_walked, mut t_sw, mut t_dw) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    for (t, a) in &ac {
        println!(
            "{:<30} {:>7} {:>7.1}% {:>7.1}% {:>7.1}% {:>7.1}% {:>8} ({:.1}%)",
            t,
            a.rings,
            pct(a.allowed, a.rings),
            pct(a.denied, a.rings),
            pct(a.silent, a.rings),
            pct(a.walked, a.rings),
            a.silent_walked,
            pct(a.silent_walked, a.rings),
        );
        t_rings += a.rings;
        t_silent += a.silent;
        t_walked += a.walked;
        t_sw += a.silent_walked;
        t_dw += a.denied_walked;
    }
    println!(
        "\n  {t_silent} of {t_rings} rings ({:.1}%) say nothing about foot access;",
        pct(t_silent, t_rings)
    );
    println!(
        "  {t_walked} ({:.1}%) have a path running through them, and {t_sw} ({:.1}% of all rings)",
        pct(t_walked, t_rings),
        pct(t_sw, t_rings)
    );
    println!("  are walked while saying nothing — a router refusing silence refuses those.");
    println!(
        "  the other side: {t_dw} rings carry a path inside while their own tag DENIES foot access."
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A closed unit square, 10 m on a side, in the metre frame.
    fn square() -> (Vec<i64>, Vec<(f64, f64)>) {
        (
            vec![1, 2, 3, 4, 1],
            vec![
                (0.0, 0.0),
                (10.0, 0.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 0.0),
            ],
        )
    }

    /// A closed `n`-gon of radius 50 m — a lake shore's shape, no right angle
    /// anywhere. `n` sets the turn per step (`360/n` degrees), which is the
    /// quantity column 2 buckets, so it is a parameter rather than a constant:
    /// a coarse ring and a densely surveyed one land in different buckets and
    /// the tests need to say which they mean.
    fn ngon(n: usize) -> (Vec<i64>, Vec<(f64, f64)>) {
        let mut ids = Vec::new();
        let mut pts = Vec::new();
        for i in 0..=n {
            let a = std::f64::consts::TAU * (i % n) as f64 / n as f64;
            ids.push((i % n) as i64 + 1);
            pts.push((50.0 * a.cos(), 50.0 * a.sin()));
        }
        (ids, pts)
    }

    #[test]
    fn the_turn_histogram_separates_a_square_from_a_circle() {
        // Column 2's whole claim. A square must land in the "square" bucket and
        // a circle in the "straight" bucket; if a bug collapsed the two buckets
        // the discriminator would report both shapes alike and the probe's
        // conclusion would be unfalsifiable.
        let (ids, pts) = square();
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut sq = Acc::default();
        measure(&mut sq, &ids, &pts, &ll);
        assert_eq!(
            sq.near_square, 3,
            "a square turns 90 deg at every interior vertex"
        );
        assert_eq!(sq.near_straight, 0, "and never goes straight on");

        // A densely surveyed shore: 120 steps of 3 deg, inside the 5 deg bar.
        let (ids, pts) = ngon(120);
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut ci = Acc::default();
        measure(&mut ci, &ids, &pts, &ll);
        assert_eq!(ci.near_square, 0, "a smooth ring has no right angle");
        assert!(
            ci.near_straight >= 115,
            "3 deg a step is inside the {NEAR_DEG} deg bar (got {})",
            ci.near_straight
        );

        // …and the bar is a real bar, not a synonym for "curved": a COARSE
        // ring turns 9 deg a step and must fall outside it. Without this half,
        // widening NEAR_DEG until everything counts as straight would pass.
        let (ids, pts) = ngon(40);
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut coarse = Acc::default();
        measure(&mut coarse, &ids, &pts, &ll);
        assert_eq!(
            (coarse.near_straight, coarse.near_square),
            (0, 0),
            "9 deg a step is neither straight nor square"
        );
    }

    #[test]
    fn rectilinearity_passes_the_square_and_fails_the_circle() {
        // Column 3, two-sided. The circle's radius is chosen so its deviation
        // is metres, not centimetres — a threshold bug that let everything pass
        // has to fail here.
        let (ids, pts) = square();
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut sq = Acc::default();
        measure(&mut sq, &ids, &pts, &ll);
        assert_eq!((sq.closed, sq.rectilinear), (1, 1));

        let (ids, pts) = ngon(40);
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut ci = Acc::default();
        measure(&mut ci, &ids, &pts, &ll);
        assert_eq!(
            (ci.closed, ci.rectilinear),
            (1, 0),
            "a ring is not rectilinear"
        );
    }

    #[test]
    fn the_shoelace_area_is_the_real_area() {
        // 10x10 m = 100 m2. An orientation bug (signed area kept negative) or a
        // missing halving would show up as a wrong number, not merely a warning.
        let (ids, pts) = square();
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut a = Acc::default();
        measure(&mut a, &ids, &pts, &ll);
        assert!((a.area_m2 - 100.0).abs() < 1e-6, "got {}", a.area_m2);
        assert!((a.boundary_m - 40.0).abs() < 1e-6, "got {}", a.boundary_m);
    }

    #[test]
    fn the_angle_chain_drifts_and_the_probe_measures_it() {
        // The honesty column. A heading chain integrates its quantisation
        // error, so drift must be strictly positive on a curve — a probe that
        // reported zero would be quoting the chain's byte count as free.
        let (ids, pts) = ngon(120);
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut a = Acc::default();
        measure(&mut a, &ids, &pts, &ll);
        let d = a.drift[0];
        assert!(d > 0.0, "quantised headings cannot reconstruct exactly");
        assert!(
            d < 5.0,
            "…but 0.5 deg steps must not wander metres off (got {d})"
        );
    }

    #[test]
    fn the_turn_bit_is_exact_on_a_square_and_hopeless_on_a_ring() {
        // Column 4's honesty claim, two-sided. A turn BIT can only say left or
        // right, so it reproduces a footprint exactly and cannot reproduce a
        // curve at all. Without the second half the byte column would read as
        // though §2(c) were simply cheapest for everything.
        let (ids, pts) = square();
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut sq = Acc::default();
        measure(&mut sq, &ids, &pts, &ll);
        assert!(
            sq.turnbit_drift[0] < 1e-6,
            "a right angle IS the bit (got {})",
            sq.turnbit_drift[0]
        );

        let (ids, pts) = ngon(120);
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut ring = Acc::default();
        measure(&mut ring, &ids, &pts, &ll);
        assert!(
            ring.turnbit_drift[0] > 50.0,
            "forcing 90 deg on a 3 deg turn must wander far off a 50 m ring (got {})",
            ring.turnbit_drift[0]
        );
    }

    #[test]
    fn the_ray_cast_answers_inside_and_outside_and_survives_a_vertex_level_point() {
        // Column 6's geometry side, two-sided plus the classic ray-cast bug: a
        // point exactly level with a vertex must be counted once, not twice. The
        // (5, 5) point sits at the height of two vertices of this diamond, which
        // is what breaks a naive `>=`/`>=` comparison.
        let diamond = vec![(0.0, 5.0), (5.0, 0.0), (10.0, 5.0), (5.0, 10.0), (0.0, 5.0)];
        assert!(point_in_ring((5.0, 5.0), &diamond), "centre is inside");
        assert!(
            !point_in_ring((0.5, 0.5), &diamond),
            "corner region is outside"
        );
        assert!(!point_in_ring((20.0, 5.0), &diamond), "far away is outside");

        let square_ring = square().1;
        assert!(point_in_ring((5.0, 5.0), &square_ring));
        assert!(!point_in_ring((15.0, 5.0), &square_ring));
    }

    #[test]
    fn foot_access_reads_the_specific_key_first_and_silence_is_not_denial() {
        // The whole point of column 6: absence is its own category. Folding
        // Silent into Denied would make "silent+walked" — the finding — vanish.
        assert_eq!(foot_access(&[("landuse", "meadow")]), Foot::Silent);
        assert_eq!(foot_access(&[("foot", "yes")]), Foot::Allowed);
        assert_eq!(foot_access(&[("foot", "no")]), Foot::Denied);
        assert_eq!(foot_access(&[("access", "private")]), Foot::Denied);
        // `foot` is the specific key and must win: a wood closed to cars but
        // open to walkers is Allowed, and reading `access` first would call it
        // Denied and hide a legitimately walkable area.
        assert_eq!(
            foot_access(&[("access", "private"), ("foot", "yes")]),
            Foot::Allowed
        );
        // An unrecognised value is silence, not a guess.
        assert_eq!(foot_access(&[("foot", "unknown_value")]), Foot::Silent);
    }

    #[test]
    fn classify_puts_wald_wiesen_and_wasser_where_they_belong() {
        // The tag rule, including the trap: `waterway=river` is a LINE and
        // `waterway=riverbank` is an AREA, so they must not share a class —
        // column 4 prices them differently on purpose.
        let c = |t: &[(&str, &str)]| classify(t).map(|(b, _)| b);
        assert_eq!(c(&[("natural", "water")]), Some(C_WATER));
        assert_eq!(c(&[("waterway", "riverbank")]), Some(C_WATER));
        assert_eq!(c(&[("waterway", "river")]), Some(C_LINEAR_WATER));
        assert_eq!(c(&[("natural", "wood")]), Some(C_WOOD));
        assert_eq!(c(&[("landuse", "forest")]), Some(C_WOOD));
        assert_eq!(c(&[("landuse", "meadow")]), Some(C_GREEN));
        assert_eq!(c(&[("leisure", "park")]), Some(C_GREEN));
        // A building wins over any area tag it also carries: it is the class P4
        // measured, and double-counting it here would corrupt the comparison.
        assert_eq!(
            c(&[("building", "yes"), ("landuse", "forest")]),
            Some(C_BUILDING)
        );
        assert_eq!(c(&[("highway", "residential")]), None);
    }
}
