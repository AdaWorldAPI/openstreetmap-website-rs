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

use osm_soa_bake::curve::{
    bezier_segments, fit_circle, fit_clothoid, fit_cubic_bezier, varint_len, wrap_pi, zigzag,
};
use osm_soa_bake::geodesy::{meridional_radius, normal_radius};
use osm_soa_bake::tms::{self, TileXy};
use osmpbf::{Element, ElementReader};

/// P2's threshold, in metres: one z=24 cell at its coarse end. Same bar P4 used,
/// so the two probes' rectilinear shares are comparable.
const Z24_CELL_M: f64 = 1.69;

/// "Straight" and "square" tolerance for the turn histogram, degrees.
const NEAR_DEG: f64 = 5.0;

/// Heading quantum for the angle-delta chain, degrees.
const ANGLE_QUANTUM_DEG: f64 = 0.5;

/// The shortest run that can mean anything on a stride-4-over-17 curve ruler.
///
/// `gcd(4, 17) = 1`, so the walk is a full permutation of all 17 residues — but
/// only after **17** steps. A run shorter than that has not visited the ruler;
/// calling it "on the template" is reading structure into noise. Runs of 3 are
/// reported alongside purely to show what that naive floor would have claimed.
const MEANINGFUL_RUN: usize = 17;

/// The stricter floor, one Fibonacci step up.
const LONG_RUN: usize = 21;

/// Zoom of the tile a client's local coordinates are relative to.
///
/// **f32 cannot carry a global z=32 coordinate**: a 24-bit mantissa against a
/// 32-bit coordinate drops 8 bits = 256 cells, and at the equator (9.33 mm per
/// z=32 cell — the world width over 2^32, NOT the 6.59 mm round-trip error,
/// which is a different quantity) that is **~2.39 m — OVER P2's 1.69 m bar**,
/// not merely at it. So the wire
/// carries a tile id plus a **u16 offset within the tile**, quantized-mesh
/// style. A u16 offset spans 16 bits, so the tile is z = 32 - 16 = **16**, and
/// full z=32 precision is preserved exactly.
///
/// The consequence this column exists to price: a chain that leaves its tile
/// must be SPLIT there. A z=16 tile is 40_075_017 / 2^16 = 611.5 m at the
/// equator, ~372 m at Berlin's latitude — shorter than many ways.
const TILE_Z: u32 = 16;

/// Bytes one wire point costs: two u16 tile-local offsets.
const WIRE_POINT_B: u64 = 4;

/// Class bits, for the shared-boundary attribution.
const C_WATER: u16 = 1 << 0;
const C_WOOD: u16 = 1 << 1;
const C_GREEN: u16 = 1 << 2;
const C_BUILDING: u16 = 1 << 3;
const C_LINEAR_WATER: u16 = 1 << 4;
const C_OTHER: u16 = 1 << 5;
/// The classified network — the roads laid out to a design standard (RAL/RAS-L),
/// whose alignment is by construction straight / clothoid / circular arc.
const C_ROAD: u16 = 1 << 6;
/// Residential and unclassified — the CONTROL inside the road family. If a
/// design standard is what makes a chain ride a template, these should ride it
/// less well than the classified network above.
const C_RESID: u16 = 1 << 7;
/// `junction=roundabout` — the circular class, and it is TAGGED rather than
/// guessed. This is the one place a rational Bezier could pay for itself, so it
/// gets measured on its own instead of being averaged into the road rows.
const C_ROUNDABOUT: u16 = 1 << 8;

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
fn classify(tags: &[(&str, &str)]) -> Option<(u16, &'static str)> {
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
    if let Some(h) = get("highway") {
        if get("junction") == Some("roundabout") {
            return Some((C_ROUNDABOUT, "junction=roundabout"));
        }
        return match h {
            "motorway" | "trunk" | "primary" | "secondary" | "tertiary" | "motorway_link"
            | "trunk_link" | "primary_link" | "secondary_link" | "tertiary_link" => {
                Some((C_ROAD, "highway (classified)"))
            }
            "residential" | "unclassified" | "living_street" => {
                Some((C_RESID, "highway (residential)"))
            }
            _ => None,
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

fn class_name(bit: u16) -> &'static str {
    match bit {
        C_WATER => "water",
        C_WOOD => "wood",
        C_GREEN => "green",
        C_BUILDING => "building",
        C_LINEAR_WATER => "linear water",
        C_ROAD => "road (DIN)",
        C_ROUNDABOUT => "roundabout",
        C_RESID => "residential",
        _ => "other",
    }
}

/// Bits a **Fibonacci (Zeckendorf) codeword** for `n >= 1` occupies.
///
/// Every positive integer is a unique sum of non-consecutive Fibonacci numbers,
/// so the bit pattern never contains `11` until the appended terminator — which
/// is what makes the code self-delimiting without a length prefix. Using
/// `F(2)=1, F(3)=2, F(4)=3, …`, the codeword for `n` is `m` bits where `m` is the
/// largest index with `F(m) <= n`.
///
/// Growth is `~1.44·log2(n)` bits against LEB128's `~1.14·log2(n)` **rounded up
/// to whole bytes**. Fibonacci therefore wins only while the byte rounding
/// dominates — small values — and loses once it does not. Which regime the real
/// data is in is the measurement, not an assumption.
fn fib_bits(n: u64) -> u32 {
    debug_assert!(n >= 1);
    let (mut a, mut b) = (1u64, 2u64); // F(2), F(3)
    let mut m = 2u32;
    while b <= n {
        let next = a + b;
        a = b;
        b = next;
        m += 1;
    }
    m
}

/// Bits an **Elias gamma** codeword for `n >= 1` occupies: `2·floor(log2 n) + 1`.
///
/// Present as a control. Without a second bit-level code, "Fibonacci lost" and
/// "bit-level coding lost" are indistinguishable, and only one of those is a
/// finding about Fibonacci.
fn gamma_bits(n: u64) -> u32 {
    debug_assert!(n >= 1);
    2 * n.ilog2() + 1
}

/// Order-0 Shannon entropy of a histogram, in bits per symbol.
///
/// The floor any prefix code over these symbols can reach without modelling
/// context. Reported so a gap between the best code and the floor is visible as
/// headroom rather than mistaken for optimality.
fn entropy_bits(hist: &HashMap<u64, u64>) -> f64 {
    let total: u64 = hist.values().sum();
    if total == 0 {
        return 0.0;
    }
    let t = total as f64;
    -hist
        .values()
        .map(|&c| {
            let p = c as f64 / t;
            p * p.log2()
        })
        .sum::<f64>()
}

/// Total bits a histogram costs under each code, plus its entropy floor.
fn code_costs(hist: &HashMap<u64, u64>) -> (u64, u64, u64, f64, u64) {
    let mut leb = 0u64;
    let mut fib = 0u64;
    let mut gam = 0u64;
    let mut n = 0u64;
    for (&v, &c) in hist {
        // Both bit codes need `n >= 1`; the streams contain 0, so shift.
        let s = v + 1;
        leb += 8 * varint_len(v) as u64 * c;
        fib += u64::from(fib_bits(s)) * c;
        gam += u64::from(gamma_bits(s)) * c;
        n += c;
    }
    (leb, fib, gam, entropy_bits(hist) * n as f64, n)
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
    /// Column 7's value streams, as histograms rather than lists — the entropy
    /// floor needs the distribution and nothing needs the order.
    hist_len: HashMap<u64, u64>,
    hist_d1: HashMap<u64, u64>,
    hist_d2: HashMap<u64, u64>,
    /// Column 8 — the curve-ruler premise: vertices covered by a maximal run of
    /// CONSTANT quantised turn (a straight or a circular arc), strict and +/-1
    /// quantum, and by a run of constant SECOND difference (a clothoid).
    run_vertices: u64,
    /// Coverage by runs at each floor. 3 is BELOW the meaningful floor and is
    /// kept only to show what a naive threshold would have claimed.
    run_cov3_strict: u64,
    run_cov17_strict: u64,
    run_cov21_strict: u64,
    run_cov17_tol: u64,
    run_cov17_clothoid: u64,
    runs_strict: u64,
    /// Longest constant-turn run seen in the class, in vertices.
    run_max: u64,
    /// Chain lengths, so "no run reached 17" can be told apart from "no chain
    /// was even 17 long" — two very different verdicts.
    chain_len: Vec<u64>,
    /// Column 9 — fit error, metres, for chains long enough to fit (>= 4 pts).
    err_bezier: Vec<f64>,
    err_clothoid: Vec<f64>,
    /// The same fit measured as distance to the CURVE — comparable to Bezier.
    err_clothoid_fair: Vec<f64>,
    err_circle: Vec<f64>,
    /// Chains offered to the fits, and vertices they speak for — so a pass rate
    /// is never quoted without saying how much of the class it covers.
    fitted: u64,
    fitted_vertices: u64,
    /// Column 10 — the RENDERING budget: vertices a polyline uploads today
    /// against control points an equivalent cubic path would upload.
    poly_vertices: u64,
    ctrl_points: u64,
    /// The same two counts once chains are split at z=16 tile boundaries —
    /// which the wire format forces, because the offsets are tile-local.
    poly_vertices_tiled: u64,
    ctrl_points_tiled: u64,
    tiles_touched: u64,
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

    // ── Column 7's streams. ──
    //
    // The SECOND difference is the interesting one and it is free: encoding
    // `q[i] - q[i-1]` instead of `q[i]` reconstructs the same quantised
    // headings, so the drift measured above is unchanged — only the coding
    // differs. A smooth boundary changes its curvature slowly, so the second
    // difference should concentrate near zero where a bit-level code is cheap.
    for l in &len {
        *acc.hist_len.entry((l * 100.0).round() as u64).or_insert(0) += 1;
    }
    for &k in &qd {
        *acc.hist_d1.entry(zigzag(k)).or_insert(0) += 1;
    }
    for w in qd.windows(2) {
        *acc.hist_d2.entry(zigzag(w[1] - w[0])).or_insert(0) += 1;
    }

    // ── Column 8: does this chain ride a template? ──
    //
    // helix's Curve-Ruler Principle says a curve is fixed by its endpoints ON a
    // template — so the question is not "how few bits per vertex" but "do the
    // vertices lie on a regenerable arc at all". A run of CONSTANT quantised
    // turn is exactly a straight (0) or a circular arc (k), and a run of
    // constant SECOND difference is a clothoid — which is what a road designed
    // to RAL/RAS-L is built from. Runs of >= 3 are counted because two vertices
    // define a stride trivially and would score every chain alike.
    acc.run_vertices += qd.len() as u64;
    acc.chain_len.push(qd.len() as u64);

    // ── Column 9: which curve family carries this chain, at P2's bar. ──
    if let Some(e) = fit_cubic_bezier(pts) {
        acc.fitted += 1;
        acc.fitted_vertices += n as u64;
        acc.err_bezier.push(e);
        if let Some((drift, fair)) = fit_clothoid(pts) {
            acc.err_clothoid.push(drift);
            acc.err_clothoid_fair.push(fair);
        }
        if let Some(c) = fit_circle(pts) {
            acc.err_circle.push(c);
        }
        // A joined cubic path of S segments costs 3S+1 control points, because
        // consecutive segments share an endpoint. Counting 4S would inflate the
        // Bezier side by a third and flatter the comparison.
        let segs = bezier_segments(pts, Z24_CELL_M) as u64;
        acc.poly_vertices += n as u64;
        acc.ctrl_points += 3 * segs + 1;

        // …and the same two counts once the WIRE FORMAT is honoured. Offsets are
        // tile-local, so a chain that leaves its tile is split there: each piece
        // carries its own tile id and repeats the boundary vertex. Splitting is
        // at the vertex where the tile changes — not at the exact crossing
        // point, which would need interpolation; that makes this a floor on the
        // tiled cost, and it is a floor for BOTH forms equally.
        let tile_of = |c: TileXy| (c.x >> (32 - TILE_Z), c.y_xyz >> (32 - TILE_Z));
        let mut cuts: Vec<usize> = vec![0];
        let mut prev = tile_of(tms::point_to_cell(ll[0].1, ll[0].0));
        let mut distinct = 1u64;
        for (i, &(la, lo)) in ll.iter().enumerate().skip(1) {
            let t = tile_of(tms::point_to_cell(lo, la));
            if t != prev {
                cuts.push(i);
                distinct += 1;
                prev = t;
            }
        }
        cuts.push(n - 1);
        acc.tiles_touched += distinct;
        for w in cuts.windows(2) {
            let piece = &pts[w[0]..=w[1]];
            if piece.len() < 2 {
                continue;
            }
            acc.poly_vertices_tiled += piece.len() as u64;
            acc.ctrl_points_tiled += 3 * bezier_segments(piece, Z24_CELL_M) as u64 + 1;
        }
    }
    let mut i = 0usize;
    while i < qd.len() {
        let mut j = i + 1;
        while j < qd.len() && qd[j] == qd[i] {
            j += 1;
        }
        let n = j - i;
        acc.runs_strict += 1;
        acc.run_max = acc.run_max.max(n as u64);
        if n >= 3 {
            acc.run_cov3_strict += n as u64;
        }
        if n >= MEANINGFUL_RUN {
            acc.run_cov17_strict += n as u64;
        }
        if n >= LONG_RUN {
            acc.run_cov21_strict += n as u64;
        }
        i = j;
    }
    let mut i = 0usize;
    while i < qd.len() {
        let mut j = i + 1;
        while j < qd.len() && (qd[j] - qd[i]).abs() <= 1 {
            j += 1;
        }
        let n = j - i;
        if n >= MEANINGFUL_RUN {
            acc.run_cov17_tol += n as u64;
        }
        i = j;
    }
    if qd.len() >= 2 {
        let dd: Vec<i64> = qd.windows(2).map(|w| w[1] - w[0]).collect();
        let mut i = 0usize;
        while i < dd.len() {
            let mut j = i + 1;
            while j < dd.len() && dd[j] == dd[i] {
                j += 1;
            }
            let n = j - i;
            if n >= MEANINGFUL_RUN {
                acc.run_cov17_clothoid += n as u64;
            }
            i = j;
        }
    }

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
    let mut by_class: HashMap<u16, Acc> = HashMap::new();
    // Undirected edge → the class bits of every way that uses it.
    let mut edge_use: HashMap<(i64, i64), u16> = HashMap::with_capacity(6_000_000);
    // Per areal way: its class and its edges, so column 5 can attribute after.
    let mut areal_edges: Vec<(u16, Vec<(i64, i64)>)> = Vec::with_capacity(100_000);
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
            // Pedestrian nodes feed column 6 and are collected for EVERY foot
            // highway; the way itself then falls through to `classify`, which
            // keeps the designed road classes and drops the rest. An early
            // return here is what made the road arm dead code on the first run.
            if let Some(hw) = tags.iter().find(|(k, _)| *k == "highway").map(|(_, v)| *v) {
                if FOOT_HIGHWAY.contains(&hw) {
                    for id in w.refs() {
                        if let Some(&(la, lo)) = coords.get(&id) {
                            foot_pts.push(frame.xy(la, lo));
                        }
                    }
                }
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
    let mut shared: HashMap<u16, (u64, u64, HashMap<u16, u64>)> = HashMap::new();
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
    let order = [
        C_BUILDING,
        C_ROAD,
        C_ROUNDABOUT,
        C_RESID,
        C_WATER,
        C_WOOD,
        C_GREEN,
        C_LINEAR_WATER,
    ];
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

    println!("\n(vii) bit-level codes on the real value streams — bits per symbol");
    println!("  LEB128 is byte-granular: a value of 3 costs a full 8 bits. Fibonacci");
    println!("  (Zeckendorf) and Elias gamma are bit-granular. gamma is the CONTROL, so");
    println!("  \"Fibonacci lost\" cannot be confused with \"bit-level coding lost\".");
    println!("  entropy = the order-0 floor; a gap to it is headroom, not optimality.");
    println!(
        "{:<14} {:<10} {:>10} {:>8} {:>8} {:>8} {:>9}",
        "class", "stream", "symbols", "leb128", "fib", "gamma", "entropy"
    );
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        for (name, hist) in [
            ("length", &a.hist_len),
            ("d1 head", &a.hist_d1),
            ("d2 head", &a.hist_d2),
        ] {
            let (leb, fib, gam, ent, n) = code_costs(hist);
            if n == 0 {
                continue;
            }
            let per = |v: u64| v as f64 / n as f64;
            println!(
                "{:<14} {:<10} {:>10} {:>8.2} {:>8.2} {:>8.2} {:>9.2}",
                class_name(bit),
                name,
                n,
                per(leb),
                per(fib),
                per(gam),
                ent / n as f64
            );
        }
    }

    println!("\n(viii) does the chain ride a template? — helix's curve-ruler premise");
    println!("  a run of CONSTANT quantised turn is a straight or a circular arc; a run of");
    println!("  constant SECOND difference is a clothoid. The stride-4-over-17 walk only");
    println!("  completes its permutation after 17 steps, so >=17 is the floor at which a");
    println!("  run can mean anything; >=3 is printed only to show what a naive floor claims.");
    println!("  \"max\" and \"med len\" separate \"no run reached 17\" from \"no CHAIN did\".");
    println!(
        "{:<16} {:>9} {:>8} {:>8} {:>8} {:>8} {:>9} {:>7} {:>8}",
        "class", "vertices", ">=3", ">=17", ">=21", "+-1 17", "cloth 17", "max", "med len"
    );
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        if a.run_vertices == 0 {
            continue;
        }
        let mut cl = a.chain_len.clone();
        cl.sort_unstable();
        let med = cl[cl.len() / 2];
        println!(
            "{:<16} {:>9} {:>7.1}% {:>7.1}% {:>7.1}% {:>7.1}% {:>8.1}% {:>7} {:>8}",
            class_name(bit),
            a.run_vertices,
            pct(a.run_cov3_strict, a.run_vertices),
            pct(a.run_cov17_strict, a.run_vertices),
            pct(a.run_cov21_strict, a.run_vertices),
            pct(a.run_cov17_tol, a.run_vertices),
            pct(a.run_cov17_clothoid, a.run_vertices),
            a.run_max,
            med,
        );
    }

    println!("\n(ix) which curve family carries the chain, at P2's bar (1 z=24 cell)");
    println!("  cubic Bezier is the EVALUATION form; the clothoid (k0, k1, L) is the storage");
    println!("  form the shape hypothesis implies. Circle is reported only where the class is");
    println!("  genuinely conic — junction=roundabout is TAGGED, so it is read, not guessed.");
    println!("  bezier error = TRUE distance to the curve (scan + ternary refine). Measuring");
    println!("  it at the point's own parameter inflates it ~27x and was this probe's first");
    println!("  version. The clothoid is now reported BOTH ways from ONE fit: \"cloth fair\" is");
    println!("  distance to the reconstructed curve — the SAME metric as bezier, so the two");
    println!("  columns are finally comparable — and \"cloth drift\" is position at matched arc");
    println!("  length, which additionally carries the chain's accumulated slip. A gap between");
    println!("  them means the shape is right and the travel along it is off.");
    println!(
        "{:<14} {:>7} {:>6} {:>8} {:>8} {:>9} {:>10} {:>10} {:>9} {:>8}",
        "class",
        "chains",
        "cover",
        "bez p50",
        "bez p95",
        "bez<1.69",
        "cloth fair",
        "fair<1.69",
        "cloth drift",
        "circ p95"
    );
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        if a.fitted == 0 {
            continue;
        }
        let mut b = a.err_bezier.clone();
        b.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let mut c = a.err_clothoid.clone();
        c.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let mut r = a.err_circle.clone();
        r.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let mut cf = a.err_clothoid_fair.clone();
        cf.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let pass = b.iter().filter(|&&e| e < Z24_CELL_M).count() as u64;
        let pass_fair = cf.iter().filter(|&&e| e < Z24_CELL_M).count() as u64;
        println!(
            "{:<14} {:>7} {:>5.1}% {:>7.3}m {:>7.3}m {:>8.1}% {:>9.3}m {:>9.1}% {:>8.3}m {:>7.3}m",
            class_name(bit),
            a.fitted,
            pct(a.fitted_vertices, a.nodes),
            q(&b, 0.5),
            q(&b, 0.95),
            pct(pass, a.fitted),
            q(&cf, 0.95),
            pct(pass_fair, cf.len() as u64),
            q(&c, 0.95),
            q(&r, 0.95),
        );
    }
    println!("  \"cover\" = share of the class's vertices living on a chain long enough to fit.");

    println!("\n(x) the RENDERING budget — what crosses the wire, not what fits");
    println!("  f32 cannot carry a global z=32 coordinate: a 24-bit mantissa loses 8 bits =");
    println!("  256 cells x 9.33 mm = ~2.39 m at the equator, OVER P2's 1.69 m bar. (The cell");
    println!("  is the world width over 2^32; the 6.59 mm in tms.rs is the ROUND-TRIP error, a");
    println!("  different quantity — reading it as a cell width understates this as ~1.7 m.)");
    println!("  So the wire is a tile id + u16");
    println!("  tile-local offsets, z=16 tiles (~372 m at Berlin) — and a chain that leaves");
    println!("  its tile must be SPLIT there. The tiled columns price that; the untiled ones");
    println!("  are what a naive count would have claimed.");
    println!("  a polyline uploads N vertices; a joined cubic path uploads 3S+1 control");
    println!("  points for S segments subdivided until each clears P2's bar. WebGL has no");
    println!("  tessellation stage, so the curve is expanded on the CPU or per-vertex from");
    println!("  control points — the win is bandwidth and vertex count, never rasterisation.");
    println!(
        "{:<14} {:>10} {:>10} {:>7} {:>10} {:>10} {:>7} {:>7}",
        "class", "poly", "ctrl", "ratio", "poly tiled", "ctrl tiled", "ratio", "tiles"
    );
    for bit in order {
        let Some(a) = by_class.get(&bit) else {
            continue;
        };
        if a.poly_vertices == 0 {
            continue;
        }
        println!(
            "{:<14} {:>10} {:>10} {:>6.2}x {:>10} {:>10} {:>6.2}x {:>7.2}",
            class_name(bit),
            a.poly_vertices,
            a.ctrl_points,
            a.poly_vertices as f64 / a.ctrl_points as f64,
            a.poly_vertices_tiled,
            a.ctrl_points_tiled,
            a.poly_vertices_tiled as f64 / a.ctrl_points_tiled.max(1) as f64,
            a.tiles_touched as f64 / a.fitted as f64,
        );
    }
    println!("  ratio > 1 means the curve form uploads LESS. Below 1 it uploads MORE.");
    println!("  both forms cost {WIRE_POINT_B} B per point (two u16 offsets), so the ratio is bytes too.");

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
        let mut ps: Vec<(&u16, &u64)> = partners.iter().collect();
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
    // These exercise the geometry that moved to `osm_soa_bake::curve`. They stay
    // here for now because moving them is a separate diff; the honest note is
    // that a test should live with the code it tests, so this is debt, not a
    // design.
    use osm_soa_bake::curve::{max_dist_to_polyline, CLOTHOID_SUB};

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

    /// Points ON the clothoid `theta(s) = k0*s + k1*s^2/2`, integrated finely.
    ///
    /// The first version of these fixtures used ONE midpoint step per emitted
    /// point — the same coarse rule the old reconstruction used, so fixture and
    /// reconstruction shared their quadrature error and the tests passed by
    /// agreeing on the same approximation. Sub-stepping the reconstruction
    /// exposed that: the finer integral is *more* correct and therefore differs
    /// *more* from a coarse fixture. Generating the fixture finely removes the
    /// shared error instead of loosening the bound to hide it.
    fn clothoid_pts(k0: f64, k1: f64, l: f64, n: usize) -> Vec<(f64, f64)> {
        const FINE: usize = 64;
        let mut pts = vec![(0.0, 0.0)];
        let (mut x, mut y) = (0.0f64, 0.0f64);
        let ds = l / n as f64;
        let h = ds / FINE as f64;
        for i in 0..n {
            for k in 0..FINE {
                let sm = i as f64 * ds + (k as f64 + 0.5) * h;
                let th = k0 * sm + 0.5 * k1 * sm * sm;
                x += h * th.cos();
                y += h * th.sin();
            }
            let _ = i;
            pts.push((x, y));
        }
        pts
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

    /// A circular arc of radius `r` spanning `sweep` radians, `n` samples.
    fn arc(r: f64, sweep: f64, n: usize) -> Vec<(f64, f64)> {
        (0..=n)
            .map(|i| {
                let a = sweep * i as f64 / n as f64;
                (r * a.cos(), r * a.sin())
            })
            .collect()
    }

    #[test]
    fn the_tile_zoom_is_the_one_a_u16_offset_actually_addresses() {
        // The wire format's arithmetic, pinned. A u16 offset spans 16 bits, so
        // it can only reach z=32 precision from a z=16 tile — off-by-one here
        // would silently change both the split count and the precision claim.
        assert_eq!(
            32 - TILE_Z,
            16,
            "u16 offset must cover exactly the low bits"
        );
        assert_eq!(WIRE_POINT_B, 4, "two u16 offsets");
        // …and the f32 figure the whole tile-local decision rests on: 24-bit
        // mantissa against a 32-bit coordinate loses 8 bits = 256 cells.
        let cells_lost = 1u64 << (32 - 24);
        assert_eq!(cells_lost, 256);
        // The z=32 cell at the equator is the world width over 2^32 = 9.33 mm.
        // NOT 6.59 mm: `tms.rs` reports 6.59 mm as the ROUND-TRIP error (exact
        // lon/lat -> tile -> centre -> lon/lat), which is a different quantity
        // and about a factor sqrt(2) smaller. Reading it as a cell width puts
        // the f32 loss at ~1.7 m, i.e. exactly AT P2's bar; the true cell puts
        // it at ~2.39 m, i.e. OVER the bar. Same conclusion, stronger.
        let cell_m = 40_075_017.0 / 4_294_967_296.0;
        assert!(
            (0.0093..0.0094).contains(&cell_m),
            "z=32 cell at the equator is 9.33 mm (got {cell_m})"
        );
        let metres = cell_m * cells_lost as f64;
        assert!(
            metres > Z24_CELL_M,
            "f32 loss ({metres} m) must EXCEED P2's {Z24_CELL_M} m bar, not merely reach it"
        );
        assert!(
            (2.3..2.5).contains(&metres),
            "…and it is ~2.39 m (got {metres})"
        );
    }

    #[test]
    fn subdivision_costs_one_segment_on_a_shallow_arc_and_more_on_a_long_one() {
        // Column 10's mechanism, two-sided. A gentle arc must need ONE cubic; a
        // full circle must need several, or the subdivision is not adapting and
        // the ratio column is a constant dressed up as a measurement.
        let gentle = arc(500.0, 0.3, 40);
        assert_eq!(bezier_segments(&gentle, Z24_CELL_M), 1);

        let full = arc(50.0, std::f64::consts::TAU, 64);
        let s = bezier_segments(&full, Z24_CELL_M);
        assert!(s >= 2, "a closed circle cannot be one cubic (got {s})");

        // …and a tighter bar must cost MORE segments, or the tolerance is inert.
        let loose = bezier_segments(&arc(50.0, std::f64::consts::PI, 64), 1.69);
        let tight = bezier_segments(&arc(50.0, std::f64::consts::PI, 64), 0.001);
        assert!(
            tight > loose,
            "tightening the bar must cost segments ({tight} vs {loose})"
        );
    }

    #[test]
    fn a_short_chain_costs_a_segment_rather_than_nothing() {
        // The trap that would make the ratio column lie in the Bezier's favour:
        // chains too short to fit are exactly where a curve form WASTES points,
        // so they must cost one segment, not zero.
        assert_eq!(
            bezier_segments(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.5)], 1.69),
            1
        );
    }

    #[test]
    fn the_cubic_hits_the_published_quarter_circle_error() {
        // The strongest falsifier available: a single cubic Bezier approximating
        // a 90 deg arc is known to sit at ~2.7e-4 of the radius. Pinning against
        // a PUBLISHED constant checks the fit against the literature rather than
        // against itself — a fit that silently returned its own residual would
        // pass a self-consistency test and fail this one.
        for r in [10.0, 100.0, 1000.0] {
            let e = fit_cubic_bezier(&arc(r, std::f64::consts::FRAC_PI_2, 64)).unwrap();
            assert!(
                e < 3.5e-4 * r,
                "quarter circle r={r}: {e} exceeds the ~2.7e-4*r figure"
            );
            assert!(e > 0.0, "…but a cubic is NOT exact on a circle");
        }
    }

    #[test]
    fn a_full_circle_defeats_one_cubic_and_the_circle_fit_nails_it() {
        // Why the roundabout row reads the way it does, two-sided. A closed
        // conic cannot be one cubic at any degree — the standard construction
        // needs four. So that row argues for RECOGNISING the conic, not
        // necessarily for a rational Bezier, and the test says both halves.
        let full = arc(50.0, std::f64::consts::TAU, 64);
        let bez = fit_cubic_bezier(&full).unwrap();
        let cir = fit_circle(&full).unwrap();
        assert!(bez > 10.0, "one cubic cannot close a circle (got {bez})");
        assert!(
            cir < 1e-6,
            "the circle fit is exact on a circle (got {cir})"
        );
    }

    #[test]
    fn the_fair_metric_ignores_slip_along_the_curve_where_drift_does_not() {
        // THE point of the fair metric, isolated from any fit. Points that lie
        // exactly ON a polyline but at shifted positions along it are distance
        // ZERO from the curve, while a matched-index comparison would report the
        // shift. If this collapsed, the two columns would still be measuring
        // different things under one name — the exact defect being repaired.
        let poly: Vec<(f64, f64)> = (0..=160).map(|i| (i as f64 * 0.5, 0.0)).collect();
        // Same line, offset 7 m ALONG it — pure slip, no shape error.
        // Inside the polyline's extent (0..80 m) — points past its END are
        // legitimately far from it, which is a fact about the fixture, not the
        // metric, and the first version of this test tripped over exactly that.
        let slipped: Vec<(f64, f64)> = (0..=8).map(|i| (i as f64 * 8.0 + 7.0, 0.0)).collect();
        let fair = max_dist_to_polyline(&slipped, &poly, CLOTHOID_SUB);
        assert!(
            fair < 1e-9,
            "slip along the curve is not shape error (got {fair})"
        );

        // …and the twin: the same points lifted 3 m OFF the line must be caught.
        // Without this half, a metric that returned zero unconditionally passes.
        let offset: Vec<(f64, f64)> = slipped.iter().map(|&(x, _)| (x, 3.0)).collect();
        let off = max_dist_to_polyline(&offset, &poly, CLOTHOID_SUB);
        assert!(
            (off - 3.0).abs() < 1e-9,
            "real deviation must survive (got {off})"
        );
    }

    #[test]
    fn the_fair_metric_measures_to_segments_not_to_vertices() {
        // A point halfway between two coarse polyline vertices is ON the line,
        // so its distance is zero. Measuring to the nearest VERTEX would report
        // half the sampling step and charge the reconstruction for its own
        // density — which would scale with CLOTHOID_SUB rather than with shape.
        let poly = vec![(0.0, 0.0), (100.0, 0.0)];
        let mid = vec![(50.0, 0.0)];
        assert!(max_dist_to_polyline(&mid, &poly, 1) < 1e-12);
    }

    #[test]
    fn the_two_kappa_metrics_barely_differ_and_that_is_the_finding() {
        // Two-sided on the pair. A genuine clothoid: both near zero, so the fair
        // metric is not merely permissive. A CIRCLE sampled unevenly: the shape
        // is exactly right (fair small) while the arc-length reconstruction can
        // only be as good as the fit — so fair must be no WORSE than drift, and
        // that ordering is the whole claim.
        let pts = clothoid_pts(0.0, 5e-5, 100.0, 50);
        let (drift, fair) = fit_clothoid(&pts).unwrap();
        assert!(
            drift < 0.01 && fair < 0.01,
            "a real clothoid fits both ways"
        );

        // The case the whole change exists for, and the FIRST version of this
        // test did not actually require it: `fair <= drift` is satisfied
        // trivially by `fair = drift`, so collapsing the two passed. Verified by
        // breaking it. This half now demands STRICT divergence.
        //
        // A coarsely sampled arc produces it by construction: the fit sees
        // CHORD length where the truth is ARC length, so the reconstruction
        // lags progressively along a curve it otherwise traces correctly.
        // Shape right, travel wrong — drift large, fair small.
        let uneven: Vec<(f64, f64)> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                let w = t + 0.25 * (std::f64::consts::TAU * t).sin() / std::f64::consts::TAU;
                let a = std::f64::consts::PI * w;
                (50.0 * a.cos(), 50.0 * a.sin())
            })
            .collect();
        let (d2, f2) = fit_clothoid(&uneven).unwrap();
        assert!(
            f2 <= d2 + 1e-9,
            "distance to the curve can never exceed distance at matched arc length \
             (fair {f2}, drift {d2})"
        );
        // …and they come out NEARLY EQUAL, which is the measured answer rather
        // than a weak assertion. The first version of this test demanded
        // `drift > 3*fair` and failed — correctly.
        //
        // The reason is structural: the reconstruction advances by the TRUE
        // chord lengths and only turns by the fitted heading, so a fit error
        // displaces the curve LATERALLY, never along itself. Longitudinal slip
        // comes solely from the chord-vs-arc gap, which grows monotonically and
        // therefore peaks at the chain's END — where the reconstruction stops,
        // so the nearest curve point to the last vertex IS its endpoint and the
        // two metrics coincide exactly at the worst point.
        //
        // Consequence, stated so it is not mistaken for a coverage hole:
        // collapsing `fair` onto `drift` would NOT fail these tests, because
        // for this scheme it is very nearly correct. The distinction is pinned
        // here as an equality, and this assertion fires if a future change makes
        // them diverge — which would mean the argument above no longer holds.
        assert!(
            (d2 - f2).abs() < 0.05 * d2.max(1e-9),
            "the two metrics agree to within 5% for this reconstruction \
             (drift {d2}, fair {f2}) — if they now diverge, re-derive why"
        );
    }

    #[test]
    fn the_window_is_an_upper_bound_not_a_shortcut() {
        // The honesty claim behind CLOTHOID_WINDOW: searching less of the
        // polyline can only return a LARGER minimum, never a smaller one. A
        // window that accidentally searched everything would make this test
        // vacuous, so it is built so the true nearest segment lies OUTSIDE the
        // window and the windowed answer is provably worse.
        let poly: Vec<(f64, f64)> = (0..=200).map(|i| (i as f64, 0.0)).collect();
        // One point whose nearest polyline segment is far along the arc.
        let far = vec![(190.0, 0.0)];
        let windowed = max_dist_to_polyline(&far, &poly, 1);
        let full = {
            let mut best: f64 = f64::INFINITY;
            for w in poly.windows(2) {
                let (a, b) = (w[0], w[1]);
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let len2 = dx * dx + dy * dy;
                let t = (((far[0].0 - a.0) * dx + (far[0].1 - a.1) * dy) / len2).clamp(0.0, 1.0);
                let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
                best = best.min(((far[0].0 - qx).powi(2) + (far[0].1 - qy).powi(2)).sqrt());
            }
            best
        };
        assert!(
            windowed >= full - 1e-9,
            "the window must never UNDER-report (windowed {windowed}, full {full})"
        );
        assert!(
            windowed > full,
            "…and here it genuinely over-reports, so the bound is live"
        );
    }

    #[test]
    fn the_fit_uses_chord_length_for_arc_length_and_that_shows_on_tight_curves() {
        // An honest bound on the fit, found by picking a bad fixture. The fit
        // treats cumulative CHORD length as the arc-length coordinate, because
        // chords are what the data gives. Chord < arc by (ds)^2/(24 R^2)
        // relatively, so the error is invisible at road curvature and real at
        // spiral curvature. Stated rather than avoided.
        let road = clothoid_pts(0.0, 5e-5, 100.0, 50); // R_min 200 m
        let (road_drift, _) = fit_clothoid(&road).unwrap();
        assert!(
            road_drift < 0.01,
            "road curvature is unaffected (got {road_drift})"
        );

        let spiral = clothoid_pts(0.0, 0.002, 200.0, 200); // R_min 2.5 m, 6.4 turns
        let (spiral_drift, _) = fit_clothoid(&spiral).unwrap();
        assert!(
            spiral_drift > 10.0 * road_drift,
            "a tight spiral must expose it, or this bound is imaginary \
             (road {road_drift}, spiral {spiral_drift})"
        );
    }

    #[test]
    fn the_clothoid_form_absorbs_the_circle_because_k1_is_zero() {
        // The reason the kappa storage form does not need a separate conic case:
        // a circle is the degenerate clothoid (curvature constant, k1 = 0). If
        // this failed, roundabouts really would need their own curve family.
        let (e, _) = fit_clothoid(&arc(50.0, std::f64::consts::TAU, 64)).unwrap();
        assert!(e < 0.05, "a circle is a clothoid with k1=0 (got {e})");
    }

    #[test]
    fn the_clothoid_beats_the_circle_where_curvature_actually_varies() {
        // Two-sided against the test above: on a REAL clothoid (curvature linear
        // in arc length) the circle fit must be materially worse, or the two
        // forms are not distinguishable and column 9 measures nothing.
        // A ROAD-LIKE transition: curvature 0 -> 1/200 m over 100 m, so the
        // heading turns 0.25 rad and R_min is 200 m. The first version used
        // k1=0.002 over 200 m — theta_end = 40 rad, i.e. SIX AND A HALF FULL
        // TURNS with R_min = 2.5 m. That is a tight spiral, not a trassierung,
        // and it exercises the chord-vs-arc approximation (below) rather than
        // the fit.
        let pts = clothoid_pts(0.0, 5e-5, 100.0, 50);
        let (cl, _) = fit_clothoid(&pts).unwrap();
        let ci = fit_circle(&pts).unwrap();
        assert!(
            cl < 0.01,
            "the clothoid fit reproduces a clothoid (got {cl})"
        );
        assert!(
            ci > 20.0 * cl.max(1e-9),
            "a circle must NOT explain a varying curvature (circle {ci}, clothoid {cl})"
        );
    }

    #[test]
    fn every_fit_is_near_exact_on_a_straight_line_and_refuses_short_chains() {
        // A straight line is the common degenerate case in this corpus (86.7% of
        // road vertices turn less than 5 deg), so a fit that blew up on it would
        // poison the road rows. And all three must REFUSE a chain too short to
        // constrain them rather than returning a flattering zero.
        let line: Vec<(f64, f64)> = (0..=20).map(|i| (i as f64 * 5.0, 0.0)).collect();
        assert!(fit_cubic_bezier(&line).unwrap() < 1e-9);
        let (d, f) = fit_clothoid(&line).unwrap();
        assert!(
            d < 1e-9 && f < 1e-9,
            "both metrics vanish on a straight line"
        );

        let short = vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)];
        assert!(fit_cubic_bezier(&short).is_none());
        assert!(fit_clothoid(&short).is_none());
        assert!(fit_circle(&short).is_none());
    }

    #[test]
    fn fibonacci_codeword_lengths_match_the_known_short_ones() {
        // Pinned against hand-derivable values, because a fencepost here would
        // shift every number in column 7 by a constant and still look plausible.
        // F(2)=1, F(3)=2, F(4)=3, F(5)=5, F(6)=8, F(7)=13.
        assert_eq!(fib_bits(1), 2, "1 -> 11");
        assert_eq!(fib_bits(2), 3, "2 -> 011");
        assert_eq!(fib_bits(3), 4, "3 -> 0011");
        assert_eq!(fib_bits(4), 4, "4 = 3+1 -> 1011");
        assert_eq!(fib_bits(7), 5, "7 = 5+2 -> 01011");
        assert_eq!(fib_bits(12), 6, "12 = 8+3+1 -> 100011");
        assert_eq!(fib_bits(13), 7, "13 = F(7) opens a new position");
        // Monotone and never shorter than 2 bits.
        for n in 1..2000u64 {
            assert!(fib_bits(n) >= 2);
            assert!(fib_bits(n) <= fib_bits(n + 1));
        }
    }

    #[test]
    fn the_bit_codes_beat_leb128_only_on_small_values_and_lose_on_large() {
        // The whole premise of column 7, two-sided. If Fibonacci were uniformly
        // better the column would be a foregone conclusion; if uniformly worse
        // it would be pointless. It is neither, and the crossover is why the
        // real distribution has to be measured rather than assumed.
        let small = fib_bits(3);
        assert!(small < 8, "3 costs {small} bits vs LEB128's 8");
        let large = fib_bits(2000);
        assert!(
            large > 8 * varint_len(2000) as u32,
            "2000 costs {large} bits vs LEB128's {}",
            8 * varint_len(2000)
        );
        assert!(gamma_bits(3) < 8, "the control agrees on small values");
    }

    #[test]
    fn entropy_is_zero_for_one_symbol_and_one_bit_for_a_fair_coin() {
        // Guards the floor column: a bug making entropy always 0 would report
        // every code as infinitely wasteful, and one making it constant would
        // hide real headroom.
        let mut one = HashMap::new();
        one.insert(7u64, 100u64);
        assert!(entropy_bits(&one).abs() < 1e-12, "no surprise, no bits");

        let mut coin = HashMap::new();
        coin.insert(0u64, 50u64);
        coin.insert(1u64, 50u64);
        assert!((entropy_bits(&coin) - 1.0).abs() < 1e-12, "one fair bit");

        assert!(entropy_bits(&HashMap::new()).abs() < 1e-12, "empty is zero");
    }

    #[test]
    fn the_second_difference_of_a_smooth_ring_concentrates_at_zero() {
        // The mechanism column 7 is built to test: a constant-curvature ring
        // turns by the SAME amount every step, so its second difference is
        // zero everywhere — the regime where a bit-level code is cheapest.
        // The first difference is NOT zero, which is what makes the pair
        // informative rather than a tautology.
        let (ids, pts) = ngon(120);
        let ll = vec![(0.0, 0.0); pts.len()];
        let mut a = Acc::default();
        measure(&mut a, &ids, &pts, &ll);

        let d2_total: u64 = a.hist_d2.values().sum();
        let d2_zero = a.hist_d2.get(&0).copied().unwrap_or(0);
        assert!(
            d2_zero * 10 >= d2_total * 9,
            "at least 90% of second differences must be zero (got {d2_zero}/{d2_total})"
        );

        let d1_total: u64 = a.hist_d1.values().sum();
        let d1_zero = a.hist_d1.get(&0).copied().unwrap_or(0);
        assert!(
            d1_zero * 2 < d1_total,
            "the FIRST difference must not be mostly zero, or this proves nothing"
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
        // Roads joined the probe when the DIN claim came up, so this arm now
        // discriminates three ways rather than dropping every highway: the
        // classified network (designed to RAL/RAS-L), the residential control,
        // and everything else — which stays dropped so column 8's road rows are
        // not diluted by driveways and footpaths.
        assert_eq!(c(&[("highway", "secondary")]), Some(C_ROAD));
        assert_eq!(c(&[("highway", "primary_link")]), Some(C_ROAD));
        assert_eq!(c(&[("highway", "residential")]), Some(C_RESID));
        assert_eq!(c(&[("highway", "footway")]), None);
        assert_eq!(c(&[("highway", "service")]), None);
        // A building that also carries a highway tag is still a building: it is
        // the class P4 measured, and column 2 compares against P4's number.
        assert_eq!(
            c(&[("building", "yes"), ("highway", "secondary")]),
            Some(C_BUILDING)
        );
    }
}
