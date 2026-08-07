//! `junction_probe` — **P12**, the junction as a first-class node.
//!
//! ```text
//! junction_probe <input.osm.pbf>
//! ```
//!
//! P8 found the gate: **76.75 % of relation-referenced nodes** lose identity
//! under an encoding that keeps only junctions, and a class-blind refcount
//! invents phantom junctions at three quarters of its hits. This probe measures
//! the proposal that answers it — *give the junction its own row*:
//!
//! * a junction is a **node with its own classid**, not a by-product of a
//!   refcount;
//! * its 12-byte V3 payload carries **2 × `Signed360`** (6 B each, exactly 12 B
//!   — [`lance_graph::helix::Signed360`] is a shipped type, not a proposal) for
//!   the through-street's in/out heading;
//! * the ways then carry **only shape**, and the street numbering can start at
//!   the junction rather than at a way endpoint;
//! * the graph itself is **sparse adjacency** — CSR over junctions in Morton
//!   order — with neighbours addressed as small signed offsets in that order
//!   (`24 × i4`, −8..+7, also exactly 96 bit: the relative-pronoun form).
//!
//! Five columns, four of which can refute the scheme:
//!
//! 1. **Population and branch degree.** `2 × Signed360` is two headings. A
//!    junction of degree 4 has four. This is the arity falsifier: if the mass
//!    sits above degree 2, one facet per junction does not carry the fork.
//! 2. **Morton locality.** The `i4` form only works if a neighbour is within
//!    ±8 *ranks* of its junction in Morton order. Measured as a distribution,
//!    not asserted — Morton is a space-filling curve, and its rank distance is
//!    not its metric distance.
//! 3. **What the ways stop carrying.** Corridor vertices split into junction
//!    and shape. The saving claimed for the street side is exactly the junction
//!    share, and it is small unless it is not.
//! 4. **Relations as local slots.** If a turn restriction's `from` and `to`
//!    ways are both incident at its `via` junction, the whole relation is a
//!    pair of slot indices *at that junction* — one byte — and P8's identity
//!    problem dissolves for this class rather than being carried. If they are
//!    not incident, it does not.
//! 5. **The size arithmetic**, assembled from 1–4 rather than from a guess.
//!
//! # Error direction, stated once
//!
//! Branch degree here counts **segment ends**, so a node interior to a way
//! contributes 2 and a way endpoint contributes 1. That over-counts a node
//! where two ways of the same street meet end-to-end (a pure split: degree 2,
//! no fork) and the column reports those separately for exactly that reason.
//! Morton rank distance is computed over the junction set **only**, which is
//! the set the adjacency would index — computing it over all nodes would
//! flatter the ±8 window by inflating the ranks it spans.

use std::collections::{HashMap, HashSet};

use osm_soa_bake::tms;
use osmpbf::{Element, ElementReader};

/// Highway values that carry no routable traffic. Same list as `graph_probe`;
/// the two probes must agree on what a road is or their counts are not
/// comparable.
const NON_ROUTABLE: &[&str] = &[
    "construction",
    "proposed",
    "platform",
    "raceway",
    "elevator",
    "emergency_bay",
    "rest_area",
    "services",
    "bus_stop",
    "street_lamp",
    "traffic_signals",
    "crossing",
    "give_way",
    "stop",
    "turning_circle",
    "milestone",
    "passing_place",
];

/// The `i4` window: a signed nibble spans −8..=+7.
const I4_MIN: i64 = -8;
/// See [`I4_MIN`].
const I4_MAX: i64 = 7;

/// Bytes in a V3 facet — `classid(4) + payload(12)`.
const FACET_B: u64 = 16;
/// Bytes in one `Signed360`: `[rim.start, rim.end, rim.floor_version, polar,
/// azimuth_lo, azimuth_hi]`. Pinned by the helix crate's own size test.
const SIGNED360_B: u64 = 6;

/// A way kept for the routing graph.
struct Way {
    nodes: Vec<i64>,
    /// `true` when the way is closed; a ring's first node is its last, so its
    /// segment-end contribution must not be double-counted.
    ring: bool,
}

fn main() {
    let path = match std::env::args().nth(1) {
        Some(p) => p,
        None => {
            eprintln!("usage: junction_probe <input.osm.pbf>");
            std::process::exit(2);
        }
    };

    // ── pass 1: routable ways, and the routable refcount ──────────────────
    let mut ways: Vec<Way> = Vec::new();
    let mut routable_ref: HashMap<i64, u32> = HashMap::with_capacity(4_000_000);

    ElementReader::from_path(&path)
        .expect("open pbf")
        .for_each(|el| {
            if let Element::Way(w) = el {
                let hw = w.tags().find(|(k, _)| *k == "highway").map(|(_, v)| v);
                let Some(hw) = hw else { return };
                if NON_ROUTABLE.contains(&hw) {
                    return;
                }
                let nodes: Vec<i64> = w.refs().collect();
                if nodes.len() < 2 {
                    return;
                }
                let ring = nodes.first() == nodes.last();
                for n in &nodes {
                    *routable_ref.entry(*n).or_insert(0) += 1;
                }
                ways.push(Way { nodes, ring });
            }
        })
        .expect("read pbf");

    eprintln!("pass 1: {} routable ways", ways.len());

    // ── branch degree: segment ends per node ──────────────────────────────
    // Interior node -> 2 (a segment arrives and one leaves). Endpoint -> 1.
    // A ring's shared first/last node is one node with two segment ends, which
    // the `ring` skip below produces without a special case.
    let mut degree: HashMap<i64, u32> = HashMap::with_capacity(routable_ref.len());
    for w in &ways {
        let last = w.nodes.len() - 1;
        for (i, n) in w.nodes.iter().enumerate() {
            if w.ring && i == last {
                continue; // already counted as index 0
            }
            let ends = if !w.ring && (i == 0 || i == last) {
                1
            } else {
                2
            };
            *degree.entry(*n).or_insert(0) += ends;
        }
    }

    // A junction is a node the ROUTABLE graph sees at least twice — P8's gate.
    let junctions: HashSet<i64> = routable_ref
        .iter()
        .filter(|(_, c)| **c >= 2)
        .map(|(n, _)| *n)
        .collect();

    // ── pass 2: coordinates for the junctions, and the restriction relations ──
    let mut coord: HashMap<i64, (f64, f64)> = HashMap::with_capacity(junctions.len());
    // (via node, from way, to way)
    let mut restrictions: Vec<(i64, i64, i64)> = Vec::new();

    ElementReader::from_path(&path)
        .expect("reopen pbf")
        .for_each(|el| match el {
            Element::Node(n) => {
                if junctions.contains(&n.id()) {
                    coord.insert(n.id(), (n.lon(), n.lat()));
                }
            }
            Element::DenseNode(n) => {
                if junctions.contains(&n.id()) {
                    coord.insert(n.id(), (n.lon(), n.lat()));
                }
            }
            Element::Relation(r) => {
                let is_restriction = r
                    .tags()
                    .any(|(k, v)| k == "type" && v.starts_with("restriction"));
                if !is_restriction {
                    return;
                }
                let (mut via, mut from, mut to) = (None, None, None);
                for m in r.members() {
                    let role = m.role().unwrap_or("");
                    let is_node = m.member_type == osmpbf::RelMemberType::Node;
                    let is_way = m.member_type == osmpbf::RelMemberType::Way;
                    match role {
                        "via" if is_node => via = Some(m.member_id),
                        "from" if is_way => from = Some(m.member_id),
                        "to" if is_way => to = Some(m.member_id),
                        _ => {}
                    }
                }
                if let (Some(v), Some(f), Some(t)) = (via, from, to) {
                    restrictions.push((v, f, t));
                }
            }
            _ => {}
        })
        .expect("read pbf 2");

    eprintln!(
        "pass 2: {} junction coords, {} node-via restrictions",
        coord.len(),
        restrictions.len()
    );

    // ── (1) population and branch degree ──────────────────────────────────
    let mut deg_hist: HashMap<u32, u64> = HashMap::new();
    for j in &junctions {
        let d = degree.get(j).copied().unwrap_or(0);
        *deg_hist.entry(d.min(9)).or_insert(0) += 1;
    }
    let n_junc = junctions.len() as u64;

    println!("\n(1) the junction population and its BRANCH DEGREE");
    println!("  routable ways                 {}", ways.len());
    println!("  junctions (routable ref >= 2) {n_junc}");
    let mut degs: Vec<u32> = deg_hist.keys().copied().collect();
    degs.sort_unstable();
    let mut fits_one_facet = 0u64;
    for d in degs {
        let c = deg_hist[&d];
        let label = if d >= 9 {
            ">=9".to_string()
        } else {
            d.to_string()
        };
        println!(
            "    degree {label:>3}  {c:>9}  ({:.1}%)",
            100.0 * c as f64 / n_junc as f64
        );
        if d <= 2 {
            fits_one_facet += c;
        }
    }
    println!(
        "  degree <= 2 — what ONE facet of 2 x Signed360 carries: {fits_one_facet} ({:.1}%)",
        100.0 * fits_one_facet as f64 / n_junc as f64
    );
    println!(
        "  the rest need ceil(deg/2) facets; mean facets/junction = {:.2}",
        facets_per_junction(&junctions, &degree)
    );
    // A degree-2 "junction" is not a fork — it is a way SPLIT, the same
    // fragmentation artefact P9 removed by stitching corridors. Reporting the
    // total without this line would price rows that corridor assembly deletes.
    let forks: HashSet<i64> = junctions
        .iter()
        .filter(|j| degree.get(j).copied().unwrap_or(0) >= 3)
        .copied()
        .collect();
    println!(
        "  degree >= 2 but NOT a fork (pure way split): {} ({:.1}%) — P9's corridor",
        deg_hist.get(&2).copied().unwrap_or(0),
        100.0 * deg_hist.get(&2).copied().unwrap_or(0) as f64 / n_junc as f64
    );
    println!(
        "  stitch deletes these, leaving {} real forks / {} facets",
        forks.len(),
        facet_total(&forks, &degree)
    );

    // ── (2) Morton locality — the i4 falsifier ────────────────────────────
    // Rank the junctions along the z=32 Morton curve. The adjacency indexes
    // this order, so "within +-8" means within 8 RANKS, not 8 cells.
    let mut ranked: Vec<(u64, i64)> = coord
        .iter()
        .map(|(id, (lon, lat))| (tms::point_to_tms_morton(*lon, *lat), *id))
        .collect();
    ranked.sort_unstable();
    let rank: HashMap<i64, i64> = ranked
        .iter()
        .enumerate()
        .map(|(i, (_, id))| (*id, i as i64))
        .collect();

    // One graph edge per consecutive junction pair along a way.
    let mut edges = 0u64;
    let mut within_i4 = 0u64;
    let mut within_i8 = 0u64;
    let mut sum_abs: u128 = 0;
    for w in &ways {
        let mut prev: Option<i64> = None;
        for n in &w.nodes {
            if !junctions.contains(n) {
                continue;
            }
            if let Some(p) = prev {
                if let (Some(a), Some(b)) = (rank.get(&p), rank.get(n)) {
                    let d = b - a;
                    edges += 1;
                    sum_abs += d.unsigned_abs() as u128;
                    if (I4_MIN..=I4_MAX).contains(&d) {
                        within_i4 += 1;
                    }
                    if (-128..=127).contains(&d) {
                        within_i8 += 1;
                    }
                }
            }
            prev = Some(*n);
        }
    }

    println!("\n(2) Morton locality — can a neighbour be a SIGNED OFFSET?");
    println!("  junction-to-junction graph edges  {edges}");
    if edges > 0 {
        println!(
            "    |rank delta| within i4 (-8..+7)   {within_i4:>9}  ({:.1}%)",
            100.0 * within_i4 as f64 / edges as f64
        );
        println!(
            "    |rank delta| within i8 (-128..127) {within_i8:>9}  ({:.1}%)",
            100.0 * within_i8 as f64 / edges as f64
        );
        println!(
            "    mean |rank delta|                  {}",
            sum_abs / edges as u128
        );
    }
    println!("  a neighbour outside the window needs an ESCAPE, and the escape is");
    println!("  the full id — so the i4 form is only as cheap as this share is high.");

    // ── (3) what the ways stop carrying ───────────────────────────────────
    let mut total_v = 0u64;
    let mut junc_v = 0u64;
    for w in &ways {
        for n in &w.nodes {
            total_v += 1;
            if junctions.contains(n) {
                junc_v += 1;
            }
        }
    }
    println!("\n(3) what moves OFF the way and onto the junction row");
    println!("  way vertices, all           {total_v}");
    println!(
        "  of those, junctions         {junc_v}  ({:.1}%)  <- moves to its own row",
        100.0 * junc_v as f64 / total_v as f64
    );
    println!(
        "  shape only                  {}  ({:.1}%)  <- stays on the way",
        total_v - junc_v,
        100.0 * (total_v - junc_v) as f64 / total_v as f64
    );

    // ── (4) relations as LOCAL SLOTS at the junction ──────────────────────
    let mut way_of_node: HashMap<i64, HashSet<i64>> = HashMap::new();
    // Re-read way ids: the first pass dropped them, so do the cheap thing and
    // index incidence by way ORDER, then map restriction ids through a second
    // read. Keeping the ids in pass 1 would have been cheaper; this is honest
    // about what the probe actually knows.
    let mut way_ids: Vec<i64> = Vec::with_capacity(ways.len());
    ElementReader::from_path(&path)
        .expect("reopen pbf 3")
        .for_each(|el| {
            if let Element::Way(w) = el {
                let hw = w.tags().find(|(k, _)| *k == "highway").map(|(_, v)| v);
                let Some(hw) = hw else { return };
                if NON_ROUTABLE.contains(&hw) {
                    return;
                }
                if w.refs().count() < 2 {
                    return;
                }
                way_ids.push(w.id());
            }
        })
        .expect("read pbf 3");
    assert_eq!(
        way_ids.len(),
        ways.len(),
        "the two reads must select the same ways or the incidence map is wrong"
    );
    for (wi, w) in ways.iter().enumerate() {
        for n in &w.nodes {
            if junctions.contains(n) {
                way_of_node.entry(*n).or_default().insert(way_ids[wi]);
            }
        }
    }

    let mut local = 0u64;
    let mut via_not_junction = 0u64;
    let mut member_not_incident = 0u64;
    for (via, from, to) in &restrictions {
        if !junctions.contains(via) {
            via_not_junction += 1;
            continue;
        }
        let inc = way_of_node.get(via);
        let ok = inc.is_some_and(|s| s.contains(from) && s.contains(to));
        if ok {
            local += 1;
        } else {
            member_not_incident += 1;
        }
    }
    let n_r = restrictions.len() as u64;
    println!("\n(4) the turn restriction as a pair of LOCAL SLOTS");
    println!("  node-via restrictions            {n_r}");
    if n_r > 0 {
        println!(
            "    from AND to incident at the via  {local:>6}  ({:.1}%)  <- collapses to (in,out), ~1 byte",
            100.0 * local as f64 / n_r as f64
        );
        println!(
            "    via is not a junction            {via_not_junction:>6}  ({:.1}%)",
            100.0 * via_not_junction as f64 / n_r as f64
        );
        println!(
            "    a member is not incident         {member_not_incident:>6}  ({:.1}%)  <- still needs global ids",
            100.0 * member_not_incident as f64 / n_r as f64
        );
    }

    // ── (5) the size arithmetic ───────────────────────────────────────────
    let facets = facet_total(&junctions, &degree);
    let junction_bytes = facets * FACET_B;
    let adj_i4_bytes = edges.div_ceil(2); // one signed nibble per edge
    let adj_id_bytes = edges * 8; // the honest alternative: a full node id
    println!("\n(5) size, assembled from the four columns above");
    println!(
        "  junction rows      {facets} facets x {FACET_B} B = {:.2} MB   (2 x Signed360 = {} B fills the payload EXACTLY)",
        junction_bytes as f64 / 1e6,
        2 * SIGNED360_B
    );
    println!(
        "  adjacency, i4      {edges} edges / 2 = {:.2} MB   (only valid for the {:.1}% inside the window)",
        adj_i4_bytes as f64 / 1e6,
        if edges > 0 {
            100.0 * within_i4 as f64 / edges as f64
        } else {
            0.0
        }
    );
    println!(
        "  adjacency, full id {edges} edges x 8 B  = {:.2} MB   (the escape cost, per edge)",
        adj_id_bytes as f64 / 1e6
    );
    // The nibble is not automatically the optimum: an out-of-window neighbour
    // costs a FULL id, so the escape share multiplies by 8 B while the width
    // saving is only half a byte. Priced here rather than assumed.
    let esc4 = edges - within_i4;
    let esc8 = edges - within_i8;
    println!(
        "    i4 + escape      {:.2} MB   ({} nibbles + {} escapes x 8 B)",
        (adj_i4_bytes + esc4 * 8) as f64 / 1e6,
        edges,
        esc4
    );
    println!(
        "    i8 + escape      {:.2} MB   ({} bytes   + {} escapes x 8 B)",
        (edges + esc8 * 8) as f64 / 1e6,
        edges,
        esc8
    );
    println!(
        "  shape vertices left on the ways: {} — priced by whatever the way encoding is,",
        total_v - junc_v
    );
    println!("  and NOT by this probe. This column sizes the junction layer only.");
}

/// Facets needed for one junction: each facet carries 2 headings.
fn facets_for(deg: u32) -> u64 {
    (deg.max(1) as u64).div_ceil(2)
}

fn facet_total(junctions: &HashSet<i64>, degree: &HashMap<i64, u32>) -> u64 {
    junctions
        .iter()
        .map(|j| facets_for(degree.get(j).copied().unwrap_or(0)))
        .sum()
}

fn facets_per_junction(junctions: &HashSet<i64>, degree: &HashMap<i64, u32>) -> f64 {
    if junctions.is_empty() {
        return 0.0;
    }
    facet_total(junctions, degree) as f64 / junctions.len() as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_facet_carries_two_headings_and_a_fork_needs_more() {
        // The arity claim, two-sided: degree 2 fits one facet, degree 3 does
        // not. If `facets_for` ever returned 1 for everything this fails.
        assert_eq!(facets_for(1), 1);
        assert_eq!(facets_for(2), 1);
        assert_eq!(facets_for(3), 2, "a three-way fork does NOT fit one facet");
        assert_eq!(facets_for(4), 2);
        assert_eq!(facets_for(5), 3);
    }

    #[test]
    fn the_i4_window_is_asymmetric_and_the_bounds_are_the_real_ones() {
        // -8..+7, not -7..+7 and not -8..+8. A symmetric guess would pass a
        // sloppy test and mis-size the escape share by two codepoints.
        assert!((I4_MIN..=I4_MAX).contains(&-8));
        assert!((I4_MIN..=I4_MAX).contains(&7));
        assert!(!(I4_MIN..=I4_MAX).contains(&8));
        assert!(!(I4_MIN..=I4_MAX).contains(&-9));
        assert_eq!(I4_MAX - I4_MIN + 1, 16, "a nibble holds 16 values");
    }

    #[test]
    fn two_signed360_fill_the_v3_payload_exactly() {
        // The whole proposal rests on this: 2 x 6 B = 12 B = the content-blind
        // register. One byte either way and the scheme needs a second facet or
        // wastes the tail.
        assert_eq!(2 * SIGNED360_B, FACET_B - 4, "classid takes the first 4 B");
        assert_eq!(2 * SIGNED360_B, 12);
    }

    #[test]
    fn facet_total_counts_per_junction_not_per_node() {
        let mut deg = HashMap::new();
        deg.insert(1i64, 2u32);
        deg.insert(2i64, 3u32);
        deg.insert(3i64, 8u32);
        let js: HashSet<i64> = [1, 2, 3].into_iter().collect();
        // 1 + 2 + 4 = 7. A version summing degrees would say 13; a version
        // counting junctions would say 3.
        assert_eq!(facet_total(&js, &deg), 7);
        assert!((facets_per_junction(&js, &deg) - 7.0 / 3.0).abs() < 1e-12);
    }

    #[test]
    fn a_junction_with_no_recorded_degree_still_costs_one_facet() {
        // Defensive: a junction absent from the degree map must not make the
        // total silently smaller than the junction count.
        let deg = HashMap::new();
        let js: HashSet<i64> = [1i64, 2, 3].into_iter().collect();
        assert_eq!(facet_total(&js, &deg), 3);
    }
}
