//! `corridor_probe` — **P9**, the corridor built from connectivity, not names.
//!
//! ```text
//! corridor_probe <input.osm.pbf>
//! ```
//!
//! Three earlier columns concluded that **the way is the wrong unit**: no chain
//! reaches a 17-step template run because no chain is long enough (P6 viii), one
//! cubic carries 90.8 % of ways but the median way has ONE interior turn (P6 ix),
//! and the wire budget saves only 23 % because 3–4 vertices cannot lose to 4
//! control points (P7 x). Each pointed at the corridor.
//!
//! P8 then measured what assembling corridors **by name** would cost: a quarter
//! of Berlin's named groups and **half of Iceland's** are not one connected
//! road, so the assembly would invent 5,053 and 6,653 joins — "Rosenweg" is 83
//! separate streets. That refuted the recommendation as stated.
//!
//! This is the same idea built the only way that survives the routing gate:
//! **connectivity alone**.
//!
//! # What a corridor is here
//!
//! OSM splits ways for reasons that are not junctions — a tag changes, a name
//! changes, an extract ends. Those splits leave a node where exactly **two way
//! ENDS** meet and nothing else passes through. That node is an artefact, and
//! merging across it is lossless for both geometry and topology.
//!
//! The merge condition is therefore two counts, and both are necessary:
//!
//! - `end_count == 2` — exactly two ways *terminate* here. A node in the
//!   *interior* of a third way is not a merge point even if two ways end at it,
//!   which is why the interior reference is counted separately.
//! - `routable_ref == 2` — no third routable way references it at all.
//!
//! A node failing either is a real junction and the corridor stops. **Nothing
//! here consults a name.**
//!
//! Same-class is additionally required: a corridor spanning `primary` into
//! `residential` has no meaningful class label, and every column below is
//! reported per class. The cost of that choice is measured (a corridor that
//! *would* have continued across a class change is counted), so the constraint
//! is visible rather than assumed harmless.
//!
//! # What is re-measured
//!
//! The three findings that pointed here, on the new unit: chain length, template
//! run coverage at the stride-4-over-17 floor, and the wire budget. If the
//! corridor does not move them, the way was not the problem.

use std::collections::HashMap;

use osm_soa_bake::curve::bezier_segments;
use osm_soa_bake::geodesy::{meridional_radius, normal_radius};
use osm_soa_bake::tms::{self, TileXy};
use osmpbf::{Element, ElementReader};

/// `highway=*` values that carry no traffic of any mode.
const NON_ROUTABLE: &[&str] = &[
    "construction",
    "proposed",
    "platform",
    "raceway",
    "bus_stop",
    "street_lamp",
    "traffic_sign",
    "rest_area",
    "services",
];

/// P2's threshold, metres: one z=24 cell at its coarse end.
const Z24_CELL_M: f64 = 1.69;

/// Heading quantum for the template-run scan, degrees.
const ANGLE_QUANTUM_DEG: f64 = 0.5;

/// The shortest run that can mean anything on a stride-4-over-17 curve ruler:
/// `gcd(4,17)=1`, so the walk permutes all 17 residues only after 17 steps.
const MEANINGFUL_RUN: usize = 17;

/// Tile zoom a u16 local offset addresses, so the wire keeps full z=32.
const TILE_Z: u32 = 16;

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

/// Class buckets, coarse on purpose — the question is about chain length, not
/// about taxonomy.
fn class_of(hw: &str) -> &'static str {
    match hw {
        "motorway" | "trunk" | "primary" | "secondary" | "tertiary" | "motorway_link"
        | "trunk_link" | "primary_link" | "secondary_link" | "tertiary_link" => "classified",
        "residential" | "unclassified" | "living_street" => "residential",
        "service" => "service",
        "footway" | "path" | "steps" | "pedestrian" | "track" | "bridleway" | "corridor" => "foot",
        "cycleway" => "cycle",
        _ => "other",
    }
}

#[derive(Default, Clone)]
struct Stat {
    chains: u64,
    vertices: u64,
    /// Chain lengths in vertices, for the median.
    lens: Vec<u64>,
    /// Vertices covered by a constant-turn run of at least `MEANINGFUL_RUN`.
    run17: u64,
    turns: u64,
    /// Wire budget, tile-split.
    poly_tiled: u64,
    ctrl_tiled: u64,
}

fn pct(n: u64, d: u64) -> f64 {
    if d == 0 {
        0.0
    } else {
        n as f64 * 100.0 / d as f64
    }
}

fn median(v: &mut [u64]) -> u64 {
    if v.is_empty() {
        return 0;
    }
    v.sort_unstable();
    v[v.len() / 2]
}

fn wrap_pi(mut a: f64) -> f64 {
    while a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    while a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

/// Measure one chain of projected points into `s`.
fn measure(s: &mut Stat, ll: &[(f64, f64)], pts: &[(f64, f64)]) {
    let n = pts.len();
    if n < 2 {
        return;
    }
    s.chains += 1;
    s.vertices += n as u64;
    s.lens.push(n as u64);

    // Quantised turns, and runs of constant turn — a straight or a circular arc.
    let mut head = Vec::with_capacity(n - 1);
    for w in pts.windows(2) {
        head.push((w[1].1 - w[0].1).atan2(w[1].0 - w[0].0));
    }
    let qd: Vec<i64> = head
        .windows(2)
        .map(|w| (wrap_pi(w[1] - w[0]).to_degrees() / ANGLE_QUANTUM_DEG).round() as i64)
        .collect();
    s.turns += qd.len() as u64;
    let mut i = 0usize;
    while i < qd.len() {
        let mut j = i + 1;
        while j < qd.len() && qd[j] == qd[i] {
            j += 1;
        }
        if j - i >= MEANINGFUL_RUN {
            s.run17 += (j - i) as u64;
        }
        i = j;
    }

    // Wire budget under the tile-local format: split where the z=16 tile changes.
    let tile_of = |c: TileXy| (c.x >> (32 - TILE_Z), c.y_xyz >> (32 - TILE_Z));
    let mut cuts = vec![0usize];
    let mut prev = tile_of(tms::point_to_cell(ll[0].1, ll[0].0));
    for (i, &(la, lo)) in ll.iter().enumerate().skip(1) {
        let t = tile_of(tms::point_to_cell(lo, la));
        if t != prev {
            cuts.push(i);
            prev = t;
        }
    }
    cuts.push(n - 1);
    for w in cuts.windows(2) {
        let piece = &pts[w[0]..=w[1]];
        if piece.len() < 2 {
            continue;
        }
        s.poly_tiled += piece.len() as u64;
        s.ctrl_tiled += 3 * bezier_segments(piece, Z24_CELL_M) as u64 + 1;
    }
}

struct Way {
    nodes: Vec<i64>,
    class: &'static str,
}

/// Stitch ways into corridors at nodes where exactly two way ENDS meet and
/// nothing else passes through.
///
/// Returns the corridors as node sequences, plus how many merges were declined
/// purely because the two ways were of different classes — the measured cost of
/// the same-class constraint.
fn build_corridors(
    ways: &[Way],
    routable_ref: &HashMap<i64, u32>,
) -> (Vec<(usize, Vec<i64>)>, u64) {
    // node -> the (way, end) pairs terminating there. `end` is 0 = first, 1 = last.
    let mut ends: HashMap<i64, Vec<(usize, u8)>> = HashMap::new();
    for (i, w) in ways.iter().enumerate() {
        ends.entry(w.nodes[0]).or_default().push((i, 0));
        ends.entry(*w.nodes.last().unwrap())
            .or_default()
            .push((i, 1));
    }

    // link[way][end] = Some(other way) when this end is a merge point.
    let mut link: Vec<[Option<usize>; 2]> = vec![[None, None]; ways.len()];
    let mut class_declined = 0u64;
    for (node, list) in &ends {
        // Two ends AND no third way passing through. A closed way contributes
        // both its ends to the same node; that is a ring, not a merge point.
        if list.len() != 2 || routable_ref.get(node).copied().unwrap_or(0) != 2 {
            continue;
        }
        let (a, ea) = list[0];
        let (b, eb) = list[1];
        if a == b {
            continue;
        }
        if ways[a].class != ways[b].class {
            class_declined += 1;
            continue;
        }
        link[a][ea as usize] = Some(b);
        link[b][eb as usize] = Some(a);
    }

    // Walk maximal chains. Start from any way with a free end; whatever is left
    // afterwards is a cycle and is emitted from an arbitrary member.
    let mut used = vec![false; ways.len()];
    let mut out: Vec<(usize, Vec<i64>)> = Vec::new();

    let emit = |start: usize, used: &mut Vec<bool>, out: &mut Vec<(usize, Vec<i64>)>| {
        let mut nodes: Vec<i64> = ways[start].nodes.clone();
        used[start] = true;
        let mut count = 1usize;
        // Extend forward from the tail.
        let mut cur = start;
        let mut cur_tail_end = 1u8;
        while let Some(next) = link[cur][cur_tail_end as usize] {
            if used[next] {
                break;
            }
            let join = *nodes.last().unwrap();
            let mut seg = ways[next].nodes.clone();
            if seg[0] != join {
                seg.reverse();
            }
            debug_assert_eq!(seg[0], join, "corridors join at a shared node");
            nodes.extend_from_slice(&seg[1..]);
            used[next] = true;
            count += 1;
            cur_tail_end = u8::from(ways[next].nodes[0] == join);
            cur = next;
        }
        // Extend backward from the head.
        let mut cur = start;
        let mut cur_head_end = 0u8;
        while let Some(prev) = link[cur][cur_head_end as usize] {
            if used[prev] {
                break;
            }
            let join = nodes[0];
            let mut seg = ways[prev].nodes.clone();
            if *seg.last().unwrap() != join {
                seg.reverse();
            }
            debug_assert_eq!(
                *seg.last().unwrap(),
                join,
                "corridors join at a shared node"
            );
            let mut merged = seg[..seg.len() - 1].to_vec();
            merged.extend_from_slice(&nodes);
            nodes = merged;
            used[prev] = true;
            count += 1;
            cur_head_end = u8::from(*ways[prev].nodes.last().unwrap() == join);
            cur = prev;
        }
        out.push((count, nodes));
    };

    for i in 0..ways.len() {
        if used[i] {
            continue;
        }
        let free = link[i][0].is_none() || link[i][1].is_none();
        if free {
            emit(i, &mut used, &mut out);
        }
    }
    for i in 0..ways.len() {
        if !used[i] {
            emit(i, &mut used, &mut out);
        }
    }
    (out, class_declined)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: corridor_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(8_000_000);
    let (mut slat, mut slon) = (0.0f64, 0.0f64);
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| match el {
            Element::Node(n) => {
                coords.insert(n.id(), (n.lat(), n.lon()));
                slat += n.lat();
                slon += n.lon();
            }
            Element::DenseNode(n) => {
                coords.insert(n.id(), (n.lat(), n.lon()));
                slat += n.lat();
                slon += n.lon();
            }
            _ => {}
        })
        .expect("pass 1");
    let nn = coords.len().max(1) as f64;
    let frame = Frame::new(slat / nn, slon / nn);

    let mut routable_ref: HashMap<i64, u32> = HashMap::with_capacity(4_000_000);
    let mut ways: Vec<Way> = Vec::with_capacity(500_000);
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Way(w) = el else { return };
            let Some(hw) = w.tags().find(|(k, _)| *k == "highway").map(|(_, v)| v) else {
                return;
            };
            if NON_ROUTABLE.contains(&hw) {
                return;
            }
            let ids: Vec<i64> = w.refs().collect();
            if ids.len() < 2 {
                return;
            }
            for &i in &ids {
                *routable_ref.entry(i).or_insert(0) += 1;
            }
            ways.push(Way {
                nodes: ids,
                class: class_of(hw),
            });
        })
        .expect("pass 2");
    eprintln!("pass 2: {} routable ways", ways.len());

    let (corridors, class_declined) = build_corridors(&ways, &routable_ref);
    eprintln!("stitched into {} corridors", corridors.len());

    // Per-class stats, ways vs corridors, measured by the same code.
    let mut way_stats: HashMap<&str, Stat> = HashMap::new();
    let mut cor_stats: HashMap<&str, Stat> = HashMap::new();
    let mut class_by_first: HashMap<usize, &'static str> = HashMap::new();
    for (i, w) in ways.iter().enumerate() {
        class_by_first.insert(i, w.class);
        let ll: Vec<(f64, f64)> = w
            .nodes
            .iter()
            .filter_map(|n| coords.get(n).copied())
            .collect();
        if ll.len() != w.nodes.len() {
            continue;
        }
        let pts: Vec<(f64, f64)> = ll.iter().map(|&(a, o)| frame.xy(a, o)).collect();
        measure(way_stats.entry(w.class).or_default(), &ll, &pts);
    }
    for (ways_in, nodes) in &corridors {
        let ll: Vec<(f64, f64)> = nodes
            .iter()
            .filter_map(|n| coords.get(n).copied())
            .collect();
        if ll.len() != nodes.len() {
            continue;
        }
        let pts: Vec<(f64, f64)> = ll.iter().map(|&(a, o)| frame.xy(a, o)).collect();
        // Class of a corridor = the class of its members (they are all equal by
        // construction), read from any one of them.
        let _ = ways_in;
        let class = ways
            .iter()
            .find(|w| w.nodes[0] == nodes[0] || *w.nodes.last().unwrap() == nodes[0])
            .map_or("other", |w| w.class);
        measure(cor_stats.entry(class).or_default(), &ll, &pts);
    }

    let order = [
        "classified",
        "residential",
        "service",
        "foot",
        "cycle",
        "other",
    ];

    println!("\n(i) the unit — ways stitched into corridors by CONNECTIVITY alone");
    println!(
        "  {} routable ways -> {} corridors; {class_declined} merges declined only because the",
        ways.len(),
        corridors.len()
    );
    println!("  two ways were of different classes (the measured cost of that constraint).");
    println!(
        "{:<14} {:>9} {:>10} {:>9} {:>11} {:>9}",
        "class", "ways", "corridors", "med way", "med corr", "ratio"
    );
    for c in order {
        let (Some(w), Some(k)) = (way_stats.get(c), cor_stats.get(c)) else {
            continue;
        };
        let mut wl = w.lens.clone();
        let mut kl = k.lens.clone();
        let (mw, mk) = (median(&mut wl), median(&mut kl));
        println!(
            "{:<14} {:>9} {:>10} {:>9} {:>11} {:>8.2}x",
            c,
            w.chains,
            k.chains,
            mw,
            mk,
            mk as f64 / mw.max(1) as f64
        );
    }

    println!("\n(ii) template runs at the stride-4-over-17 floor — the finding that pointed here");
    println!(
        "{:<14} {:>12} {:>10} {:>12} {:>10}",
        "class", "way turns", "way >=17", "corr turns", "corr >=17"
    );
    for c in order {
        let (Some(w), Some(k)) = (way_stats.get(c), cor_stats.get(c)) else {
            continue;
        };
        println!(
            "{:<14} {:>12} {:>9.2}% {:>12} {:>9.2}%",
            c,
            w.turns,
            pct(w.run17, w.turns),
            k.turns,
            pct(k.run17, k.turns)
        );
    }

    println!("\n(iii) wire budget, tile-split — poly vertices against cubic control points");
    println!(
        "{:<14} {:>11} {:>11} {:>8} {:>11} {:>11} {:>8}",
        "class", "way poly", "way ctrl", "ratio", "corr poly", "corr ctrl", "ratio"
    );
    for c in order {
        let (Some(w), Some(k)) = (way_stats.get(c), cor_stats.get(c)) else {
            continue;
        };
        println!(
            "{:<14} {:>11} {:>11} {:>7.2}x {:>11} {:>11} {:>7.2}x",
            c,
            w.poly_tiled,
            w.ctrl_tiled,
            w.poly_tiled as f64 / w.ctrl_tiled.max(1) as f64,
            k.poly_tiled,
            k.ctrl_tiled,
            k.poly_tiled as f64 / k.ctrl_tiled.max(1) as f64,
        );
    }
    println!("  ratio > 1 means the curve form uploads LESS.");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn refs_of(ways: &[Way]) -> HashMap<i64, u32> {
        let mut m = HashMap::new();
        for w in ways {
            for &n in &w.nodes {
                *m.entry(n).or_insert(0) += 1;
            }
        }
        m
    }

    fn way(nodes: &[i64], class: &'static str) -> Way {
        Way {
            nodes: nodes.to_vec(),
            class,
        }
    }

    #[test]
    fn two_ways_meeting_at_a_free_end_become_one_corridor() {
        // The whole point: an OSM split that is not a junction is merged away.
        let ways = vec![way(&[1, 2, 3], "classified"), way(&[3, 4, 5], "classified")];
        let r = refs_of(&ways);
        let (cor, declined) = build_corridors(&ways, &r);
        assert_eq!(cor.len(), 1, "one corridor");
        assert_eq!(
            cor[0].1,
            vec![1, 2, 3, 4, 5],
            "joined, shared node not repeated"
        );
        assert_eq!(declined, 0);
    }

    #[test]
    fn a_third_way_at_the_node_makes_it_a_real_junction() {
        // The half that keeps this from being a name-assembly in disguise: a
        // T-junction must NOT be merged, or the corridor invents a through-road
        // where a driver actually has a choice. Without this the probe would
        // report long corridors by swallowing junctions.
        let ways = vec![
            way(&[1, 2, 3], "classified"),
            way(&[3, 4, 5], "classified"),
            way(&[3, 6, 7], "classified"),
        ];
        let r = refs_of(&ways);
        let (cor, _) = build_corridors(&ways, &r);
        assert_eq!(cor.len(), 3, "three ways, three corridors — nothing merged");
    }

    #[test]
    fn an_end_landing_in_the_interior_of_another_way_is_not_a_merge_point() {
        // `end_count == 2` alone is not enough: node 3 is the END of two ways
        // AND an interior node of a third. Merging there would drive straight
        // past a real turn-off. This is why the routable refcount is checked
        // separately from the end count.
        let ways = vec![
            way(&[1, 2, 3], "classified"),
            way(&[3, 4, 5], "classified"),
            way(&[8, 3, 9], "classified"),
        ];
        let r = refs_of(&ways);
        assert_eq!(r[&3], 3, "node 3 is referenced three times");
        let (cor, _) = build_corridors(&ways, &r);
        assert_eq!(cor.len(), 3, "the interior reference blocks the merge");
    }

    #[test]
    fn a_reversed_way_is_flipped_rather_than_dropped() {
        // OSM way direction is arbitrary. If the second way runs the other way
        // round, the corridor must reverse it — a version that only handled
        // head-to-tail would silently emit two corridors and understate the gain.
        let ways = vec![way(&[1, 2, 3], "foot"), way(&[5, 4, 3], "foot")];
        let r = refs_of(&ways);
        let (cor, _) = build_corridors(&ways, &r);
        assert_eq!(cor.len(), 1);
        assert_eq!(cor[0].1, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn a_class_change_declines_the_merge_and_the_decline_is_counted() {
        // The constraint is a choice, so its cost is measured rather than
        // assumed harmless — and the counter must actually move, or the report
        // would claim a free constraint.
        let ways = vec![
            way(&[1, 2, 3], "classified"),
            way(&[3, 4, 5], "residential"),
        ];
        let r = refs_of(&ways);
        let (cor, declined) = build_corridors(&ways, &r);
        assert_eq!(cor.len(), 2, "different classes stay apart");
        assert_eq!(declined, 1, "and the decline is counted");
    }

    #[test]
    fn a_closed_ring_is_emitted_once_and_not_merged_with_itself() {
        // A roundabout is one way whose two ends are the same node. Both ends
        // land in the same bucket, which a naive merge would read as a merge
        // point and splice the way onto itself.
        let ways = vec![way(&[1, 2, 3, 4, 1], "classified")];
        let r = refs_of(&ways);
        let (cor, _) = build_corridors(&ways, &r);
        assert_eq!(cor.len(), 1);
        assert_eq!(cor[0].1, vec![1, 2, 3, 4, 1]);
    }
}
