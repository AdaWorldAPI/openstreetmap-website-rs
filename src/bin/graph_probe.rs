//! `graph_probe` — **P8**, the routing gate. Binary, not tolerant.
//!
//! ```text
//! graph_probe <input.osm.pbf>
//! ```
//!
//! Every probe before this one measured **geometry**: fit error, byte counts,
//! turn distributions, control points. All of them are graded against P2's
//! 1.69 m tolerance. **None of that is the criterion that matters.**
//!
//! The criterion is: *does a driver get sent into the middle of nowhere because
//! an edge was not recognised exactly as a path?* A 1.7 m displacement is
//! invisible to a router. A **missing** turn-off, a **phantom** junction, or a
//! **false** connection is not — and none of those is a tolerance, they are
//! yes/no. This probe measures the yes/no.
//!
//! Four exposures, each a count that must be understood before any encoding
//! that removes node identity ships:
//!
//! 1. **Restriction nodes that would lose identity.** A turn restriction is a
//!    relation referencing a node by **id** (`via`). Node identity is exactly
//!    what the chain/template encodings discard for shape nodes, so a `via` node
//!    classified as shape leaves the restriction dangling — the router turns
//!    where it must not. This is the sharpest failure mode and the cheapest to
//!    check.
//! 2. **Phantom junctions.** P4 measured that two fifths of Berlin's
//!    refcount ≥ 2 nodes are shared *building walls*. Deriving junctions from a
//!    class-blind refcount puts a routing node on a house wall. This counts the
//!    contamination against the routable graph specifically.
//! 3. **Crossings with no shared node.** A bridge over a road crosses it
//!    geometrically and shares nothing. Any transformation that joins by
//!    proximity rather than by identity would connect them — a road that does
//!    not exist, which is the failure in its purest form. This counts the
//!    exposure.
//! 4. **Name-based corridor assembly.** Earlier probes concluded three times
//!    over that "the way is the wrong unit, the corridor is". That
//!    recommendation is checked here against its own risk: if corridors are
//!    assembled by **name**, how many name groups are not actually connected?
//!    Each disconnected group is a join the assembly would invent.
//!
//! # Why two extracts
//!
//! Berlin is dense, urban, heavily split, and short-wayed — it under-represents
//! exactly the cases that break a routing graph: long rural ways, trunk roads,
//! bridges over open ground. **A Berlin pass is therefore weak evidence, while a
//! Berlin failure is decisive.** Iceland is the complement (ring road, bridges,
//! few buildings), so the gate is run on both and neither number stands alone.
//!
//! # What "routable" means here, stated once
//!
//! A way with a `highway` tag whose value is not one of the non-routable
//! markers (`construction`, `proposed`, `platform`, `raceway`, and the point
//! features that occasionally appear on ways). Buildings, landuse and barriers
//! are **not** routable and are counted separately — that separation is the
//! whole of exposure 2.

use std::collections::{HashMap, HashSet};

use osm_soa_bake::geodesy::{meridional_radius, normal_radius};
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

/// Grid cell for the segment index, metres.
const SEG_CELL_M: f64 = 64.0;

/// A local equirectangular frame — metres from an anchor, for the crossing test.
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

/// Sign of the cross product `(b-a) x (c-a)`, as -1 / 0 / +1.
fn orient(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> i8 {
    let v = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    if v > 1e-9 {
        1
    } else if v < -1e-9 {
        -1
    } else {
        0
    }
}

/// Do segments `p1p2` and `p3p4` properly cross (interiors intersect)?
///
/// Proper crossing only — touching at an endpoint is how OSM expresses a real
/// junction and must NOT be counted as a bridge-style crossing.
fn segments_cross(p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), p4: (f64, f64)) -> bool {
    let d1 = orient(p3, p4, p1);
    let d2 = orient(p3, p4, p2);
    let d3 = orient(p1, p2, p3);
    let d4 = orient(p1, p2, p4);
    d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0 && d1 != d2 && d3 != d4
}

/// Union-find over way indices, for the name-group connectivity check.
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

fn pct(n: u64, d: u64) -> f64 {
    if d == 0 {
        0.0
    } else {
        n as f64 * 100.0 / d as f64
    }
}

struct RoutableWay {
    nodes: Vec<i64>,
    name: Option<String>,
    /// `bridge`, `tunnel` or a non-zero `layer` — the tags that say "this does
    /// not meet what it crosses".
    grade_separated: bool,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: graph_probe <input.osm.pbf>");
        std::process::exit(2);
    }
    let path = &args[1];

    // ── Pass 1: coordinates + frame anchor. ──
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
    eprintln!("pass 1: {} nodes", coords.len());

    // ── Pass 2: ways, split by routability. ──
    let mut routable_ref: HashMap<i64, u32> = HashMap::with_capacity(4_000_000);
    let mut other_ref: HashMap<i64, u32> = HashMap::with_capacity(6_000_000);
    let mut ways: Vec<RoutableWay> = Vec::with_capacity(500_000);

    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Way(w) = el else { return };
            let tags: Vec<(&str, &str)> = w.tags().collect();
            let get = |k: &str| tags.iter().find(|(t, _)| *t == k).map(|(_, v)| *v);
            let hw = get("highway");
            let routable = hw.is_some_and(|v| !NON_ROUTABLE.contains(&v));
            let ids: Vec<i64> = w.refs().collect();
            if ids.len() < 2 {
                return;
            }
            if routable {
                for &i in &ids {
                    *routable_ref.entry(i).or_insert(0) += 1;
                }
                let grade_separated = get("bridge").is_some_and(|v| v != "no")
                    || get("tunnel").is_some_and(|v| v != "no")
                    || get("layer").is_some_and(|v| v != "0");
                ways.push(RoutableWay {
                    nodes: ids,
                    name: get("name").map(std::string::ToString::to_string),
                    grade_separated,
                });
            } else {
                for &i in &ids {
                    *other_ref.entry(i).or_insert(0) += 1;
                }
            }
        })
        .expect("pass 2");
    eprintln!("pass 2: {} routable ways", ways.len());

    // ── Pass 3: relations — the nodes whose IDENTITY is load-bearing. ──
    let mut via_nodes: HashSet<i64> = HashSet::new();
    let mut relation_nodes: HashSet<i64> = HashSet::new();
    let mut restrictions = 0u64;
    ElementReader::from_path(path)
        .expect("open")
        .for_each(|el| {
            let Element::Relation(r) = el else { return };
            let is_restriction = r
                .tags()
                .any(|(k, v)| k == "type" && v.starts_with("restriction"));
            if is_restriction {
                restrictions += 1;
            }
            for m in r.members() {
                if m.member_type != osmpbf::RelMemberType::Node {
                    continue;
                }
                relation_nodes.insert(m.member_id);
                if is_restriction && m.role().is_ok_and(|s| s == "via") {
                    via_nodes.insert(m.member_id);
                }
            }
        })
        .expect("pass 3");

    // ── (1) Restriction nodes that a shape-node drop would erase. ──
    //
    // A node is "shape" exactly when the routable graph references it once. If a
    // `via` node is in that class, dropping shape identity dangles the
    // restriction — and a dangling turn restriction is a router turning where it
    // must not.
    let via_shape = via_nodes
        .iter()
        .filter(|n| routable_ref.get(n).copied().unwrap_or(0) < 2)
        .count() as u64;
    let rel_shape = relation_nodes
        .iter()
        .filter(|n| routable_ref.get(n).copied().unwrap_or(0) < 2)
        .count() as u64;

    println!("\n(1) node identity a relation depends on");
    println!(
        "  {restrictions} turn restrictions, {} via nodes",
        via_nodes.len()
    );
    println!(
        "  via nodes the routable graph sees fewer than twice: {via_shape} ({:.2}%)",
        pct(via_shape, via_nodes.len() as u64)
    );
    println!(
        "  ALL relation-referenced nodes in that class:        {rel_shape} of {} ({:.2}%)",
        relation_nodes.len(),
        pct(rel_shape, relation_nodes.len() as u64)
    );
    println!("  GATE: any encoding dropping shape-node identity must keep these by id.");

    // ── (2) Phantom junctions — refcount without class separation. ──
    let mut class_blind = 0u64;
    let mut routing_junctions = 0u64;
    let mut phantom = 0u64;
    let mut all: HashSet<i64> = HashSet::with_capacity(routable_ref.len() + other_ref.len());
    all.extend(routable_ref.keys().copied());
    all.extend(other_ref.keys().copied());
    for n in &all {
        let r = routable_ref.get(n).copied().unwrap_or(0);
        let o = other_ref.get(n).copied().unwrap_or(0);
        if r + o >= 2 {
            class_blind += 1;
        }
        if r >= 2 {
            routing_junctions += 1;
        }
        if r + o >= 2 && r < 2 {
            phantom += 1;
        }
    }
    println!("\n(2) phantom junctions — what a class-blind refcount would invent");
    println!("  class-blind refcount >= 2:      {class_blind}");
    println!("  actual routing junctions:       {routing_junctions}");
    println!(
        "  PHANTOM (not routable, counted): {phantom} — {:.1}% of the class-blind set",
        pct(phantom, class_blind)
    );
    println!("  GATE: junctions must be derived from the ROUTABLE refcount alone.");

    // ── (3) Crossings that share no node. ──
    //
    // Two routable segments whose interiors intersect while the ways share no
    // node id. In OSM that is a bridge, a tunnel, or a layer difference — never
    // a junction. Any join-by-proximity would connect them.
    let mut grid: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
    for (wi, w) in ways.iter().enumerate() {
        for si in 0..w.nodes.len().saturating_sub(1) {
            let (Some(&a), Some(&b)) = (coords.get(&w.nodes[si]), coords.get(&w.nodes[si + 1]))
            else {
                continue;
            };
            let p = frame.xy(a.0, a.1);
            let q = frame.xy(b.0, b.1);
            let (x0, x1) = (p.0.min(q.0), p.0.max(q.0));
            let (y0, y1) = (p.1.min(q.1), p.1.max(q.1));
            let cx0 = (x0 / SEG_CELL_M).floor() as i32;
            let cx1 = (x1 / SEG_CELL_M).floor() as i32;
            let cy0 = (y0 / SEG_CELL_M).floor() as i32;
            let cy1 = (y1 / SEG_CELL_M).floor() as i32;
            for cx in cx0..=cx1 {
                for cy in cy0..=cy1 {
                    grid.entry((cx, cy)).or_default().push((wi, si));
                }
            }
        }
    }
    let node_sets: Vec<HashSet<i64>> = ways
        .iter()
        .map(|w| w.nodes.iter().copied().collect())
        .collect();
    let mut crossings: HashSet<(usize, usize)> = HashSet::new();
    let mut crossings_graded = 0u64;
    for bucket in grid.values() {
        for i in 0..bucket.len() {
            for j in i + 1..bucket.len() {
                let (wa, sa) = bucket[i];
                let (wb, sb) = bucket[j];
                if wa == wb {
                    continue;
                }
                let key = (wa.min(wb), wa.max(wb));
                if crossings.contains(&key) {
                    continue;
                }
                // Sharing ANY node makes this a junction, not a crossing.
                if node_sets[wa].intersection(&node_sets[wb]).next().is_some() {
                    continue;
                }
                let (Some(&a1), Some(&a2)) = (
                    coords.get(&ways[wa].nodes[sa]),
                    coords.get(&ways[wa].nodes[sa + 1]),
                ) else {
                    continue;
                };
                let (Some(&b1), Some(&b2)) = (
                    coords.get(&ways[wb].nodes[sb]),
                    coords.get(&ways[wb].nodes[sb + 1]),
                ) else {
                    continue;
                };
                if segments_cross(
                    frame.xy(a1.0, a1.1),
                    frame.xy(a2.0, a2.1),
                    frame.xy(b1.0, b1.1),
                    frame.xy(b2.0, b2.1),
                ) {
                    crossings.insert(key);
                    if ways[wa].grade_separated || ways[wb].grade_separated {
                        crossings_graded += 1;
                    }
                }
            }
        }
    }
    println!("\n(3) crossings with NO shared node — the false-join exposure");
    println!(
        "  routable way pairs crossing geometrically: {}",
        crossings.len()
    );
    println!(
        "  of which one side is bridge/tunnel/layer:  {crossings_graded} ({:.1}%)",
        pct(crossings_graded, crossings.len() as u64)
    );
    println!("  GATE: connectivity must come from node IDENTITY, never from proximity.");

    // ── (4) Name-based corridor assembly, checked against its own risk. ──
    let mut by_name: HashMap<&str, Vec<usize>> = HashMap::new();
    for (i, w) in ways.iter().enumerate() {
        if let Some(n) = &w.name {
            by_name.entry(n.as_str()).or_default().push(i);
        }
    }
    let mut groups = 0u64;
    let mut split_groups = 0u64;
    let mut spurious = 0u64;
    let mut worst = (0usize, "");
    for (name, members) in &by_name {
        if members.len() < 2 {
            continue;
        }
        groups += 1;
        let mut dsu = Dsu::new(members.len());
        let mut owner: HashMap<i64, usize> = HashMap::new();
        for (li, &wi) in members.iter().enumerate() {
            for &n in &ways[wi].nodes {
                if let Some(&prev) = owner.get(&n) {
                    dsu.union(prev, li);
                } else {
                    owner.insert(n, li);
                }
            }
        }
        let comps: HashSet<usize> = (0..members.len()).map(|i| dsu.find(i)).collect();
        if comps.len() > 1 {
            split_groups += 1;
            spurious += comps.len() as u64 - 1;
            if comps.len() > worst.0 {
                worst = (comps.len(), name);
            }
        }
    }
    println!("\n(4) name-based corridor assembly — checking my own recommendation");
    println!("  named groups with >= 2 ways:  {groups}");
    println!(
        "  groups NOT actually connected: {split_groups} ({:.1}%)",
        pct(split_groups, groups)
    );
    println!("  joins such an assembly would INVENT: {spurious}");
    println!(
        "  worst group: \"{}\" in {} disconnected pieces",
        worst.1, worst.0
    );
    println!("  GATE: a corridor is a connected component, never a name.");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_proper_crossing_counts_and_a_shared_endpoint_does_not() {
        // The distinction the whole of exposure 3 rests on. Two segments meeting
        // AT a point is how OSM writes a junction; two segments whose interiors
        // cross is a bridge. Conflating them would either invent roads or erase
        // every real junction, depending on which way the confusion ran.
        let cross = segments_cross((0.0, 0.0), (10.0, 10.0), (0.0, 10.0), (10.0, 0.0));
        assert!(cross, "interiors intersect");

        let touching = segments_cross((0.0, 0.0), (10.0, 0.0), (10.0, 0.0), (20.0, 0.0));
        assert!(!touching, "a shared endpoint is a junction, not a crossing");

        let apart = segments_cross((0.0, 0.0), (10.0, 0.0), (0.0, 5.0), (10.0, 5.0));
        assert!(!apart, "parallel segments do not cross");

        let t_junction = segments_cross((0.0, 0.0), (10.0, 0.0), (5.0, 0.0), (5.0, 10.0));
        assert!(
            !t_junction,
            "a T touching the line is not a proper crossing"
        );
    }

    #[test]
    fn union_find_splits_a_disconnected_name_group() {
        // Exposure 4's mechanism, two-sided: two ways sharing a node are ONE
        // corridor, two ways sharing nothing are TWO — and a union-find that
        // merged everything would report every group connected and hide the
        // whole finding.
        let mut d = Dsu::new(3);
        d.union(0, 1);
        let comps: HashSet<usize> = (0..3).map(|i| d.find(i)).collect();
        assert_eq!(comps.len(), 2, "0-1 joined, 2 alone");

        let mut e = Dsu::new(3);
        assert_eq!(
            (0..3).map(|i| e.find(i)).collect::<HashSet<_>>().len(),
            3,
            "nothing joined without a union"
        );
    }

    #[test]
    fn orientation_is_signed_and_collinear_is_zero() {
        // Guards the crossing test's core predicate. A sign error would flip
        // every verdict; a missing zero case would make collinear segments
        // report as crossing.
        assert_eq!(orient((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)), 1);
        assert_eq!(orient((0.0, 0.0), (1.0, 0.0), (0.0, -1.0)), -1);
        assert_eq!(orient((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)), 0);
    }

    #[test]
    fn non_routable_markers_exclude_only_what_carries_no_traffic() {
        // Exposure 2 depends entirely on this split being right: too wide and
        // real roads vanish from the graph, too narrow and platforms become
        // junctions.
        assert!(NON_ROUTABLE.contains(&"construction"));
        assert!(NON_ROUTABLE.contains(&"platform"));
        assert!(!NON_ROUTABLE.contains(&"footway"), "pedestrians route");
        assert!(!NON_ROUTABLE.contains(&"service"), "driveways route");
        assert!(!NON_ROUTABLE.contains(&"residential"));
    }
}
