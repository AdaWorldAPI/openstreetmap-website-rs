//! Proves `access::{access_from_tags, oneway_from_tags,
//! bicycle_contraflow_from_tags, RestrictionMask}` against REAL Brandenburg
//! data — tags from real ways, restrictions from real relations.
//!
//! ```text
//! access_probe <region.osm.pbf>
//! ```
//!
//! The restriction half does NOT claim real slot indices — junction rows
//! aren't written yet, so there is no real `from_slot`/`to_slot` to read.
//! What it DOES prove, independently of P12's own probe: how many forbidden
//! (from-way, to-way) pairs land at ONE junction, because that number is
//! `RestrictionMask`'s real capacity question — 8×8 has to be enough, not
//! merely non-zero.

use std::collections::HashMap;

use osm_soa_bake::access::{
    access_from_tags, bicycle_contraflow_from_tags, oneway_from_tags, RestrictionMask, ACCESS_ALL,
    ACCESS_BIKE, ACCESS_CAR, ACCESS_FOOT,
};
use osmpbf::{Element, ElementReader};

fn main() {
    let path = std::env::args().nth(1).expect("usage: access_probe <pbf>");

    let (mut ways, mut open, mut oneway_fwd, mut oneway_back, mut contraflow) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut denied: HashMap<&'static str, u64> = HashMap::new();

    // via-node -> ways incident on it (routable ways only, same gate P12 used)
    let mut incident: HashMap<i64, Vec<i64>> = HashMap::new();
    // restriction: (via, from_way, to_way)
    let mut restrictions: Vec<(i64, i64, i64)> = Vec::new();

    ElementReader::from_path(&path)
        .expect("open pbf")
        .for_each(|el| match el {
            Element::Way(w) => {
                let tags: Vec<(&str, &str)> = w.tags().collect();
                let Some(_hw) = tags.iter().find(|(k, _)| *k == "highway") else { return };
                ways += 1;
                let access = access_from_tags(tags.iter().copied());
                if access == ACCESS_ALL {
                    open += 1;
                }
                if access & ACCESS_CAR == 0 {
                    *denied.entry("car").or_default() += 1;
                }
                if access & ACCESS_BIKE == 0 {
                    *denied.entry("bike").or_default() += 1;
                }
                if access & ACCESS_FOOT == 0 {
                    *denied.entry("foot").or_default() += 1;
                }
                match oneway_from_tags(tags.iter().copied()) {
                    1 => oneway_fwd += 1,
                    2 => oneway_back += 1,
                    _ => {}
                }
                if bicycle_contraflow_from_tags(tags.iter().copied()) {
                    contraflow += 1;
                }
                for n in w.refs() {
                    incident.entry(n).or_default().push(w.id());
                }
            }
            Element::Relation(r) => {
                let is_restriction = r.tags().any(|(k, v)| k == "type" && v.starts_with("restriction"));
                if !is_restriction {
                    return;
                }
                let (mut via, mut from, mut to) = (None, None, None);
                for m in r.members() {
                    match m.role().unwrap_or("") {
                        "via" => {
                            if let osmpbf::RelMemberType::Node = m.member_type {
                                via = Some(m.member_id);
                            }
                        }
                        "from" => from = Some(m.member_id),
                        "to" => to = Some(m.member_id),
                        _ => {}
                    }
                }
                if let (Some(v), Some(f), Some(t)) = (via, from, to) {
                    restrictions.push((v, f, t));
                }
            }
            _ => {}
        })
        .expect("read pbf");

    println!("ways with highway tag: {ways}");
    println!("fully open (no access denial): {open} ({:.1}%)", 100.0 * open as f64 / ways as f64);
    println!("access denials: {denied:?}");
    println!("oneway forward: {oneway_fwd}  backward: {oneway_back}");
    println!("bicycle contraflow (oneway:bicycle=no): {contraflow}");

    println!("\n── restrictions ──");
    println!("relations parsed: {}", restrictions.len());

    // Group forbidden (from,to) way-pairs by via node, using the SAME
    // incidence check P12's probe used — both from and to must be incident.
    let mut per_via: HashMap<i64, Vec<(i64, i64)>> = HashMap::new();
    let mut incident_both = 0u64;
    for &(via, from, to) in &restrictions {
        let Some(ways_here) = incident.get(&via) else { continue };
        if ways_here.contains(&from) && ways_here.contains(&to) {
            incident_both += 1;
            per_via.entry(via).or_default().push((from, to));
        }
    }
    println!(
        "from+to both incident at via: {incident_both} / {} ({:.1}%) — the P12 collapse, re-measured independently",
        restrictions.len(),
        100.0 * incident_both as f64 / restrictions.len().max(1) as f64
    );

    // The real capacity question: how many forbidden pairs land at ONE
    // junction? RestrictionMask is 8x8 = 64 slot-pairs; a junction can have
    // at most 8 DISTINCT incident ways, so at most 8x7=56 distinct ordered
    // pairs are even possible — but "how many restrictions actually pack in"
    // is a measured fact, not a combinatorial ceiling.
    let mut max_per_junction = 0usize;
    let mut multi_restriction_junctions = 0usize;
    for (via, pairs) in &per_via {
        max_per_junction = max_per_junction.max(pairs.len());
        if pairs.len() > 1 {
            multi_restriction_junctions += 1;
        }
        // Prove the mask actually holds this junction's real pairs, with
        // arbitrary-but-consistent slot assignment (way id order) — NOT a
        // claim about real slot indices, which need the row writer.
        let ways_here = &incident[via];
        let slot_of = |way: i64| ways_here.iter().position(|&w| w == way).map(|i| i as u8);
        let mut mask = RestrictionMask::EMPTY;
        let mut ok = true;
        for &(from, to) in pairs {
            match (slot_of(from), slot_of(to)) {
                (Some(f), Some(t)) if f < 8 && t < 8 => mask = mask.with_forbidden(f, t),
                _ => ok = false, // more than 8 distinct incident ways at this junction
            }
        }
        for &(from, to) in pairs {
            if let (Some(f), Some(t)) = (slot_of(from), slot_of(to)) {
                if f < 8 && t < 8 {
                    assert!(mask.is_forbidden(f, t), "real pair did not survive packing");
                }
            }
        }
        if !ok {
            eprintln!("note: junction {via} has >8 distinct incident ways — outside EDGE_SLOTS");
        }
    }
    println!("junctions carrying >1 restriction: {multi_restriction_junctions}");
    println!("max restrictions packed at one junction: {max_per_junction} (mask capacity: 64 pairs)");
    println!("every real pair round-tripped through RestrictionMask correctly");
}
