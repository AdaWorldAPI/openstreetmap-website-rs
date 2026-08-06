//! `tier_probe` — the M4 question, in its OSM form.
//!
//! ```text
//! tier_probe <input.osm.pbf>
//! ```
//!
//! `lance-graph/.claude/knowledge/bf16-hhtl-terrain.md` carries probe **M4**
//! ("HHTL termination: what % at each level?") as NOT RUN, and its process rule
//! says that an agent changing bucketing strategy runs the probe before
//! synthesising further. "One row per tile-cluster" **is** a bucketing-strategy
//! decision, so this is that probe.
//!
//! It reports, per cascade tier, two things measured on the real extract:
//!
//! 1. **Selectivity** — distinct prefixes and the feature-count distribution.
//!    That is termination: a tier whose cells hold one feature each terminates
//!    there; a tier whose cells hold thousands does not prune.
//! 2. **Slot fit** — a cluster row has 30 facet slots, one per member and one
//!    per tag pair, so a tile needs `members + tag_pairs` slots. The fraction
//!    of tiles fitting is what decides which tier a cluster is keyed at, and
//!    how often the design needs a continuation row.
//!
//! Nothing here asserts; it measures and prints. The numbers go in the plan.

use std::collections::HashMap;

use osm_soa_bake::{read, tms};
use osmpbf::{Element, ElementReader};

/// The cluster row's facet-slot budget (`ROW_SLOTS - RESERVED_SLOTS`).
const SLOTS: u64 = 30;

struct Tier {
    name: &'static str,
    /// How many low bits of the 64-bit Morton code to discard.
    shift: u32,
}

/// Quantiles of a sorted slice, reported rather than summarised away — a mean
/// alone hides exactly the tail that decides whether a fixed budget fits.
fn q(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let i = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[i]
}

/// **P1 — the way-refcount histogram** (`osm-chain-encoding-v1` §4).
///
/// OSM has no edges: a `way` is an ordered node-ref list, and the graph is
/// derived by refcounting. A node referenced by ≥2 ways is a junction; a node
/// referenced once is pure geometry whose *identity nothing dereferences*.
/// The share of the second class bounds how much of the node list is removable.
///
/// The three confounders the plan names are counted **separately, not merged**,
/// because each one is a reason a refcount-1 node may still not be removable:
///
/// - **way endpoints** — first/last of a way. A junction candidate regardless of
///   refcount, since a way that ends where nothing else touches still needs its
///   terminus.
/// - **tagged nodes** — a traffic signal or crossing carries meaning at
///   refcount 1; deleting it deletes data, not redundancy.
/// - **`layer`-tagged** — the bridge/tunnel case: distinct identity at a shared
///   position.
///
/// Reported as counts, not a verdict. The falsification condition is the
/// plan's: refcount-1 share near 1.0 *with* a high tagged share means the list
/// is re-keyable, not removable.
fn refcount_histogram(path: &str) -> Result<(), Box<dyn std::error::Error>> {
    // Which nodes are tagged, and which carry `layer`.
    let mut tagged: HashMap<i64, bool> = HashMap::with_capacity(1_300_000);
    ElementReader::from_path(path)?.for_each(|el| match el {
        Element::Node(n) if n.tags().next().is_some() => {
            tagged.insert(n.id(), n.tags().any(|(k, _)| k == "layer"));
        }
        Element::DenseNode(n) if n.tags().next().is_some() => {
            tagged.insert(n.id(), n.tags().any(|(k, _)| k == "layer"));
        }
        _ => {}
    })?;

    // Refcount over way membership, and separately whether a node is ever a
    // way endpoint.
    let mut refs: HashMap<i64, u32> = HashMap::with_capacity(8_000_000);
    let mut endpoint: HashMap<i64, ()> = HashMap::with_capacity(3_000_000);
    ElementReader::from_path(path)?.for_each(|el| {
        if let Element::Way(w) = el {
            let ids: Vec<i64> = w.refs().collect();
            if let (Some(&first), Some(&last)) = (ids.first(), ids.last()) {
                endpoint.insert(first, ());
                endpoint.insert(last, ());
            }
            for id in ids {
                *refs.entry(id).or_insert(0) += 1;
            }
        }
    })?;

    let referenced = refs.len() as u64;
    let mut once = 0u64;
    let mut once_endpoint = 0u64;
    let mut once_tagged = 0u64;
    let mut once_layer = 0u64;
    let mut once_plain = 0u64;
    let mut junction = 0u64;
    let mut hist: HashMap<u32, u64> = HashMap::new();
    for (&id, &c) in &refs {
        *hist.entry(c.min(8)).or_insert(0) += 1;
        if c >= 2 {
            junction += 1;
            continue;
        }
        once += 1;
        let is_end = endpoint.contains_key(&id);
        let tag = tagged.get(&id);
        if is_end {
            once_endpoint += 1;
        }
        if tag.is_some() {
            once_tagged += 1;
        }
        if tag == Some(&true) {
            once_layer += 1;
        }
        if !is_end && tag.is_none() {
            once_plain += 1;
        }
    }

    let pct = |n: u64| 100.0 * n as f64 / referenced as f64;
    println!("── P1: way-refcount histogram ──");
    println!("referenced nodes      {referenced:>12}");
    println!(
        "  refcount >= 2       {junction:>12}  ({:.2}%)  junctions",
        pct(junction)
    );
    println!("  refcount == 1       {once:>12}  ({:.2}%)", pct(once));
    println!(
        "    …way endpoint     {once_endpoint:>12}  ({:.2}%)  junction candidate anyway",
        pct(once_endpoint)
    );
    println!(
        "    …tagged           {once_tagged:>12}  ({:.2}%)  carries meaning at refcount 1",
        pct(once_tagged)
    );
    println!("      …of those, layer{once_layer:>12}  bridge/tunnel identity at a shared position",);
    println!(
        "    PLAIN SHAPE NODES {once_plain:>12}  ({:.2}%)  removable without semantic loss",
        pct(once_plain)
    );
    let mut ks: Vec<u32> = hist.keys().copied().collect();
    ks.sort_unstable();
    print!("  histogram (capped at 8):");
    for k in ks {
        print!(" {k}:{}", hist[&k]);
    }
    println!("\n");
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: tier_probe <input.osm.pbf>");
        std::process::exit(2);
    }

    if let Err(e) = refcount_histogram(&args[1]) {
        eprintln!("P1 failed: {e}");
        std::process::exit(1);
    }

    let (features, store, stats) = match read::read_features(&args[1]) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("read failed: {e}");
            std::process::exit(1);
        }
    };

    // (morton, tag_count) per feature — everything the probe needs.
    let coded: Vec<(u64, u64)> = features
        .iter()
        .map(|f| {
            (
                tms::point_to_tms_morton(f.lon, f.lat),
                u64::from(f.tags.len),
            )
        })
        .collect();
    drop(features);

    println!("features              {:>12}", coded.len());
    println!("tag pairs             {:>12}", stats.tag_pairs);
    println!("  distinct keys       {:>12}", store.distinct_keys());
    println!("  distinct values     {:>12}", store.distinct_values());
    println!(
        "tags per feature      {:>12.2}",
        stats.tag_pairs as f64 / coded.len() as f64
    );
    println!();

    // z=32 is 4 tiers of 8 quadtree levels; each tier is 16 Morton bits.
    let tiers = [
        Tier {
            name: "heel (z=8)",
            shift: 48,
        },
        Tier {
            name: "hip  (z=16)",
            shift: 32,
        },
        Tier {
            name: "twig (z=24)",
            shift: 16,
        },
        Tier {
            name: "leaf (z=32)",
            shift: 0,
        },
    ];

    println!(
        "{:<12} {:>10} {:>8} {:>8} {:>8} {:>9} {:>10} {:>9}",
        "tier", "tiles", "med", "p95", "max", "med slot", "p95 slot", "fit<=30"
    );
    for t in &tiers {
        // members and slot demand per tile
        let mut per_tile: HashMap<u64, (u64, u64)> = HashMap::new();
        for &(code, tags) in &coded {
            let e = per_tile.entry(code >> t.shift).or_insert((0, 0));
            e.0 += 1;
            e.1 += 1 + tags; // one slot for the member, one per tag pair
        }

        let mut members: Vec<u64> = per_tile.values().map(|v| v.0).collect();
        let mut slots: Vec<u64> = per_tile.values().map(|v| v.1).collect();
        members.sort_unstable();
        slots.sort_unstable();
        let fit = slots.iter().filter(|&&s| s <= SLOTS).count();

        println!(
            "{:<12} {:>10} {:>8} {:>8} {:>8} {:>9} {:>10} {:>8.1}%",
            t.name,
            per_tile.len(),
            q(&members, 0.5),
            q(&members, 0.95),
            members.last().copied().unwrap_or(0),
            q(&slots, 0.5),
            q(&slots, 0.95),
            100.0 * fit as f64 / per_tile.len() as f64,
        );
    }
}
