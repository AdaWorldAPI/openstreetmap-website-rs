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

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: tier_probe <input.osm.pbf>");
        std::process::exit(2);
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
