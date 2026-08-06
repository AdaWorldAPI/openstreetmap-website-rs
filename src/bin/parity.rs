//! `parity` — does the bake reproduce OSM's own data, element for element?
//!
//! ```text
//! parity <input.osm.pbf>
//! ```
//!
//! Everything else in this crate measures a *mechanism*. This measures the
//! **claim**: that a baked row, read back through nothing but its own bytes and
//! the bake's codebooks, yields the same element the `.osm.pbf` holds — same
//! id, same kind, same position, same tags.
//!
//! It bakes and verifies in one process rather than reading a `.soa` file,
//! because the codebooks are what make an ordinal mean anything and they are
//! not (yet) written alongside the slab. That is a real gap, not a shortcut:
//! **a bake that ships rows without its codebooks ships numbers.** Until the
//! codebooks are persisted with their digests, this binary is where the two
//! halves meet.
//!
//! # Two different position claims, deliberately separated
//!
//! - A **node** has a position OSM itself stores, on the 1e-7 degree grid. For
//!   a node, "position parity" means the bake hands back *that integer* — a
//!   claim about OSM's data.
//! - A **way** or **relation** has no stored position; this crate *derives* an
//!   anchor (centroid, or the `via` member). For those, the same check is a
//!   round-trip of our own derived value — a claim about the key, not about
//!   OSM.
//!
//! Reporting them merged would let the 52% that are derived carry the 48% that
//! are not, so they are counted apart.
//!
//! # This verdict is not vacuous — it failed twice before it passed
//!
//! A binary that prints `PARITY` is worth nothing unless it can print
//! `MISMATCH`, and this one did, on its first two runs against real data:
//!
//! 1. **2,633 tag mismatches, all with matching COUNTS.** `sort_unstable_by_key`
//!    on the Morton code alone is not a total order, and a feature's
//!    continuation rows all share its Morton — so the chunks of a long tag list
//!    came out permuted. Fixed by sorting on `(morton, tags.start)`
//!    (`row::sort_key`), which also makes two bakes of one extract
//!    byte-identical.
//! 2. **435,426 derived-position mismatches.** Not a defect — a wrong
//!    criterion. See the comment at the derived-position branch below.
//!
//! Neither was reachable from a unit test: the first needs enough features at
//! one Morton code for the sort to actually reorder, the second needs anchors
//! that are not grid points. Both are properties of the corpus, not of a
//! fixture.

use std::collections::HashMap;

/// One element as the extract states it: position on OSM's grid, plus its tags
/// in ordinal space.
type Truth = (i32, i32, Vec<(u32, u32)>);

use osm_soa_bake::cluster::{self, Facet};
use osm_soa_bake::identity::read_identity;
use osm_soa_bake::read::{self, OSM_NODE};
use osm_soa_bake::row::{self, Keyed};
use osm_soa_bake::tms;

/// What one element looks like after being read back out of the bake.
#[derive(Default)]
struct Recovered {
    kind: u16,
    lon_e7: i32,
    lat_e7: i32,
    tags: Vec<(u32, u32)>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: parity <input.osm.pbf>");
        std::process::exit(2);
    }

    // ── Ground truth: the extract itself. ──
    let (features, tag_store, _stats) = match read::read_features(&args[1]) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("read failed: {e}");
            std::process::exit(1);
        }
    };
    let tags = tag_store.resolve().expect("tag codebooks");

    // Ground truth keyed by the same (kind, osm_id) the bake identifies by.
    let mut truth: HashMap<(u16, i64), Truth> = HashMap::with_capacity(features.len());
    for f in &features {
        let expect = (
            (f.lon * tms::OSM_GRID_PER_DEGREE).round() as i32,
            (f.lat * tms::OSM_GRID_PER_DEGREE).round() as i32,
        );
        truth.insert(
            (f.entity_type, f.osm_id),
            (expect.0, expect.1, tags.span(f.tags).to_vec()),
        );
    }

    // ── The bake, exactly as `bake` runs it. ──
    let mut keyed: Vec<Keyed> = features.iter().map(row::key_feature).collect();
    drop(features);
    row::expand_tag_overflow(&mut keyed);
    keyed.sort_unstable_by_key(row::sort_key);
    row::assign_identities(&mut keyed);
    let book = row::resolve_identities(&mut keyed).expect("identity codebook");

    // ── Read every row back, using ONLY its bytes and the codebooks. ──
    //
    // Nothing from `Keyed` leaks into this side except the row it produced —
    // otherwise the check would be comparing the pipeline to itself.
    let mut got: HashMap<u32, Recovered> = HashMap::with_capacity(keyed.len());
    for k in &keyed {
        let r = row::build_row(k, &tags);

        let (kind, ordinal) = match read_identity(&r) {
            Some(v) => v,
            None => {
                eprintln!("a row carries no identity — the bake keyed what it cannot name");
                std::process::exit(1);
            }
        };

        // Position comes from the KEY, decoded, not from the feature.
        let b = r.key.as_bytes();
        let code = (u64::from(u16::from_le_bytes([b[4], b[5]])) << 48)
            | (u64::from(u16::from_le_bytes([b[6], b[7]])) << 32)
            | (u64::from(u16::from_le_bytes([b[8], b[9]])) << 16)
            | u64::from(u16::from_le_bytes([b[10], b[11]]));
        let (lon_e7, lat_e7) = tms::morton_to_osm_grid(code);

        let e = got.entry(ordinal).or_default();
        e.kind = kind;
        e.lon_e7 = lon_e7;
        e.lat_e7 = lat_e7;
        for (_, f) in cluster::facets(&r) {
            if let Facet::Tag { member, key, value } = f {
                if member == ordinal {
                    e.tags.push((key, value));
                }
            }
        }
    }

    // ── Diff. ──
    let mut checked = 0u64;
    let (mut node_pos_ok, mut node_pos_bad) = (0u64, 0u64);
    let (mut derived_pos_ok, mut derived_pos_bad) = (0u64, 0u64);
    let (mut tags_ok, mut tags_bad) = (0u64, 0u64);
    let mut kind_bad = 0u64;
    let mut missing = 0u64;
    let mut derived_max = 0u32;
    let mut shown = 0u32;

    for (ordinal, rec) in &got {
        // The ordinal pulls back to "kind:osm_id" through the codebook — the
        // bijection #902 calls the tenant's acceptance criterion.
        let Some(key) = book.key(*ordinal) else {
            eprintln!("ordinal {ordinal} is not in the codebook");
            std::process::exit(1);
        };
        let (kind_hex, id_str) = key.split_once(':').expect("kind-qualified key");
        let kind = u16::from_str_radix(kind_hex, 16).expect("hex kind");
        let osm_id: i64 = id_str.parse().expect("numeric id");

        let Some((lon_e7, lat_e7, want_tags)) = truth.get(&(kind, osm_id)) else {
            missing += 1;
            continue;
        };
        checked += 1;

        if rec.kind != kind {
            kind_bad += 1;
        }

        let d = (
            (rec.lon_e7 - *lon_e7).unsigned_abs(),
            (rec.lat_e7 - *lat_e7).unsigned_abs(),
        );
        if kind == OSM_NODE {
            // A node's coordinate IS a grid point, so the round trip is exact
            // or the key lost it. No tolerance: that is the whole claim.
            if d == (0, 0) {
                node_pos_ok += 1;
            } else {
                node_pos_bad += 1;
            }
        } else {
            // A derived anchor is an arbitrary f64 centroid, NOT a grid point.
            // The key snaps it to a cell, and the cell centre rounds to the
            // grid point nearest THE CENTRE — which is a neighbour of the one
            // nearest the centroid whenever the centroid sits off-centre. So
            // exact equality is the wrong criterion here and asserting it
            // measured nothing but that centroids are not grid points (it
            // "failed" 435,426 of 1,333,178 on the first run).
            //
            // The right claim is quantisation: the recovered point is within
            // one grid step of the anchor. The observed maximum is reported,
            // so a genuine key defect would show up as a larger deviation
            // rather than hiding under a generous bound.
            derived_max = derived_max.max(d.0.max(d.1));
            if d.0 <= 1 && d.1 <= 1 {
                derived_pos_ok += 1;
            } else {
                derived_pos_bad += 1;
            }
        }

        if rec.tags == *want_tags {
            tags_ok += 1;
        } else {
            tags_bad += 1;
            if shown < 5 {
                shown += 1;
                eprintln!(
                    "tag mismatch {kind:#06x}:{osm_id} — bake {} tags, source {}",
                    rec.tags.len(),
                    want_tags.len()
                );
            }
        }
    }

    let unrecovered = truth.len() as u64 - checked;

    println!("── OSM feature parity ──");
    println!("source features       {:>12}", truth.len());
    println!("recovered elements    {:>12}", got.len());
    println!("checked               {:>12}", checked);
    println!("NOT recovered         {:>12}", unrecovered);
    println!("unknown to source     {:>12}", missing);
    println!("kind mismatches       {:>12}", kind_bad);
    println!(
        "node position   ok    {node_pos_ok:>12}   bad {node_pos_bad}   (vs OSM's own 1e-7 integer)"
    );
    println!(
        "derived position ok   {derived_pos_ok:>12}   bad {derived_pos_bad}   \
(anchor quantised to the key; max deviation {derived_max} grid step(s))"
    );
    println!("tags exact            {tags_ok:>12}   bad {tags_bad}");

    let clean = unrecovered == 0
        && missing == 0
        && kind_bad == 0
        && node_pos_bad == 0
        && derived_pos_bad == 0
        && tags_bad == 0;
    println!(
        "\nVERDICT               {}",
        if clean { "PARITY" } else { "MISMATCH" }
    );
    if !clean {
        std::process::exit(1);
    }
}
