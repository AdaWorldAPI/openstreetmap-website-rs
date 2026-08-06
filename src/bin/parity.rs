//! `parity` — does the bake reproduce OSM's own data, element for element?
//!
//! ```text
//! parity <input.osm.pbf> [<slab> <slab.books>]
//! ```
//!
//! Everything else in this crate measures a *mechanism*. This measures the
//! **claim**: that a baked row, read back through nothing but its own bytes and
//! the bake's codebooks, yields the same element the `.osm.pbf` holds — same
//! id, same kind, same position, same tags.
//!
//! # Two modes, and why the second one exists
//!
//! ```text
//! parity <input.osm.pbf>                       # re-bake in-process, verify
//! parity <input.osm.pbf> <slab> <slab.books>   # verify the ARTIFACT on disk
//! ```
//!
//! The first mode verifies the *pipeline*. It was all this binary could do
//! before the codebooks were persisted, and it is **strictly weaker than it
//! looks** — it never touches the file a consumer would actually read.
//!
//! That weakness was not theoretical. `bake` never called
//! `row::resolve_identities`, so every `Keyed` carried `identity_ordinal:
//! None`, `build_row` wrote no identity facet and no tags, and the 1.2 GiB slab
//! on disk was **keys with a wholly zeroed value slab** — 200 of 200 sampled
//! rows. In-process parity passed the whole time, because this binary resolved
//! the identities itself. The pipeline was correct; the artifact was empty; and
//! only a check that opened the file could tell the difference.
//!
//! So the second mode is the real one. It reads rows from the slab and books
//! from the sidecar and reconstructs elements from those bytes alone.
//!
//! # Two position contracts, both now exact
//!
//! - **Published** (a node) — the coordinate OSM itself stores, on its 1e-7
//!   degree grid. Parity means handing back *that integer*.
//! - **Derived** (a way or relation) — an anchor this crate computed. It is a
//!   z=32 grid cell **by construction** (`tms::mean_cell`, integer mean of
//!   member cells under a fixed rounding rule), so parity means handing back
//!   *that cell*.
//!
//! Both are `assert_eq!` on integers. The derived one used to be "within one
//! grid step", because the anchor was an f64 centroid compared against OSM's
//! 1e-7 grid — the wrong grid for a value that was never an OSM coordinate.
//! Deriving in the key's own grid removed the tolerance and, more importantly,
//! removed the float: the derivation is now deterministic across platforms with
//! no transcendental kernel to certify. The one remaining float is the ingest
//! step (`merc_y`, once per published coordinate), which belongs to the first
//! contract.
//!
//! They stay counted apart because they are different claims — one about OSM's
//! data, one about this crate's own derivation.
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
//!    criterion, and the measurement that motivated deriving anchors natively
//!    in the key's grid. Both halves are exact now.
//!
//! Neither was reachable from a unit test: the first needs enough features at
//! one Morton code for the sort to actually reorder, the second needs anchors
//! that are not grid points. Both are properties of the corpus, not of a
//! fixture.

use std::collections::HashMap;

use lance_graph_contract::canonical_node::{EdgeBlock, NodeGuid, NodeRow, TailVariant};
use osm_soa_bake::codebook::read_books;

/// One element as the extract states it: its anchor under whichever contract
/// applies, plus its tags in ordinal space.
type Truth = (Anchor, Vec<(u32, u32)>);

use osm_soa_bake::cluster::{self, Facet};
use osm_soa_bake::identity::read_identity;
use osm_soa_bake::read::{self, Anchor};
use osm_soa_bake::row::{self, Keyed};
use osm_soa_bake::tms;

/// Load a slab file into an ALIGNED row vector.
///
/// `RowSlab` borrows `&[NodeRow]` out of a byte slice and rightly refuses one
/// that is not 64-byte aligned — `NodeRow` is `#[repr(C, align(64))]`. An
/// `mmap` satisfies that (page-aligned); a `Vec<u8>` from `fs::read` does not,
/// and this verifier hit exactly that. Copying into a `Vec<NodeRow>` is the
/// honest fix here: it is one pass over a cold artifact, and it keeps the
/// alignment guarantee a type-level fact rather than a hope about the allocator.
fn load_rows(path: &str) -> Vec<NodeRow> {
    let bytes = std::fs::read(path).expect("read slab");
    let stride = core::mem::size_of::<NodeRow>();
    assert_eq!(
        bytes.len() % stride,
        0,
        "slab is not a whole number of rows"
    );
    let blank = NodeRow {
        key: NodeGuid::mint_for(TailVariant::V3, 0, 0, 0, 0, 0, 0, 0),
        edges: EdgeBlock::default(),
        value: [0u8; 480],
    };
    let mut rows = vec![blank; bytes.len() / stride];
    // SAFETY: NodeRow is #[repr(C, align(64))] with a compile-asserted 512-byte
    // size and no padding-dependent semantics — the canon defines the row AS
    // its little-endian bytes, which is the same reason `bake` writes it this way.
    let dst = unsafe {
        core::slice::from_raw_parts_mut(rows.as_mut_ptr().cast::<u8>(), rows.len() * stride)
    };
    dst.copy_from_slice(&bytes);
    rows
}

/// What one element looks like after being read back out of the bake.
#[derive(Default)]
struct Recovered {
    kind: u16,
    /// The cell the key names — recovered by integer de-interleave, exact.
    cell: tms::TileXy,
    /// The OSM grid point that cell snaps to. Only meaningful for a published
    /// anchor; see the two contracts in the module doc.
    lon_e7: i32,
    lat_e7: i32,
    tags: Vec<(u32, u32)>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: parity <input.osm.pbf> [<slab> <slab.books>]");
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
        truth.insert(
            (f.entity_type, f.osm_id),
            (f.anchor, tags.span(f.tags).to_vec()),
        );
    }

    // ── The rows to verify: from disk if given, else re-baked in-process. ──
    let (book, rows): (_, Vec<NodeRow>) = if args.len() >= 4 {
        let mut f = std::fs::File::open(&args[3]).expect("open codebooks");
        let books = match read_books(&mut f) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("codebooks refused: {e}");
                std::process::exit(1);
            }
        };
        // The books on disk must be the books the bake used, or every
        // ordinal addresses a neighbour. Digests are checked by read_books;
        // this pins them against the freshly-read extract too.
        if books.tag_keys.digest() != tags.keys.digest()
            || books.tag_values.digest() != tags.values.digest()
        {
            eprintln!("tag codebooks on disk do not match this extract");
            std::process::exit(1);
        }
        eprintln!("verifying the ARTIFACT: {} + {}", args[2], args[3]);
        (books.identities, load_rows(&args[2]))
    } else {
        eprintln!("verifying the PIPELINE (no slab given) — weaker; see the module doc");
        let mut keyed: Vec<Keyed> = features.iter().map(row::key_feature).collect();
        row::expand_tag_overflow(&mut keyed);
        keyed.sort_unstable_by_key(row::sort_key);
        row::assign_identities(&mut keyed);
        let book = row::resolve_identities(&mut keyed).expect("identity codebook");
        let rows = keyed.iter().map(|k| row::build_row(k, &tags)).collect();
        (book, rows)
    };
    drop(features);

    // ── Read every row back, using ONLY its bytes and the codebooks. ──
    let mut got: HashMap<u32, Recovered> = HashMap::with_capacity(rows.len());
    for r in &rows {
        let r = *r;
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
        e.cell = tms::morton_to_cell(code);
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

        let Some((want_anchor, want_tags)) = truth.get(&(kind, osm_id)) else {
            missing += 1;
            continue;
        };
        checked += 1;

        if rec.kind != kind {
            kind_bad += 1;
        }

        // Both contracts are now `assert_eq!` on integers. The derived one used
        // to be "within one grid step", because the anchor was an f64 centroid
        // compared against OSM's 1e-7 grid — the wrong grid for a value that was
        // never an OSM coordinate. Deriving in the key's own grid instead makes
        // the anchor a grid point BY CONSTRUCTION, so the tolerance is gone.
        match want_anchor {
            Anchor::Published { lon, lat } => {
                // A node's coordinate IS an OSM grid point, so the round trip
                // reproduces that integer or the key lost it.
                let want = (
                    (lon * tms::OSM_GRID_PER_DEGREE).round() as i32,
                    (lat * tms::OSM_GRID_PER_DEGREE).round() as i32,
                );
                if (rec.lon_e7, rec.lat_e7) == want {
                    node_pos_ok += 1;
                } else {
                    node_pos_bad += 1;
                }
            }
            Anchor::Derived(cell) => {
                // A derived anchor is a cell; the key carries cells; so the
                // check is cell equality, with no float anywhere in it.
                if rec.cell == *cell {
                    derived_pos_ok += 1;
                } else {
                    derived_pos_bad += 1;
                }
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
(cell equality — integer, exact by construction)"
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
