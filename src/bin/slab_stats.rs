//! `slab_stats` — what is actually IN the slab, byte for byte.
//!
//! ```text
//! slab_stats <file.soa>
//! ```
//!
//! The slab's raw size is stride, not information: at ~19% slot occupancy most
//! of every row is a reserved zero. This probe measures that directly — how
//! many facet slots are filled, how many value bytes are non-zero, and how the
//! zeros are *distributed* — because the distribution is what decides whether
//! the stride is expensive in practice or only on paper.
//!
//! # Why this exists rather than a compression number in a doc comment
//!
//! An ad-hoc `xz` run is not reproducible from the repo and drifts silently.
//! Compressing here would mean a compression dependency in a crate whose whole
//! point is a light closure, which is not worth it for a probe. So this
//! measures the *mechanism* — occupancy and run structure — in pure Rust, and
//! the compression figures it explains are recorded as observations with their
//! method stated, not asserted as behaviour.
//!
//! Measured on Berlin (2,525,052 rows, LZMA2 preset 6, 200k-row sample
//! extrapolated; codebooks compressed whole):
//!
//! | artifact | raw | compressed |
//! |---|---|---|
//! | slab, row-major | 1,232.9 MB | 39.7 MB (3.22%) |
//! | slab, column-major | 1,232.9 MB | 61.5 MB (4.99%) |
//! | codebooks | 56.0 MB | 4.7 MB (8.39%) |
//! | **total** | **1,288.9 MB** | **44.4 MB** |
//! | the `.osm.pbf` it came from | — | 93.9 MB |
//!
//! **Column-major came out WORSE**, which is the opposite of the usual
//! columnar-compression story and is explained by what this probe measures: the
//! slab's zeros arrive as one long run per row, already contiguous in row-major
//! order, and transposing shatters each run into 2.5 M interleaved fragments.
//! For a *sparse* fixed-stride row, row-major runs are the thing to preserve.
//! (A real Lance file is neither of these — it encodes per column with typed
//! encodings and keeps random access, which a single compressed stream gives up
//! entirely. 44.4 MB is therefore a floor on the information content, **not** a
//! shippable artifact size.)

use lance_graph_contract::canonical_node::NodeRow;
use ogar_osm::{RESERVED_SLOTS, ROW_SLOTS};
use osm_soa_bake::cluster;
use osm_soa_bake::identity::{slab_offset_of_slot, SLOT_BYTES};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: slab_stats <file.soa>");
        std::process::exit(2);
    }

    let bytes = std::fs::read(&args[1]).expect("read slab");
    let stride = core::mem::size_of::<NodeRow>();
    assert_eq!(bytes.len() % stride, 0, "not a whole number of rows");
    let rows = bytes.len() / stride;

    let mut filled_slots = 0u64;
    let mut nonzero_value_bytes = 0u64;
    let mut zero_run_total = 0u64;
    let mut zero_runs = 0u64;
    let mut trailing_zero_total = 0u64;

    for i in 0..rows {
        let row = &bytes[i * stride..(i + 1) * stride];
        let value = &row[32..];

        for slot in RESERVED_SLOTS..ROW_SLOTS {
            let off = slab_offset_of_slot(slot).expect("value slot");
            if value[off..off + SLOT_BYTES].iter().any(|&b| b != 0) {
                filled_slots += 1;
            }
        }

        nonzero_value_bytes += value.iter().filter(|&&b| b != 0).count() as u64;

        // Run structure: how the zeros arrive. One long tail per row is what
        // makes row-major order compress better than a transpose.
        let mut run = 0u64;
        for &b in value {
            if b == 0 {
                run += 1;
            } else if run > 0 {
                zero_runs += 1;
                zero_run_total += run;
                run = 0;
            }
        }
        if run > 0 {
            zero_runs += 1;
            zero_run_total += run;
            trailing_zero_total += run;
        }
    }

    let value_bytes = rows as u64 * (stride as u64 - 32);
    let slots = rows as u64 * (ROW_SLOTS - RESERVED_SLOTS) as u64;

    println!("rows                  {rows:>12}");
    println!(
        "bytes                 {:>12}  ({:.1} MB)",
        bytes.len(),
        bytes.len() as f64 / 1048576.0
    );
    println!(
        "facet slots filled    {filled_slots:>12} / {slots}  ({:.2}%)",
        100.0 * filled_slots as f64 / slots as f64
    );
    println!(
        "value bytes non-zero  {nonzero_value_bytes:>12} / {value_bytes}  ({:.2}%)",
        100.0 * nonzero_value_bytes as f64 / value_bytes as f64
    );
    println!(
        "zero runs             {zero_runs:>12}  (mean length {:.1})",
        zero_run_total as f64 / zero_runs.max(1) as f64
    );
    println!(
        "  of which trailing   {:>12}  ({:.1}% of all zero bytes — the run a transpose shatters)",
        trailing_zero_total,
        100.0 * trailing_zero_total as f64 / zero_run_total.max(1) as f64
    );

    // Anti-vacuity: a slab whose value slab is entirely zero is the defect
    // caught in the codebook commit, and it must be loud rather than reported
    // as excellent occupancy.
    if filled_slots == 0 {
        eprintln!("\nEVERY VALUE SLOT IS EMPTY — this slab carries keys and nothing else");
        std::process::exit(1);
    }
    let sample = cluster::facets(&{
        let mut r = [0u8; 512];
        r.copy_from_slice(&bytes[..stride]);
        // SAFETY: same repr(C, align(64)) argument as `bake`'s writer; the
        // canon defines the row AS its little-endian bytes. Copied into an
        // aligned local first.
        unsafe { core::ptr::read_unaligned(r.as_ptr().cast::<NodeRow>()) }
    });
    println!("first row facets      {:>12}", sample.len());
}
