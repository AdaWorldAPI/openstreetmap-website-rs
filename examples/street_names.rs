//! Proves street names actually resolve out of a real baked artifact —
//! nothing but the slab bytes + the books sidecar, read the way a consumer
//! (the q2 cockpit's `osm_features.rs`) would.
//!
//! ```text
//! cargo run --release --example street_names -- <slab> <slab.books> [count]
//! ```

use lance_graph_contract::canonical_node::NodeRow;
use osm_soa_bake::codebook::read_books;
use osm_soa_bake::{identity, street};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: street_names <slab> <slab.books> [count]");
        std::process::exit(2);
    }
    let count: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(10);

    let slab_bytes = std::fs::read(&args[1]).expect("read slab");
    let mut f = std::fs::File::open(&args[2]).expect("open books");
    let (_header, books) = read_books(&mut f).expect("read books");

    let stride = core::mem::size_of::<NodeRow>();
    assert_eq!(slab_bytes.len() % stride, 0);
    let mut rows = vec![
        NodeRow {
            key: lance_graph_contract::canonical_node::NodeGuid::mint_for(
                lance_graph_contract::canonical_node::TailVariant::V3,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ),
            edges: lance_graph_contract::canonical_node::EdgeBlock::default(),
            value: [0u8; 480],
        };
        slab_bytes.len() / stride
    ];
    // SAFETY: same 512-byte ABI the baker writes; matches `parity`'s own
    // `load_rows_from`.
    let dst = unsafe {
        core::slice::from_raw_parts_mut(rows.as_mut_ptr().cast::<u8>(), rows.len() * stride)
    };
    dst.copy_from_slice(&slab_bytes);

    println!(
        "slab: {} rows · labels book: {} distinct street names",
        rows.len(),
        books.labels.len()
    );
    println!();

    let mut shown = 0usize;
    for r in &rows {
        let Some((kind, ordinal)) = identity::read_identity(r) else {
            continue;
        };
        if kind != osm_soa_bake::read::OSM_NODE {
            continue;
        }
        let names: Vec<u16> = (0..street::EDGE_SLOTS)
            .map(|e| street::edge_name(r, e))
            .filter(|&n| n != street::NAME_NONE)
            .collect();
        if names.is_empty() {
            continue;
        }
        let Some(key) = books.identities.key(ordinal) else {
            continue;
        };
        let Some((_, osm_id)) = key.split_once(':') else {
            continue;
        };
        let resolved: Vec<&str> = names
            .iter()
            .filter_map(|&o| books.labels.key(u32::from(o)))
            .collect();
        println!("junction node {osm_id} (osm_node ordinal {ordinal}) -> {resolved:?}");
        shown += 1;
        if shown >= count {
            break;
        }
    }
    println!();
    println!(
        "{shown} junction rows shown, resolved through nothing but raw bytes + the books sidecar."
    );
}
