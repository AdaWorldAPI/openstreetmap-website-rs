//! `bake` — `.osm.pbf` → a Morton-sorted slab of 512-byte V3 ABI rows.
//!
//! ```text
//! bake <input.osm.pbf> <output.soa>
//! ```
//!
//! The output is the raw row slab at stride 512, little-endian — which IS what
//! `SoaEnvelope::as_le_bytes` hands to Lance's columnar I/O. Wrapping it in a
//! Lance dataset is the next step and changes no byte of this file.
//!
//! Rows are sorted by Morton code, so the file is in trie order: a tile prefix
//! is a contiguous row range.

use std::io::Write;

use osm_soa_bake::codebook::{write_books, Books, Header, SlabHasher};
use osm_soa_bake::{read, row, tms};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: bake <input.osm.pbf> <output.soa>");
        std::process::exit(2);
    }
    let (input, output) = (&args[1], &args[2]);

    let t0 = std::time::Instant::now();
    let (features, tag_store, stats) = match read::read_features(input) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("read failed: {e}");
            std::process::exit(1);
        }
    };
    eprintln!("read in {:.1}s", t0.elapsed().as_secs_f64());

    // ── Resolve the tag codebooks, once, before anything is written. ──
    //
    // The bake does not yet WRITE tags into rows — `cluster` is the storage
    // shape and its write path is the next step. Resolving here anyway is not
    // ceremony: it is the only place the `u24` capacity bound is exercised, and
    // it must fail before an output file exists rather than after.
    let t_tags = std::time::Instant::now();
    let tags = match tag_store.resolve() {
        Ok(t) => t,
        Err(e) => {
            eprintln!("tag codebook failed: {e:?}");
            std::process::exit(1);
        }
    };
    eprintln!(
        "tag codebooks resolved in {:.1}s (keys digest {:016x}, values digest {:016x})",
        t_tags.elapsed().as_secs_f64(),
        tags.keys.digest(),
        tags.values.digest(),
    );

    // ── Key, sort into trie order, number collisions. ──
    let t1 = std::time::Instant::now();
    let mut keyed: Vec<row::Keyed> = features.iter().map(row::key_feature).collect();
    drop(features);
    let continuations = row::expand_tag_overflow(&mut keyed);
    keyed.sort_unstable_by_key(row::sort_key);
    let collisions = row::assign_identities(&mut keyed);
    let identities = match row::resolve_identities(&mut keyed) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("identity codebook failed: {e:?}");
            std::process::exit(1);
        }
    };
    eprintln!(
        "keyed + sorted {} rows in {:.1}s",
        keyed.len(),
        t1.elapsed().as_secs_f64()
    );

    // ── Emit. Streamed, so peak memory is the keyed vec, not the slab. ──
    //
    // The slab goes to a TEMP path and is renamed only after the sidecar
    // exists. The sidecar has to carry the slab's digest, so it cannot be
    // written first — but a slab without its books is unreadable, so it must
    // not appear first either. Writing to `.tmp` and renaming last satisfies
    // both: a complete slab only ever exists once its books do.
    let t2 = std::time::Instant::now();
    let tmp = format!("{output}.tmp");
    let mut hasher = SlabHasher::new();
    let mut slots_written = 0u64;
    {
        let f = std::fs::File::create(&tmp).expect("create output");
        let mut w = std::io::BufWriter::with_capacity(1 << 22, f);
        for k in &keyed {
            let r = row::build_row(k, &tags);
            slots_written += osm_soa_bake::cluster::filled_slots(&r) as u64;
            // SAFETY: NodeRow is #[repr(C, align(64))] with a compile-asserted
            // 512-byte size and no padding-dependent semantics; the canon defines
            // the row AS its little-endian bytes.
            let bytes: &[u8; 512] = unsafe { &*(std::ptr::addr_of!(r).cast()) };
            hasher.eat(bytes);
            w.write_all(bytes).expect("write row");
        }
        w.flush().expect("flush");
    }
    let bytes = keyed.len() as u64 * 512;

    // The occupancy guard, as a REFUSAL rather than a report. A bake that
    // filled no slot produced a slab of keys and nothing else — the defect that
    // survived undetected because in-process verification resolved identities
    // the bake never did. Recording zero and comparing zero to zero would agree;
    // stopping is what makes it impossible.
    if slots_written == 0 {
        eprintln!(
            "refusing to emit: {} rows, ZERO facet slots filled — the pipeline \
             produced keys and nothing else",
            keyed.len()
        );
        let _ = std::fs::remove_file(&tmp);
        std::process::exit(1);
    }

    // ── The codebooks, with the header that pins them to this slab. ──
    let books_path = format!("{output}.books");
    {
        let f = std::fs::File::create(&books_path).expect("create codebook sidecar");
        let mut w = std::io::BufWriter::new(f);
        write_books(
            &mut w,
            &Header {
                rows: keyed.len() as u64,
                slots_written,
                slab: hasher.finish(),
                rounding: tms::AnchorRounding::CURRENT,
            },
            &Books {
                identities,
                tag_keys: tags.keys.clone(),
                tag_values: tags.values.clone(),
            },
        )
        .expect("write codebooks");
        w.flush().expect("flush codebooks");
    }
    std::fs::rename(&tmp, output).expect("publish slab");

    // ── Report. Every number measured from what was emitted. ──
    let uniq_heel = {
        let mut v: Vec<u16> = keyed.iter().map(|k| k.tiers.heel).collect();
        v.dedup();
        v.len()
    };
    eprintln!("wrote in {:.1}s", t2.elapsed().as_secs_f64());
    println!("input                 {input}");
    println!("output                {output}");
    println!("codebooks             {books_path}");
    println!("nodes indexed         {:>12}", stats.nodes_indexed);
    println!("tagged nodes          {:>12}", stats.tagged_nodes);
    println!("tagged ways           {:>12}", stats.tagged_ways);
    println!("relations anchored    {:>12}", stats.relations_anchored);
    println!("  unanchorable        {:>12}", stats.relations_unanchorable);
    println!("  turn restrictions   {:>12}", stats.restrictions);
    println!("    via node (exact)  {:>12}", stats.restrictions_via_node);
    println!("    via way           {:>12}", stats.restrictions_via_way);
    println!("    no via (centroid) {:>12}", stats.restrictions_no_via);
    println!("ways unresolved       {:>12}", stats.ways_unresolved);
    println!("tag pairs             {:>12}", stats.tag_pairs);
    println!("  distinct keys       {:>12}", stats.distinct_tag_keys);
    println!("  distinct values     {:>12}", stats.distinct_tag_values);
    println!("continuation rows     {:>12}", continuations);
    println!("facet slots written   {:>12}", slots_written);
    println!("slab digest           {:>12x}", hasher.finish());
    println!("ROWS                  {:>12}", keyed.len());
    println!(
        "bytes                 {:>12}  ({:.2} GiB at stride 512)",
        bytes,
        bytes as f64 / (1024.0 * 1024.0 * 1024.0)
    );
    println!(
        "same-tile collisions  {:>12}  ({:.4}% — z={} keying)",
        collisions,
        100.0 * collisions as f64 / keyed.len().max(1) as f64,
        tms::HHTL_DEPTH4
    );
    println!("distinct HEEL runs    {:>12}", uniq_heel);
    println!("classid               0x{:08X}", row::CLASSID_GEO_V3);
    println!("total                 {:.1}s", t0.elapsed().as_secs_f64());
}
