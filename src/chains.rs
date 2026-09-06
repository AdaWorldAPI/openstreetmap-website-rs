//! The `.chains` sidecar — a tagged way's **vertex chain**, and its exact
//! rehydration.
//!
//! # Why this exists: the encoding already had the data in hand
//!
//! `read.rs`'s way arm builds `cells: Vec<TileXy>` — the full z=32 vertex
//! chain — for EVERY way, in order to compute the derived anchor
//! (`mean_cell`), and then drops it. The slab therefore stored an anchor for a
//! shape whose shape was computed and discarded. This module persists what was
//! already computed; it invents no new reading of the source.
//!
//! # The encode rule and its paired rehydration live HERE, together
//!
//! The lesson is `osm_tiles`' V1/V3 drift (Berlin HEEL `0x624b` vs `0xc8e1`):
//! two implementations of one projection is how they diverge. So the writer
//! ([`write_chains`]) and the reader ([`Chains::get`]) are one module in the
//! bake crate, and a consumer (the q2 cockpit) calls the reader — it never
//! re-interprets the bytes itself. Rendering is the DECODE half of this codec,
//! not a separate design decision.
//!
//! # Format choice is the probes' verdict, not a preference
//!
//! `areal_probe` P6 measured every angle/turn-bit form failing outside
//! buildings (a water ring drifts 8.26 m, five times outside the 1.69 m bar)
//! and concluded *"delta-position varints do [clear it], which is what the PBF
//! already spends."* So a chain is stored as: first vertex absolute, then
//! zigzag-varint `(dx, dy)` cell deltas. Cells are `u32` integers, so the
//! roundtrip falsifier is **exact equality**, not a tolerance.
//!
//! # Wire layout (little-endian throughout)
//!
//! ```text
//! magic        8   b"OSMCHNS1"
//! slab_digest  8   u64 — SlabHasher digest of the slab these chains belong to
//! count        4   u32 — number of chains
//! blob_len     4   u32 — total blob bytes
//! index    count × 12  (ordinal u32, offset u32, len u32), strictly
//!                      ascending by ordinal — binary-searchable as stored
//! blob         …   per chain: n varint, x0 varint, y0 varint,
//!                  then (n-1) × (zigzag dx varint, zigzag dy varint)
//! ```
//!
//! # Scope, stated so the POC's gaps are named rather than discovered
//!
//! Round 1 stores chains for **tagged ways only**. Relations (multipolygon
//! outer/inner rings — a lake with an island) are NOT assembled here; a
//! relation still renders as its anchor. That is the first gap the POC will
//! show, deliberately: the operator's own criterion is that the POC surfaces
//! what the encoding does not yet carry.

use crate::tms::TileXy;
use std::io::{Read, Write};

/// The sidecar magic. Versioned in the name so a layout change is a new magic,
/// never a silent reinterpretation.
pub const MAGIC: [u8; 8] = *b"OSMCHNS1";

/// Everything that can go wrong opening or reading a chains file.
#[derive(Debug, PartialEq, Eq)]
pub enum ChainError {
    /// Not a chains file, or a layout this reader does not speak.
    BadMagic,
    /// The header promises more bytes than the file has.
    Truncated,
    /// Index entries out of order or duplicated — the binary search would lie.
    IndexNotAscending,
    /// An index entry points outside the blob.
    IndexOutOfBounds,
    /// A varint ran past its record or exceeded u64.
    BadVarint,
    /// A delta stepped outside the u32 cell space.
    DeltaOverflow,
    /// The writer was handed two chains for one ordinal.
    DuplicateOrdinal,
}

// ── varint + zigzag primitives ──────────────────────────────────────

fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            out.push(b);
            return;
        }
        out.push(b | 0x80);
    }
}

fn get_varint(bytes: &[u8], pos: &mut usize) -> Result<u64, ChainError> {
    let mut v: u64 = 0;
    let mut shift = 0u32;
    loop {
        let b = *bytes.get(*pos).ok_or(ChainError::BadVarint)?;
        *pos += 1;
        if shift >= 64 {
            return Err(ChainError::BadVarint);
        }
        v |= u64::from(b & 0x7f) << shift;
        if b & 0x80 == 0 {
            return Ok(v);
        }
        shift += 7;
    }
}

fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}

fn unzigzag(v: u64) -> i64 {
    ((v >> 1) as i64) ^ -((v & 1) as i64)
}

// ── encode ──────────────────────────────────────────────────────────

/// Serialize one chain into the blob form.
fn encode_chain(out: &mut Vec<u8>, chain: &[TileXy]) {
    put_varint(out, chain.len() as u64);
    let Some(first) = chain.first() else { return };
    put_varint(out, u64::from(first.x));
    put_varint(out, u64::from(first.y_xyz));
    let mut prev = *first;
    for c in &chain[1..] {
        put_varint(out, zigzag(i64::from(c.x) - i64::from(prev.x)));
        put_varint(out, zigzag(i64::from(c.y_xyz) - i64::from(prev.y_xyz)));
        prev = *c;
    }
}

/// Write the sidecar. `chains` is `(identity ordinal, vertex chain)`; it is
/// sorted here, and a duplicate ordinal is a refusal, not a last-write-wins.
///
/// # Errors
///
/// [`ChainError::DuplicateOrdinal`] on duplicate ordinals; otherwise any I/O
/// failure from `w`.
pub fn write_chains<W: Write>(
    w: &mut W,
    slab_digest: u64,
    chains: &mut [(u32, Vec<TileXy>)],
) -> Result<(), Box<dyn std::error::Error>> {
    chains.sort_unstable_by_key(|(o, _)| *o);
    if chains.windows(2).any(|p| p[0].0 == p[1].0) {
        return Err(Box::new(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "duplicate ordinal in chains",
        )));
    }

    let mut blob = Vec::new();
    let mut index = Vec::with_capacity(chains.len());
    for (ordinal, chain) in chains.iter() {
        let start = blob.len();
        encode_chain(&mut blob, chain);
        index.push((*ordinal, start as u32, (blob.len() - start) as u32));
    }

    w.write_all(&MAGIC)?;
    w.write_all(&slab_digest.to_le_bytes())?;
    w.write_all(&u32::try_from(index.len())?.to_le_bytes())?;
    w.write_all(&u32::try_from(blob.len())?.to_le_bytes())?;
    for (ordinal, off, len) in &index {
        w.write_all(&ordinal.to_le_bytes())?;
        w.write_all(&off.to_le_bytes())?;
        w.write_all(&len.to_le_bytes())?;
    }
    w.write_all(&blob)?;
    Ok(())
}

// ── decode ──────────────────────────────────────────────────────────

/// Read ONLY the 24-byte header — magic, slab digest, chain count, blob
/// length — from any [`Read`] source, without ever reading the index or blob
/// that follow it.
///
/// This exists for a caller that needs to answer "is this sidecar valid for
/// the current slab" (the same [`Chains::slab_digest`] check a full open
/// implicitly enables) without paying [`Chains::from_bytes`]'s cost: that
/// function takes an already fully-`std::fs::read`-in-memory `Vec<u8>`, so
/// even a cheap parse afterward doesn't avoid the expensive READ. Passing a
/// `File` handle here reads exactly 24 bytes off disk — `/api/osm/health` in
/// the q2 cockpit is exactly the caller this is for: a status/diagnostic
/// endpoint has no business eagerly loading the full resident sidecar the
/// way the real serving path does.
///
/// # Errors
///
/// [`ChainError::Truncated`] if fewer than 24 bytes are available (a short
/// read or any I/O failure — this format has no dedicated I/O-error variant,
/// see the module's `ChainError`); [`ChainError::BadMagic`] on a magic
/// mismatch.
pub fn read_chains_header<R: Read>(r: &mut R) -> Result<(u64, usize, usize), ChainError> {
    let mut buf = [0u8; 24];
    r.read_exact(&mut buf).map_err(|_| ChainError::Truncated)?;
    if buf[0..8] != MAGIC {
        return Err(ChainError::BadMagic);
    }
    let slab_digest = u64::from_le_bytes(buf[8..16].try_into().unwrap());
    let count = u32::from_le_bytes(buf[16..20].try_into().unwrap()) as usize;
    let blob_len = u32::from_le_bytes(buf[20..24].try_into().unwrap()) as usize;
    Ok((slab_digest, count, blob_len))
}

/// The chains sidecar, opened for reading. Holds the raw bytes; a chain is
/// decoded on demand ([`Self::get`]), so opening a 50 MB file does not
/// materialise every vertex of every way.
impl std::fmt::Debug for Chains {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Never dump the byte buffer — a Debug on a 50 MB sidecar must not be
        // a 50 MB log line.
        f.debug_struct("Chains")
            .field("slab_digest", &format_args!("{:016x}", self.slab_digest))
            .field("count", &self.count)
            .field("blob_len", &self.blob_len)
            .finish_non_exhaustive()
    }
}

pub struct Chains {
    bytes: Vec<u8>,
    /// Digest of the slab this sidecar belongs to — the consumer compares it
    /// against `codebook::hash_slab` of the slab it actually mapped.
    pub slab_digest: u64,
    count: usize,
    index_at: usize,
    blob_at: usize,
    blob_len: usize,
}

impl Chains {
    /// Parse the header and index bounds. Chain payloads stay undecoded.
    ///
    /// # Errors
    ///
    /// [`ChainError`] on any structural violation — a bad magic, a truncated
    /// file, a non-ascending index, or an index entry outside the blob.
    pub fn from_bytes(bytes: Vec<u8>) -> Result<Self, ChainError> {
        if bytes.len() < 24 || bytes[0..8] != MAGIC {
            return Err(ChainError::BadMagic);
        }
        let slab_digest = u64::from_le_bytes(bytes[8..16].try_into().unwrap());
        let count = u32::from_le_bytes(bytes[16..20].try_into().unwrap()) as usize;
        let blob_len = u32::from_le_bytes(bytes[20..24].try_into().unwrap()) as usize;
        let index_at = 24;
        let blob_at = index_at + count * 12;
        if bytes.len() < blob_at + blob_len {
            return Err(ChainError::Truncated);
        }
        let me = Self {
            bytes,
            slab_digest,
            count,
            index_at,
            blob_at,
            blob_len,
        };
        let mut prev: Option<u32> = None;
        for i in 0..me.count {
            let (ordinal, off, len) = me.entry(i);
            if prev.is_some_and(|p| p >= ordinal) {
                return Err(ChainError::IndexNotAscending);
            }
            prev = Some(ordinal);
            if off as usize + len as usize > me.blob_len {
                return Err(ChainError::IndexOutOfBounds);
            }
        }
        Ok(me)
    }

    /// How many ways carry a chain.
    #[must_use]
    pub fn len(&self) -> usize {
        self.count
    }

    /// True when no chains are stored.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Every stored `(ordinal, raw record bytes)`, in ascending-ordinal
    /// (storage) order — zero-copy, borrowing slices into the backing
    /// buffer rather than decoding each chain into an owned `Vec<TileXy>`.
    ///
    /// This exists for bulk sidecar consumers (a boot-time Lance conversion)
    /// that want the exact on-disk bytes to persist verbatim, not the
    /// decoded geometry — [`Self::get`] remains the entry point for callers
    /// that want a specific ordinal's decoded chain.
    pub fn iter(&self) -> impl Iterator<Item = (u32, &[u8])> + '_ {
        (0..self.count).map(move |i| {
            let (ordinal, off, len) = self.entry(i);
            let rec = &self.bytes[self.blob_at + off as usize..self.blob_at + (off + len) as usize];
            (ordinal, rec)
        })
    }

    fn entry(&self, i: usize) -> (u32, u32, u32) {
        let at = self.index_at + i * 12;
        let b = &self.bytes[at..at + 12];
        (
            u32::from_le_bytes(b[0..4].try_into().unwrap()),
            u32::from_le_bytes(b[4..8].try_into().unwrap()),
            u32::from_le_bytes(b[8..12].try_into().unwrap()),
        )
    }

    /// The vertex chain of the way with this identity ordinal, or `None` when
    /// no chain is stored for it (a node, a relation, or an unresolved way).
    ///
    /// # Errors
    ///
    /// [`ChainError`] when the stored record is malformed — a decode failure
    /// is loud, never an empty chain.
    pub fn get(&self, ordinal: u32) -> Result<Option<Vec<TileXy>>, ChainError> {
        let mut lo = 0usize;
        let mut hi = self.count;
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            match self.entry(mid).0.cmp(&ordinal) {
                std::cmp::Ordering::Less => lo = mid + 1,
                std::cmp::Ordering::Greater => hi = mid,
                std::cmp::Ordering::Equal => {
                    let (_, off, len) = self.entry(mid);
                    let rec = &self.bytes
                        [self.blob_at + off as usize..self.blob_at + (off + len) as usize];
                    return decode_chain(rec).map(Some);
                }
            }
        }
        Ok(None)
    }
}

/// Decode one chain's raw record bytes (as yielded by [`Chains::iter`] or
/// read back from a consumer's own copy of the bytes, e.g. after a
/// round-trip through Lance) into its vertex chain. `Chains::get` uses this
/// internally; it is `pub` so a consumer that stores raw records verbatim
/// (never re-encoding) can decode them through the SAME function, per this
/// module's "one codec, one place" doc rule.
///
/// # Errors
///
/// [`ChainError`] when `rec` is not a validly-encoded chain record.
pub fn decode_chain(rec: &[u8]) -> Result<Vec<TileXy>, ChainError> {
    let mut pos = 0usize;
    let n = get_varint(rec, &mut pos)? as usize;
    let mut out = Vec::with_capacity(n);
    if n == 0 {
        return Ok(out);
    }
    let x0 = u32::try_from(get_varint(rec, &mut pos)?).map_err(|_| ChainError::DeltaOverflow)?;
    let y0 = u32::try_from(get_varint(rec, &mut pos)?).map_err(|_| ChainError::DeltaOverflow)?;
    out.push(TileXy { x: x0, y_xyz: y0 });
    let (mut x, mut y) = (i64::from(x0), i64::from(y0));
    for _ in 1..n {
        x += unzigzag(get_varint(rec, &mut pos)?);
        y += unzigzag(get_varint(rec, &mut pos)?);
        let (cx, cy) = (
            u32::try_from(x).map_err(|_| ChainError::DeltaOverflow)?,
            u32::try_from(y).map_err(|_| ChainError::DeltaOverflow)?,
        );
        out.push(TileXy { x: cx, y_xyz: cy });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(x: u32, y: u32) -> TileXy {
        TileXy { x, y_xyz: y }
    }

    /// A ring that walks all four delta-sign quadrants, so zigzag is exercised
    /// on genuinely negative steps — a monotone fixture would pass with plain
    /// (wrong) unsigned deltas too.
    fn ring() -> Vec<TileXy> {
        vec![
            c(1_000_000, 2_000_000),
            c(1_000_500, 2_000_010),
            c(1_000_490, 2_000_600),
            c(999_900, 2_000_580),
            c(999_910, 2_000_005),
            c(1_000_000, 2_000_000),
        ]
    }

    fn build(chains: Vec<(u32, Vec<TileXy>)>) -> Vec<u8> {
        let mut chains = chains;
        let mut buf = Vec::new();
        write_chains(&mut buf, 0xDEAD_BEEF_CAFE_F00D, &mut chains).expect("write");
        buf
    }

    #[test]
    fn roundtrip_is_exact_including_negative_deltas() {
        let buf = build(vec![(7, ring()), (42, vec![c(5, 5)]), (100, ring())]);
        let ch = Chains::from_bytes(buf).expect("parse");
        assert_eq!(ch.slab_digest, 0xDEAD_BEEF_CAFE_F00D);
        assert_eq!(ch.len(), 3);
        // Exact equality — integer cells, no tolerance.
        assert_eq!(ch.get(7).unwrap().unwrap(), ring());
        assert_eq!(ch.get(42).unwrap().unwrap(), vec![c(5, 5)]);
        assert_eq!(ch.get(100).unwrap().unwrap(), ring());
        // Anti-vacuity: the fixture really does step negatively on both axes.
        let r = ring();
        assert!(r.windows(2).any(|w| w[1].x < w[0].x));
        assert!(r.windows(2).any(|w| w[1].y_xyz < w[0].y_xyz));
    }

    /// `read_chains_header` must agree with `Chains::from_bytes`'s own
    /// parsed fields exactly, since a caller (q2's `/api/osm/health`) uses
    /// it as a cheap stand-in for the same digest check a full open enables.
    #[test]
    fn read_chains_header_agrees_with_from_bytes() {
        let buf = build(vec![(7, ring()), (100, vec![c(1, 1)])]);
        let full = Chains::from_bytes(buf.clone()).expect("from_bytes");

        let (slab_digest, count, blob_len) =
            read_chains_header(&mut buf.as_slice()).expect("read_chains_header");
        assert_eq!(slab_digest, full.slab_digest);
        assert_eq!(count, full.len());
        assert_eq!(blob_len, full.blob_len);
    }

    /// The falsifier that actually matters: it must succeed on a source
    /// truncated right after the 24-byte header — proving it never reads the
    /// index or blob. `Chains::from_bytes` on the same truncated bytes must
    /// fail, which is the contrast that shows this is doing meaningfully
    /// less work, not just wrapping the same call.
    #[test]
    fn read_chains_header_never_reads_past_the_header_even_when_truncated() {
        let full = build(vec![(7, ring()), (100, vec![c(1, 1)])]);
        assert!(
            full.len() > 24,
            "fixture must actually have body bytes past the header"
        );
        let truncated = &full[..24];

        let (slab_digest, count, blob_len) =
            read_chains_header(&mut { truncated }).expect("header-only read must succeed");
        assert_eq!(slab_digest, 0xDEAD_BEEF_CAFE_F00D);
        assert_eq!(count, 2);
        assert!(blob_len > 0);

        let err = Chains::from_bytes(truncated.to_vec()).expect_err("full parse must fail");
        assert_eq!(err, ChainError::Truncated);
    }

    /// A short source (fewer than 24 bytes) must report `Truncated`, not
    /// panic or silently return zeroed fields.
    #[test]
    fn read_chains_header_reports_truncated_on_a_too_short_source() {
        let short = [0u8; 10];
        let err = read_chains_header(&mut { &short[..] }).expect_err("must fail");
        assert_eq!(err, ChainError::Truncated);
    }

    /// A malformed magic must be refused identically by both readers — the
    /// cheap path must not accept what the full path would reject.
    #[test]
    fn read_chains_header_rejects_bad_magic_the_same_way_from_bytes_does() {
        let mut buf = build(vec![(7, ring())]);
        buf[0] = b'X';

        let full_err = Chains::from_bytes(buf.clone()).expect_err("full parse must reject");
        let header_err =
            read_chains_header(&mut buf.as_slice()).expect_err("header read must reject");
        assert_eq!(full_err, ChainError::BadMagic);
        assert_eq!(header_err, ChainError::BadMagic);
    }

    /// `iter()` must yield every stored ordinal, in ascending order, and its
    /// raw bytes must decode to EXACTLY what `get()` returns for that same
    /// ordinal — proving the zero-copy slice and the decode-on-demand path
    /// agree, not just that both compile.
    #[test]
    fn iter_yields_raw_records_that_decode_to_the_same_chains_as_get() {
        let ch = Chains::from_bytes(build(vec![(100, ring()), (7, vec![c(5, 5)]), (42, ring())]))
            .unwrap();

        let collected: Vec<(u32, Vec<TileXy>)> = ch
            .iter()
            .map(|(ordinal, raw)| (ordinal, decode_chain(raw).expect("decode")))
            .collect();

        // Ascending storage order (write_chains sorts by ordinal), not
        // insertion order — the anti-vacuity check that iter() isn't just
        // echoing the input Vec unchanged.
        assert_eq!(
            collected.iter().map(|(o, _)| *o).collect::<Vec<_>>(),
            vec![7, 42, 100]
        );
        assert_eq!(collected[0].1, vec![c(5, 5)]);
        assert_eq!(collected[1].1, ring());
        assert_eq!(collected[2].1, ring());

        // And each entry matches get() for the same ordinal — two different
        // read paths over the same stored bytes must agree.
        for (ordinal, decoded) in &collected {
            assert_eq!(ch.get(*ordinal).unwrap().as_ref(), Some(decoded));
        }
    }

    /// Empty sidecar: iter() must yield nothing, not panic on an empty
    /// index/blob.
    #[test]
    fn iter_on_an_empty_chains_file_yields_nothing() {
        let ch = Chains::from_bytes(build(vec![])).unwrap();
        assert_eq!(ch.iter().count(), 0);
    }

    #[test]
    fn absent_ordinal_is_none_not_an_error() {
        let ch = Chains::from_bytes(build(vec![(7, ring())])).unwrap();
        assert_eq!(ch.get(6).unwrap(), None);
        assert_eq!(ch.get(8).unwrap(), None);
    }

    #[test]
    fn duplicate_ordinal_is_refused_at_write() {
        let mut chains = vec![(7, ring()), (7, vec![c(1, 1)])];
        let mut buf = Vec::new();
        assert!(write_chains(&mut buf, 1, &mut chains).is_err());
    }

    /// Can-fire half of the falsifier: corrupt one payload byte and the decode
    /// must CHANGE or FAIL — never silently return the original chain. Without
    /// this, the roundtrip test cannot distinguish a working codec from one
    /// that ignores its input.
    #[test]
    fn a_corrupted_record_does_not_decode_to_the_original() {
        let buf = build(vec![(7, ring())]);
        let ch = Chains::from_bytes(buf.clone()).unwrap();
        let good = ch.get(7).unwrap().unwrap();
        let blob_at = ch.blob_at;
        let mut evil = buf;
        evil[blob_at + 3] ^= 0x55; // inside the first record's varints
        match Chains::from_bytes(evil).unwrap().get(7) {
            Err(_) => {}                                    // loud failure: fine
            Ok(decoded) => assert_ne!(decoded, Some(good)), // or a different chain
        }
    }

    #[test]
    fn structural_violations_are_loud() {
        assert_eq!(
            Chains::from_bytes(b"NOTCHAIN".to_vec()).unwrap_err(),
            ChainError::BadMagic
        );
        let mut buf = build(vec![(7, ring())]);
        buf.truncate(30);
        assert_eq!(Chains::from_bytes(buf).unwrap_err(), ChainError::Truncated);
    }
}
