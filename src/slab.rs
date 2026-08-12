//! The row slab — **ABI, not serialization.**
//!
//! A baked file IS `&[NodeRow]`: `#[repr(C, align(64))]`, stride 512, little
//! endian. Reading it is one cast, not a decode. There is no Arrow gather, no
//! DataFusion plan, no serde. `lance-graph-contract` supplies the canonical
//! types at **zero runtime dependency cost** — its only deps (`serde`, `glob`)
//! are `[build-dependencies]`.
//!
//! The distinction that matters is *format vs engine*. Lance as a storage
//! format is sanctioned — the canon's own words, on `NodeRowPacket`: "a
//! `&[NodeRow]` IS already a row-strided LE packet at stride 512 — no
//! allocation, no copy … so Lance's columnar I/O reads it directly." A query
//! engine on the lookup path is not: it plans and materialises what arithmetic
//! already answers.
//!
//! # Lookup, not search
//!
//! A tile's rows are a **contiguous range**, because the file is Morton-sorted
//! and Morton prefix ≡ tile containment. Both endpoints are *computed*:
//!
//! ```text
//! (z, x, y) → TMS flip → Morton prefix → [prefix << s, (prefix+1) << s)
//! ```
//!
//! Only the offsets of those two computed endpoints are found by binary
//! search, and the tile-per-file layout shrinks even that to the file itself.
//! Nothing is planned, nothing is gathered, nothing is decoded.
//!
//! The same property is what `stockfish-rs` measures on the board: its
//! `ButterflyHistory` is `[from·64 + to]`, a **computed index into a flat
//! array** — no hash, no search — and `examples/morton_ka.rs` pins HalfKA
//! square addressing as a *Morton-preserved gridlake* (nearest-in-Morton ⇒
//! nearest-in-feature-space). Chess measures it over 4096 squares; this
//! measures it over 2^64.

use core::ops::Range;

use lance_graph_contract::canonical_node::{NodeRow, NODE_ROW_STRIDE};

use crate::tms::{morton64, xyz_to_tms_y, HHTL_DEPTH4};

/// A borrowed, Morton-sorted run of baked rows.
///
/// Holds a `&[u8]` — typically an mmap — and never owns or copies it. Not
/// `Clone`/`Copy`: a borrow that duplicates itself silently can outlive the
/// compartment that owns the bytes, which is the same reasoning that stripped
/// the derive from `NodeRowPacket` upstream.
pub struct RowSlab<'a> {
    bytes: &'a [u8],
}

// Debug, but deliberately NOT Clone/Copy — a borrow that duplicates itself
// silently can be carried out of the compartment that owns the bytes. Prints
// the shape, never the bytes (the same shape `NodeRowPacket` prints upstream).
impl core::fmt::Debug for RowSlab<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RowSlab")
            .field("n_rows", &self.len())
            .field("row_stride", &NODE_ROW_STRIDE)
            .finish()
    }
}

/// Why a byte range could not be viewed as rows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlabError {
    /// Length is not a whole number of 512-byte rows.
    NotRowAligned { len: usize },
    /// The base pointer does not meet `NodeRow`'s 64-byte alignment.
    ///
    /// **No longer returned by [`RowSlab::new`]** — alignment stopped being a
    /// construction precondition when the lookup path moved off the cast (see
    /// `new`'s doc). It survives as the vocabulary a CONSUMER uses to say why
    /// [`RowSlab::rows`] returned `None`, which is where the requirement
    /// actually lives now.
    Misaligned { align: usize },
}

impl<'a> RowSlab<'a> {
    /// Wrap a byte range. Validates **stride only**; copies nothing.
    ///
    /// # Why alignment is NOT checked here (changed deliberately)
    ///
    /// An earlier version demanded `NodeRow`'s 64-byte alignment at
    /// construction. That made the whole slab unusable over a buffer a real
    /// producer hands back: **Lance's read path gives no alignment guarantee**
    /// — 8-byte aligned, sometimes 64 by luck of allocator state. Measured
    /// against a real dataset before that measurement's module was removed;
    /// the finding survives the module because it is about arrow and Lance,
    /// not about anything this crate wrote.
    ///
    /// The check was in the wrong place. **Nothing the slab does for a LOOKUP
    /// needs the cast:** [`morton_at`](Self::morton_at) reads bytes at computed
    /// offsets, so [`lower_bound`](Self::lower_bound) and
    /// [`tile_range`](Self::tile_range) are byte arithmetic end to end. Only
    /// [`rows`](Self::rows) — the `&[NodeRow]` projection — needs alignment,
    /// and that is where the check now lives.
    ///
    /// This is what makes the borrow unconditional. Refusing here would have
    /// left a caller with one lawful option, a copy — and zero copy is a law
    /// without escape hatches, so a refusal that forces a copy is not a safe
    /// default, it is the violation wearing a guard's clothes.
    pub fn new(bytes: &'a [u8]) -> Result<Self, SlabError> {
        if !bytes.len().is_multiple_of(NODE_ROW_STRIDE) {
            return Err(SlabError::NotRowAligned { len: bytes.len() });
        }
        Ok(Self { bytes })
    }

    /// Number of rows.
    #[must_use]
    pub fn len(&self) -> usize {
        self.bytes.len() / NODE_ROW_STRIDE
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// The rows, as rows — one cast, still zero copy, but **conditional**.
    ///
    /// Returns `None` when the buffer is not 64-byte aligned. That is not a
    /// failure to work around by copying: it means this particular projection
    /// is unavailable over this buffer, while every LOOKUP operation
    /// ([`morton_at`](Self::morton_at), [`lower_bound`](Self::lower_bound),
    /// [`tile_range`](Self::tile_range)) remains available and borrowed,
    /// because they read bytes rather than cast.
    ///
    /// An **empty** slab returns `Some(&[])`: an empty slice carries a dangling
    /// pointer (`0x1` for `u8`), so demanding alignment of it would reject the
    /// legitimate zero-row case.
    #[must_use]
    pub fn rows(&self) -> Option<&'a [NodeRow]> {
        // The empty case returns without constructing a slice: an empty range
        // carries a dangling pointer, and `from_raw_parts` requires a properly
        // aligned pointer *even at length 0*. Passing `0x1` would be UB.
        if self.bytes.is_empty() {
            return Some(&[]);
        }
        if !(self.bytes.as_ptr() as usize).is_multiple_of(core::mem::align_of::<NodeRow>()) {
            return None;
        }
        // SAFETY: `new` proved the length is a whole number of 512-byte rows
        // and the base meets `NodeRow`'s alignment. `NodeRow` is
        // `#[repr(C, align(64))]` with a compile-asserted 512-byte size and no
        // padding-dependent semantics — the canon defines the row AS its
        // little-endian bytes.
        Some(unsafe {
            core::slice::from_raw_parts(self.bytes.as_ptr().cast::<NodeRow>(), self.len())
        })
    }

    /// The borrowed bytes themselves.
    ///
    /// Exists so a caller can prove the slab BORROWS — pointer identity is the
    /// only way to tell a borrow from a well-optimised copy, and
    /// [`rows`](Self::rows) cannot serve that purpose because it may decline.
    #[must_use]
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Row `i`'s Morton code, read from its key's four cascade tiers.
    ///
    /// Read from the key bytes rather than through a tail accessor so it is
    /// independent of which `TailVariant` the classid resolves to.
    ///
    /// Reads the key BYTES at a computed offset rather than going through
    /// [`rows`](Self::rows). That is not a micro-optimisation: it is what keeps
    /// the entire lookup path free of `NodeRow`'s 64-byte alignment
    /// requirement, so a Lance-returned buffer is borrowed rather than copied.
    /// The bytes read are identical either way — the key's cascade tiers sit at
    /// offsets 4..12 of the row, by the canon's own layout.
    #[must_use]
    pub fn morton_at(&self, i: usize) -> u64 {
        let b = &self.bytes[i * NODE_ROW_STRIDE..];
        let t = |o: usize| u64::from(u16::from_le_bytes([b[o], b[o + 1]]));
        (t(4) << 48) | (t(6) << 32) | (t(8) << 16) | t(10)
    }

    /// Index of the first row whose Morton code is `>= code`.
    ///
    /// The only search in the whole path, and the tile-per-file layout reduces
    /// even this to a few probes inside one small file.
    #[must_use]
    pub fn lower_bound(&self, code: u128) -> usize {
        let (mut lo, mut hi) = (0usize, self.len());
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if u128::from(self.morton_at(mid)) < code {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// The **contiguous row range** covering slippy tile `z/x/y`.
    ///
    /// `x`/`y` are OSM-XYZ as a slippy client sends them; the TMS Y-flip
    /// happens here, at the ingest boundary, per the Q3 ruling — the runtime
    /// sees one key.
    #[must_use]
    pub fn tile_range(&self, z: u32, x: u32, y: u32) -> Range<usize> {
        let (lo, hi) = morton_bounds(z, x, y);
        self.lower_bound(lo)..self.lower_bound(hi)
    }
}

/// The half-open Morton interval a slippy tile covers, as `u128` so the
/// `z == 0` end (`1 << 64`) and the `z == 32` shift are both representable
/// without a special case.
#[must_use]
pub fn morton_bounds(z: u32, x: u32, y: u32) -> (u128, u128) {
    let z = z.min(HHTL_DEPTH4);
    let n = 1u64 << z;
    let (x, y) = (
        u64::from(x).min(n - 1) as u32,
        u64::from(y).min(n - 1) as u32,
    );
    let prefix = u128::from(morton64(x, xyz_to_tms_y(z, y)));
    let shift = 2 * (HHTL_DEPTH4 - z);
    (prefix << shift, (prefix + 1) << shift)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::row::{build_row_notags, Keyed};
    use crate::tms::tiers_of;

    /// Build a slab from Morton codes, laid out exactly as the baker writes.
    fn slab_of(codes: &[u64]) -> Vec<u8> {
        let mut v = Vec::with_capacity(codes.len() * NODE_ROW_STRIDE);
        for &c in codes {
            let k = Keyed {
                morton: c,
                tiers: tiers_of(c),
                entity_type: crate::read::OSM_NODE,
                osm_id: 0,
                identity_ordinal: None,
                identity: 0,
                tags: crate::tags::TagSpan::default(),
                edge_names: crate::row::EdgeNames::default(),
            };
            let r = build_row_notags(&k);
            // SAFETY: same 512-byte ABI the baker writes.
            let b: &[u8; 512] = unsafe { &*(std::ptr::addr_of!(r).cast()) };
            v.extend_from_slice(b);
        }
        v
    }

    /// 64-byte-aligned backing, since `NodeRow` requires it and a `Vec<u8>`
    /// only guarantees 1.
    #[repr(align(64))]
    struct Aligned([u8; 512 * 64]);

    fn aligned(bytes: &[u8]) -> Box<Aligned> {
        let mut a = Box::new(Aligned([0u8; 512 * 64]));
        a.0[..bytes.len()].copy_from_slice(bytes);
        a
    }

    #[test]
    fn a_slab_is_a_cast_not_a_decode() {
        let raw = slab_of(&[1, 2, 3]);
        let a = aligned(&raw);
        let s = RowSlab::new(&a.0[..raw.len()]).unwrap();
        assert_eq!(s.len(), 3);
        assert_eq!(s.rows().expect("aligned").len(), 3);
        // The cast lands on the same bytes the file holds — no copy, no decode.
        assert_eq!(
            s.rows().expect("aligned").as_ptr() as usize,
            a.0.as_ptr() as usize
        );
        assert_eq!(s.morton_at(1), 2);
    }

    #[test]
    fn an_empty_slab_is_valid_and_is_not_alignment_checked() {
        // An empty slice's pointer is dangling (`0x1` for u8), so it is NOT
        // 64-aligned. A naive alignment check rejects the legitimate zero-row
        // case; and once accepted, `rows()` must not hand that dangling
        // pointer to `from_raw_parts`, which demands alignment even at len 0.
        let empty: &[u8] = &[];
        assert_ne!(
            (empty.as_ptr() as usize) % core::mem::align_of::<NodeRow>(),
            0,
            "precondition: an empty slice dangles rather than being aligned"
        );
        let s = RowSlab::new(empty).expect("an empty slab is valid");
        assert_eq!(s.len(), 0);
        assert!(s.is_empty());
        assert!(s.rows().expect("empty is always projectable").is_empty());
        // And a query over it answers empty rather than panicking.
        assert!(s.tile_range(14, 8802, 5373).is_empty());
        // The same via an empty Vec, which is how a zero-row file reads.
        let v: Vec<u8> = Vec::new();
        assert!(RowSlab::new(&v).is_ok());
    }

    #[test]
    fn a_ragged_or_misaligned_range_is_refused_not_cast() {
        // Casting either would be UB; both must be errors, and the guard has
        // to discriminate — a well-formed range still succeeds.
        let raw = slab_of(&[1]);
        let a = aligned(&raw);
        assert_eq!(
            RowSlab::new(&a.0[..500]).unwrap_err(),
            SlabError::NotRowAligned { len: 500 }
        );
        // An 8-offset sub-slice is NOT 64-aligned. It used to be refused at
        // construction; now it CONSTRUCTS — and that is the point, because the
        // lookup path does not need the cast. What it cannot do is hand out
        // `&[NodeRow]`, and both halves are asserted so neither can silently
        // change.
        let unaligned = RowSlab::new(&a.0[8..8 + 512]).expect("construction no longer gates");
        assert_eq!(unaligned.len(), 1, "the slab is usable");
        assert!(
            unaligned.rows().is_none(),
            "…but the &[NodeRow] projection is unavailable over an unaligned buffer"
        );
        // Morton arithmetic works anyway — the whole reason for the change.
        let _ = unaligned.morton_at(0);

        let ok = RowSlab::new(&a.0[..512]).expect("aligned");
        assert!(
            ok.rows().is_some(),
            "an aligned buffer keeps the projection"
        );
    }

    #[test]
    fn a_tile_is_a_contiguous_range_and_its_bounds_are_computed() {
        // Codes chosen so tile (1, 0, 0) — the TMS-flipped top-left quadrant —
        // owns a known middle run. The point is that the endpoints come from
        // arithmetic, and the rows between them are one slice.
        let (lo, hi) = morton_bounds(1, 0, 0);
        let inside = (lo as u64) + 5;
        let codes = [0u64, inside, inside + 1, u64::MAX];
        let mut sorted = codes;
        sorted.sort_unstable();
        let raw = slab_of(&sorted);
        let a = aligned(&raw);
        let s = RowSlab::new(&a.0[..raw.len()]).unwrap();
        let r = s.tile_range(1, 0, 0);
        for i in r.clone() {
            let m = u128::from(s.morton_at(i));
            assert!(m >= lo && m < hi, "row {i} escaped the computed bounds");
        }
        assert_eq!(r.len(), 2, "exactly the two rows inside the tile");
    }

    #[test]
    fn the_whole_world_is_every_row_and_a_far_tile_is_none() {
        // Can-fire paired with can-stay-silent: z=0 must cover everything,
        // and a tile with no rows must return an EMPTY range rather than a
        // clamped or wrapped one.
        let raw = slab_of(&[1, 1 << 40, 1 << 60]);
        let a = aligned(&raw);
        let s = RowSlab::new(&a.0[..raw.len()]).unwrap();
        assert_eq!(s.tile_range(0, 0, 0).len(), 3);
        let (lo, hi) = morton_bounds(32, 0, 0);
        assert!(hi > lo, "a z=32 tile is a non-empty interval");
        assert!(s.tile_range(32, 12345, 12345).is_empty());
    }

    #[test]
    fn deeper_tiles_nest_inside_their_parent() {
        // Prefix containment is the property the whole layout rests on: a
        // child's interval must sit inside its parent's.
        let (plo, phi) = morton_bounds(8, 137, 83);
        for (cx, cy) in [(274u32, 166u32), (275, 166), (274, 167), (275, 167)] {
            let (clo, chi) = morton_bounds(9, cx, cy);
            assert!(
                clo >= plo && chi <= phi,
                "child ({cx},{cy}) escaped its parent"
            );
        }
    }
}
