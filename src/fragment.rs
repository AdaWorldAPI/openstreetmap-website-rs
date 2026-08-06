//! The 64k-row fragment — **64k files × 64k rows = 2³² rows = 2 TiB.**
//!
//! A fragment is exactly [`FRAGMENT_ROWS`] rows of [`NODE_ROW_STRIDE`] bytes:
//! **32 MiB**.
//!
//! **This is the workspace's canonical frame, not a choice made here.**
//! lance-graph `.claude/plans/measure-64k-axes-v1.md`: *"Canonical row: 512
//! bytes; canonical frame: 65,536 × 512 B = 32 MiB."* Same figure in
//! `dialectic-engine-v1.md` §258 (*"the 64k column (64k×512 B = 32 MB) is a
//! cache convenience, server-L3-resident"*), and measured in the 1BRC probe
//! lanes I/J at `grid=65536` (`crates/onebrc-probe`) — the same lane
//! `stockfish-rs` cites as the "64×64 gridlake sweet spot". Everything in
//! lancedb across this workspace is on 64k × 512 B; this crate adopts it
//! rather than deriving it.
//!
//! It is also what keeps the layout native: `SoaEnvelope::as_le_bytes` hands a
//! row-strided LE packet, so a 64k-row file is the unit Lance's columnar I/O
//! already reads.
//!
//! # The two addresses
//!
//! | | |
//! |---|---|
//! | **logical** | the 64-bit Morton key — spatial, sorted, computed from `(lon, lat)` |
//! | **physical** | `(fragment: u16, row: u16)` — **one u32**, where the bytes are |
//!
//! [`BoundaryIndex`] is the entire map between them: one `u64` per fragment,
//! **64k × 8 B = 512 KB**, resident. Locating any row on Earth is 16 probes in
//! that array (L2-resident) to pick the fragment, one `mmap`, then 16 probes
//! inside it. No catalog, no manifest, no listing.
//!
//! # Why row-count and not tile aligned
//!
//! Tile-aligned files were measured on the Berlin bake at 0.11 MB … 335 MB
//! depending on depth — ragged, and with no fragment discipline. Fixing the
//! row count fixes the fragment. The cost, stated plainly: fragment boundaries
//! fall at arbitrary Morton codes rather than tile edges, so a tile's range can
//! straddle two fragments. It stays *contiguous* — it just crosses a boundary —
//! and the index resolves which fragments to touch.
//!
//! # Occupancy
//!
//! | | rows | fragments |
//! |---|---|---|
//! | Berlin | 2,526,608 | 39 |
//! | Germany (est.) | ~123 M | ~1,877 |
//! | planet (est.) | ~2.15 B | ~32,800 |
//! | namespace | 4.29 B | 65,536 |
//!
//! The planet is about half the namespace, so there is no scaling cliff.

use lance_graph_contract::canonical_node::NODE_ROW_STRIDE;

/// Rows per fragment. `u16::MAX + 1`, so a row's index within its fragment is
/// exactly a `u16` and the global address is a `u32`.
pub const FRAGMENT_ROWS: usize = 1 << 16;

/// Bytes per full fragment — 32 MiB.
pub const FRAGMENT_BYTES: usize = FRAGMENT_ROWS * NODE_ROW_STRIDE;

/// Fragments addressable by a `u16` id.
pub const MAX_FRAGMENTS: usize = 1 << 16;

const _: () = assert!(FRAGMENT_BYTES == 32 * 1024 * 1024, "a fragment is 32 MiB");
const _: () = assert!(
    (MAX_FRAGMENTS as u64) * (FRAGMENT_ROWS as u64) == 1u64 << 32,
    "64k x 64k = 2^32 rows"
);

/// A row's address: which fragment, which row inside it.
///
/// **The row half is an ownership identity, not merely an offset.** The 1BRC
/// lane-I probe states the correspondence directly: *"mailbox `i` owns row `i`
/// of every 65536-row table by index correspondence"*
/// (`lance-graph/crates/onebrc-probe/src/lane_i.rs`). A `u16` row index is a
/// mailbox ordinal.
///
/// Packs to a `u32` — which is also the shape of
/// `lance_graph_contract::collapse_gate::MailboxId`. Whether those are the
/// same thing is an **open reconciliation upstream**, not something to assume:
/// `le-contract.md` §5 records "MailboxId ≠ NiblePath in code … no conversion,
/// no shared trait, no code comment linking them." Flagged, not unified here.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct RowAddr {
    pub fragment: u16,
    pub row: u16,
}

impl RowAddr {
    /// The global row ordinal this address denotes.
    #[must_use]
    pub const fn to_u32(self) -> u32 {
        ((self.fragment as u32) << 16) | self.row as u32
    }

    /// Split a global row ordinal back into `(fragment, row)`.
    #[must_use]
    pub const fn from_u32(v: u32) -> Self {
        Self {
            fragment: (v >> 16) as u16,
            row: (v & 0xFFFF) as u16,
        }
    }

    /// The address of global row `i`, or `None` past the 2³² namespace.
    #[must_use]
    pub fn of_row(i: usize) -> Option<Self> {
        u32::try_from(i).ok().map(Self::from_u32)
    }
}

/// Deterministic path for a fragment: `{hi:02x}/{id:04x}.soa`.
///
/// Two levels so no directory holds more than 256 entries — 64k files in one
/// directory is fine for an object store, which has no directories, but not for
/// the local filesystem the volume caches them on.
#[must_use]
pub fn fragment_path(id: u16) -> String {
    format!("{:02x}/{:04x}.soa", id >> 8, id)
}

/// One `u64` per fragment: the **first** Morton code it contains.
///
/// Sorted by construction, because the corpus is. 64k entries = 512 KB.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BoundaryIndex {
    firsts: Vec<u64>,
}

/// Wire magic for a serialized [`BoundaryIndex`].
pub const BOUNDARY_MAGIC: [u8; 4] = *b"OSBI";

impl BoundaryIndex {
    /// Build from the first Morton code of each fragment, in fragment order.
    ///
    /// Returns `None` if the codes are not non-decreasing — an unsorted index
    /// would make `locate` silently wrong rather than loudly broken.
    #[must_use]
    pub fn new(firsts: Vec<u64>) -> Option<Self> {
        if firsts.windows(2).any(|w| w[0] > w[1]) {
            return None;
        }
        if firsts.len() > MAX_FRAGMENTS {
            return None;
        }
        Some(Self { firsts })
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.firsts.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.firsts.is_empty()
    }

    /// The fragment that may contain `code`: the last one whose first code is
    /// `<= code`.
    ///
    /// `None` only when `code` precedes the whole corpus. A code past the end
    /// resolves to the final fragment — which may not actually contain it; the
    /// fragment's own search is what confirms. This function narrows, it does
    /// not assert membership.
    #[must_use]
    pub fn locate(&self, code: u64) -> Option<u16> {
        let p = self.firsts.partition_point(|&f| f <= code);
        (p > 0).then(|| (p - 1) as u16)
    }

    /// Every fragment overlapping the half-open Morton interval — what a tile
    /// range needs when it straddles a fragment boundary.
    #[must_use]
    pub fn locate_range(&self, lo: u64, hi: u64) -> core::ops::Range<usize> {
        if self.is_empty() || hi <= lo {
            return 0..0;
        }
        let start = self.locate(lo).map_or(0, usize::from);
        let end = self.firsts.partition_point(|&f| f < hi);
        start..end.max(start + 1).min(self.firsts.len())
    }

    /// LE wire: magic, `u32` count, then the `u64` codes.
    #[must_use]
    pub fn to_le_bytes(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(8 + self.firsts.len() * 8);
        v.extend_from_slice(&BOUNDARY_MAGIC);
        v.extend_from_slice(&(self.firsts.len() as u32).to_le_bytes());
        for f in &self.firsts {
            v.extend_from_slice(&f.to_le_bytes());
        }
        v
    }

    /// Parse the wire form. `None` on bad magic, short buffer, or a
    /// non-monotone body — never a partially-read index.
    #[must_use]
    pub fn from_le_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < 8 || b[..4] != BOUNDARY_MAGIC {
            return None;
        }
        let n = u32::from_le_bytes([b[4], b[5], b[6], b[7]]) as usize;
        if b.len() < 8 + n * 8 {
            return None;
        }
        let firsts = (0..n)
            .map(|i| {
                let o = 8 + i * 8;
                u64::from_le_bytes(b[o..o + 8].try_into().expect("8 bytes"))
            })
            .collect();
        Self::new(firsts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_namespace_is_exactly_two_to_the_thirty_two_rows() {
        assert_eq!(FRAGMENT_ROWS, 65_536);
        assert_eq!(FRAGMENT_BYTES, 33_554_432);
        assert_eq!(MAX_FRAGMENTS as u64 * FRAGMENT_ROWS as u64, 4_294_967_296);
        // 2^32 rows x 512 B = 2 TiB.
        assert_eq!(
            MAX_FRAGMENTS as u64 * FRAGMENT_BYTES as u64,
            2 * 1024 * 1024 * 1024 * 1024
        );
    }

    #[test]
    fn a_row_address_is_a_u32_and_round_trips() {
        for i in [
            0usize,
            1,
            65_535,
            65_536,
            65_537,
            2_526_607,
            u32::MAX as usize,
        ] {
            let a = RowAddr::of_row(i).expect("inside the namespace");
            assert_eq!(a.to_u32() as usize, i);
            assert_eq!(RowAddr::from_u32(a.to_u32()), a);
        }
        // The boundary is real: row 2^32 has no address.
        assert!(RowAddr::of_row(1usize << 32).is_none());
        // ...paired with the last one that does, so the guard discriminates.
        assert!(RowAddr::of_row((1usize << 32) - 1).is_some());
    }

    #[test]
    fn row_65536_is_fragment_1_row_0_not_fragment_0() {
        // The off-by-one that would silently overfill fragment 0.
        assert_eq!(
            RowAddr::of_row(65_536).unwrap(),
            RowAddr {
                fragment: 1,
                row: 0
            }
        );
        assert_eq!(
            RowAddr::of_row(65_535).unwrap(),
            RowAddr {
                fragment: 0,
                row: 65_535
            }
        );
    }

    #[test]
    fn paths_are_computed_and_two_level() {
        assert_eq!(fragment_path(0), "00/0000.soa");
        assert_eq!(fragment_path(0x1234), "12/1234.soa");
        assert_eq!(fragment_path(u16::MAX), "ff/ffff.soa");
        // No directory exceeds 256 entries.
        let dirs: std::collections::BTreeSet<String> = (0u32..=u16::MAX as u32)
            .map(|i| fragment_path(i as u16)[..2].to_string())
            .collect();
        assert_eq!(dirs.len(), 256);
    }

    #[test]
    fn locate_finds_the_last_fragment_at_or_before_the_code() {
        let idx = BoundaryIndex::new(vec![10, 20, 30]).unwrap();
        assert_eq!(idx.locate(9), None, "before the corpus");
        assert_eq!(idx.locate(10), Some(0));
        assert_eq!(idx.locate(19), Some(0));
        assert_eq!(idx.locate(20), Some(1));
        assert_eq!(idx.locate(999), Some(2), "past the end narrows to the last");
    }

    #[test]
    fn an_unsorted_index_is_refused_rather_than_silently_wrong() {
        // locate() is a binary search; an unsorted body would not error, it
        // would answer wrongly. Refuse at construction instead.
        assert!(BoundaryIndex::new(vec![10, 5, 30]).is_none());
        // ...and the guard discriminates: equal neighbours are legal (a
        // fragment can begin on the same code the previous one ended within).
        assert!(BoundaryIndex::new(vec![10, 10, 30]).is_some());
    }

    #[test]
    fn a_straddling_range_names_every_fragment_it_touches() {
        let idx = BoundaryIndex::new(vec![0, 100, 200, 300]).unwrap();
        // Wholly inside one fragment.
        assert_eq!(idx.locate_range(110, 120), 1..2);
        // Straddling a boundary — the case fixed-row fragments introduce.
        assert_eq!(idx.locate_range(150, 250), 1..3);
        // Spanning everything.
        assert_eq!(idx.locate_range(0, u64::MAX), 0..4);
        // Empty interval touches nothing.
        assert_eq!(idx.locate_range(150, 150), 0..0);
    }

    #[test]
    fn the_index_wire_round_trips_and_rejects_garbage() {
        let idx = BoundaryIndex::new(vec![1, 2, 3, 1 << 63]).unwrap();
        let b = idx.to_le_bytes();
        assert_eq!(BoundaryIndex::from_le_bytes(&b).unwrap(), idx);
        assert_eq!(b.len(), 8 + 4 * 8);
        // Bad magic, truncated body, and a non-monotone body are all refused.
        let mut bad = b.clone();
        bad[0] = b'X';
        assert!(BoundaryIndex::from_le_bytes(&bad).is_none());
        assert!(BoundaryIndex::from_le_bytes(&b[..b.len() - 1]).is_none());
        let mut unsorted = idx.clone();
        unsorted.firsts.swap(0, 3);
        assert!(BoundaryIndex::from_le_bytes(&unsorted.to_le_bytes()).is_none());
    }
}
