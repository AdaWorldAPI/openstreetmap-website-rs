//! The identity facet — a **consumer** of `lance_graph_contract::identity_quad`.
//!
//! # This module was rewritten after a real mistake
//!
//! An earlier version implemented its own 12-byte identity payload: a raw
//! `i64` `osm_id` stored as `LegacyOutlier::WideQuad` (G3, `3 × u32`), with its
//! own read/write pair. That was a **parallel implementation of
//! `identity_quad`**, which had merged upstream (lance-graph PR #902) as the
//! sanctioned home for exactly this.
//!
//! Worse, `read.rs` already said so — *"raw ids need 34 bits; the
//! identity-quad codebook is the sanctioned home — lance-graph PR #902"* — so
//! the pointer existed and was contradicted rather than followed. Recorded
//! here because a rewritten file otherwise erases the reason it was rewritten.
//!
//! # What the sanctioned design actually is
//!
//! Not "store the external id". The external key resolves to a **24-bit
//! ordinal** through an [`IdentityCodebook`] **before the bake**, and the row
//! carries the ordinal. Per #902: *"the saving is not space, it is the
//! disappearance of the read-time join."*
//!
//! That matters concretely here: an OSM node id is ~2³⁴ today and cannot fit a
//! `u24` at all. The codebook is not an optimisation over storing the raw id —
//! it is the only thing that makes the id storable in this tenant.
//!
//! Two encodings come from upstream rather than from this crate, and both are
//! load-bearing:
//!
//! - **G2 `4 × u24` (`WideTriple`), not G3.** #902 chose it deliberately:
//!   *"an exact identity split across three independently-read bytes is no
//!   longer a single invertible value, and invertibility is this tenant's
//!   whole acceptance criterion."* The earlier G3 choice here was reasoned
//!   from headroom; upstream reasoned from invertibility, which is the better
//!   axis.
//! - **Slots store `ordinal + 1`, so raw `0` means absent** — the zero-fallback
//!   ladder. Handled inside `IdentityQuad`; this module never sees it.
//!
//! # What this module still owns
//!
//! The **element kind as the facet's own classid**. `IdentityQuad::into_facet`
//! takes a classid, so `osm_node` / `osm_way` / `osm_relation` stamp different
//! classids onto the same slot and the slot stays readable standalone. That is
//! orthogonal to the quad and survived the rewrite.
//!
//! # An upstream caveat this inherits
//!
//! #902 flags `ISS-IDENTITY-QUAD-WIDE-CARVING-HOME`: under the federation
//! model the cross-bake join key must sit at a **fixed slot index in every
//! bake**, and nothing enforces that yet — *"the design permits the invariant;
//! it does not guarantee it."* [`OSM_ID_SLOT`] is 0 because that is #902's own
//! candidate pin. If the ruling lands elsewhere, this constant moves and
//! nothing else here does.
//!
//! [`IdentityCodebook`]: lance_graph_contract::identity_quad::IdentityCodebook

use lance_graph_contract::canonical_node::NodeRow;
use lance_graph_contract::facet::FacetCascade;
use lance_graph_contract::identity_quad::IdentityQuad;
use ogar_osm::{geo_identity_classid, GEO_IDENTITY_SLOT, RESERVED_SLOTS, ROW_SLOTS};

/// Bytes per facet slot — `classid(4) + payload(12)`.
pub const SLOT_BYTES: usize = 16;

/// Which [`IdentityQuad`] slot carries the OSM element id ordinal.
///
/// Slot 0 per #902's candidate pin for the cross-bake join key (see the module
/// doc's caveat — the pin is proposed upstream, not yet enforced).
pub const OSM_ID_SLOT: usize = 0;

/// Where slot `n` starts **within the value slab**.
///
/// Slots 0 and 1 are the key and the edge block, which live outside the slab,
/// so the slab's own slot 0 is row-slot [`RESERVED_SLOTS`]. Returns `None` for
/// a slot outside the row — a caller cannot address past the stride.
#[must_use]
pub fn slab_offset_of_slot(slot: usize) -> Option<usize> {
    (RESERVED_SLOTS..ROW_SLOTS)
        .contains(&slot)
        .then(|| (slot - RESERVED_SLOTS) * SLOT_BYTES)
}

/// Write the identity facet: the element kind as the facet's **classid**, the
/// codebook ordinal in [`OSM_ID_SLOT`].
///
/// `ordinal` is what an `IdentityCodebook` resolved **pre-bake** — never a raw
/// OSM id, which does not fit a `u24`.
///
/// Returns `false` without writing when `entity_type` is not a Geo concept, or
/// when the ordinal exceeds the quad's capacity. Both are refusals, never
/// truncations: a silently narrowed identity points at the WRONG element and
/// still reads as valid.
pub fn write_identity(row: &mut NodeRow, entity_type: u16, ordinal: u32) -> bool {
    let (Some(classid), Some(off)) = (
        geo_identity_classid(entity_type),
        slab_offset_of_slot(GEO_IDENTITY_SLOT),
    ) else {
        return false;
    };
    let Ok(quad) = IdentityQuad::empty().with_slot(OSM_ID_SLOT, Some(ordinal)) else {
        return false;
    };
    row.value[off..off + SLOT_BYTES].copy_from_slice(&quad.into_facet(classid).to_bytes());
    true
}

/// Read the identity facet back: `(entity_type, ordinal)`.
///
/// Returns `None` when the facet's classid is not a Geo concept — including
/// the all-zero slot of a row that never had an identity written, which reads
/// as *not asserted* rather than as element 0 — or when the slot is empty.
#[must_use]
pub fn read_identity(row: &NodeRow) -> Option<(u16, u32)> {
    let off = slab_offset_of_slot(GEO_IDENTITY_SLOT)?;
    let bytes: [u8; SLOT_BYTES] = row.value[off..off + SLOT_BYTES].try_into().ok()?;
    let facet = FacetCascade::from_bytes(&bytes);
    let concept = (facet.facet_classid >> 16) as u16;
    // Round-trip the classid through the same guard the writer used, so a
    // hand-poked or foreign slot cannot be read as a geo identity.
    geo_identity_classid(concept)?;
    IdentityQuad::from_facet(&facet)
        .slot(OSM_ID_SLOT)
        .map(|ord| (concept, ord))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::{OSM_NODE, OSM_RELATION, OSM_WAY};
    use lance_graph_contract::canonical_node::{EdgeBlock, NodeGuid, TailVariant};
    use lance_graph_contract::identity_quad::MAX_ORDINAL;

    fn blank() -> NodeRow {
        NodeRow {
            key: NodeGuid::mint_for(TailVariant::V3, 0, 0, 0, 0, 0, 0, 0),
            edges: EdgeBlock::default(),
            value: [0u8; 480],
        }
    }

    #[test]
    fn the_slot_is_addressed_by_index_never_by_a_literal_offset() {
        // Slot 2 is the FIRST value slot, so it sits at slab offset 0.
        assert_eq!(slab_offset_of_slot(GEO_IDENTITY_SLOT), Some(0));
        assert_eq!(slab_offset_of_slot(3), Some(16));
        // Key and edge slots are not slab-addressable.
        assert_eq!(slab_offset_of_slot(0), None);
        assert_eq!(slab_offset_of_slot(1), None);
        // Nor is anything past the stride.
        assert_eq!(slab_offset_of_slot(ROW_SLOTS), None);
        // The last real slot IS addressable, so the guard is a boundary and
        // not a function that always refuses.
        assert_eq!(slab_offset_of_slot(ROW_SLOTS - 1), Some((30 - 1) * 16));
    }

    #[test]
    fn an_ordinal_round_trips_across_the_whole_u24_range() {
        for ord in [0u32, 1, 2, 255, 256, 65_535, 65_536, MAX_ORDINAL] {
            let mut row = blank();
            assert!(write_identity(&mut row, OSM_NODE, ord), "ordinal {ord}");
            assert_eq!(
                read_identity(&row),
                Some((OSM_NODE, ord)),
                "ordinal {ord} corrupted"
            );
        }
    }

    #[test]
    fn an_ordinal_past_capacity_is_refused_rather_than_truncated() {
        // The failure this guards: a narrowed identity points at the WRONG
        // element and still reads as valid. Refusal is the only safe outcome.
        let mut row = blank();
        assert!(!write_identity(&mut row, OSM_NODE, MAX_ORDINAL + 1));
        assert_eq!(row.value, [0u8; 480], "a refused write must not land");
        // Boundary asserted from both sides.
        assert!(write_identity(&mut row, OSM_NODE, MAX_ORDINAL));
    }

    #[test]
    fn the_kind_is_recoverable_from_the_slot_alone() {
        // The reason EntityType can go: three kinds, same ordinal, three
        // readings.
        for kind in [OSM_NODE, OSM_WAY, OSM_RELATION] {
            let mut row = blank();
            assert!(write_identity(&mut row, kind, 42));
            assert_eq!(read_identity(&row), Some((kind, 42)));
        }
        // And they really differ — not three names for one stored value.
        let (mut a, mut b) = (blank(), blank());
        write_identity(&mut a, OSM_NODE, 42);
        write_identity(&mut b, OSM_WAY, 42);
        assert_ne!(a.value, b.value);
    }

    #[test]
    fn a_non_geo_kind_is_refused_and_leaves_the_row_untouched() {
        let mut row = blank();
        assert!(
            !write_identity(&mut row, 0x0102, 7),
            "project concept accepted"
        );
        assert_eq!(
            row.value, [0u8; 480],
            "a refused write must not partially land"
        );
        // Can-fire half: the same row accepts a geo kind, so the guard
        // discriminates rather than refusing everything.
        assert!(write_identity(&mut row, OSM_NODE, 7));
    }

    #[test]
    fn an_unwritten_slot_reads_as_not_asserted_never_as_ordinal_zero() {
        // Zero-fallback: an all-zero slot is dormant, not "osm_node ordinal 0".
        // This is exactly why IdentityQuad stores `ordinal + 1` — and ordinal 0
        // IS storable, which is what makes the distinction non-vacuous.
        assert_eq!(read_identity(&blank()), None);
        let mut row = blank();
        assert!(write_identity(&mut row, OSM_NODE, 0));
        assert_eq!(read_identity(&row), Some((OSM_NODE, 0)));
    }

    #[test]
    fn writing_the_identity_touches_only_its_own_slot() {
        // Field isolation (I-LEGACY-API-FEATURE-GATED): the facet must not
        // bleed into the neighbouring slot.
        let mut row = blank();
        assert!(write_identity(&mut row, OSM_WAY, MAX_ORDINAL));
        let off = slab_offset_of_slot(GEO_IDENTITY_SLOT).unwrap();
        let rest: usize = row.value[off + SLOT_BYTES..]
            .iter()
            .map(|b| usize::from(*b))
            .sum();
        assert_eq!(rest, 0, "no byte past the identity slot may be written");
    }

    #[test]
    fn the_other_three_quad_slots_stay_empty_for_a_geo_row() {
        // A geo bake fills ONE of the four identifier spaces. The rest must
        // read as absent, not as ordinal 0 — otherwise a later bake joining a
        // second space would find its slot already claimed.
        let mut row = blank();
        assert!(write_identity(&mut row, OSM_NODE, 12_345));
        let off = slab_offset_of_slot(GEO_IDENTITY_SLOT).unwrap();
        let bytes: [u8; SLOT_BYTES] = row.value[off..off + SLOT_BYTES].try_into().unwrap();
        let quad = IdentityQuad::from_facet(&FacetCascade::from_bytes(&bytes));
        assert_eq!(quad.filled(), 1, "a geo row fills exactly one slot");
        for slot in 0..4 {
            if slot != OSM_ID_SLOT {
                assert_eq!(quad.slot(slot), None, "slot {slot} must stay absent");
            }
        }
    }
}
