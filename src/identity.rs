//! The identity facet — the row's foreign key, in its own 16-byte slot.
//!
//! # The value slab has two readings, and geo picks slots
//!
//! The 480-byte value slab can be read as [`ValueTenant`] lanes (`Meta` at
//! offset 0, `Qualia` at 8, …) **or** as 30 uniform `classid(4) + 12` facet
//! slots. Those two readings **overlap byte-for-byte** — slot 2 is exactly
//! `Meta ++ Qualia` — so a class picks one, and the pick is the ClassView's
//! (`classid → ReadMode`, never a stride change).
//!
//! A geo row picks **slots**. That is what lets `EntityType` go: the element's
//! kind is the identity slot's own classid, so writing it a second time into a
//! tenant lane would be the same fact stored twice under two readings, which
//! is a drift generator rather than redundancy.
//!
//! # Why a foreign key has no rail
//!
//! The 12-byte payload here is `LegacyOutlier::WideQuad` (G3, `3 × u32`) — a
//! *wide* carving with no per-byte axis, which the grace-period amendment
//! calls sanctioned but strongly discouraged.
//!
//! It is the honest shape all the same. The canon's own position is that
//! *"native/foreign discrimination lives where semantics live: in `classid`
//! (foreign keys get a foreign family)"*. An `osm_id` is a **foreign** key —
//! a monotonic counter owned by an external system — and a foreign id has no
//! native axis to decompose onto. The usual remedy (palette256) is closed to
//! it too: quantisation is lossy, and exact round-trip to upstream OSM is the
//! entire reason to carry the field.
//!
//! `4 × u24` (G2) was considered and declined. It would cap the id at 47
//! signed bits — ample against a measured ~2^34 node id — and free two spare
//! fields for `version`/`changeset`. But both carvings are outliers, so the
//! tie-break is simplicity, not headroom: `2 × u32` stores an `i64` exactly,
//! with no cap and therefore no truncation guard to get wrong. The spare
//! fields would buy room for data the PBF's optional `Info` block routinely
//! omits.
//!
//! [`ValueTenant`]: lance_graph_contract::canonical_node::ValueTenant

use lance_graph_contract::canonical_node::NodeRow;
use lance_graph_contract::legacy_outliers::{LegacyOutlier, PAYLOAD_LEN};
use ogar_osm::{geo_identity_classid, GEO_IDENTITY_SLOT, RESERVED_SLOTS, ROW_SLOTS};

/// Bytes per facet slot — `classid(4) + payload(12)`.
pub const SLOT_BYTES: usize = 16;

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

/// Write the identity facet: the element kind as the slot's **classid**, the
/// OSM id as the G3 payload.
///
/// Returns `false` without writing when `entity_type` is not a Geo concept —
/// a non-geo classid must never be stamped into a geo identity slot.
pub fn write_identity(row: &mut NodeRow, entity_type: u16, osm_id: i64) -> bool {
    let (Some(classid), Some(off)) = (
        geo_identity_classid(entity_type),
        slab_offset_of_slot(GEO_IDENTITY_SLOT),
    ) else {
        return false;
    };
    // i64 -> two u32 groups, little-endian low then high. Exact for every
    // i64, including the negative ids JOSM/iD use for not-yet-uploaded drafts.
    let bits = osm_id as u64;
    let payload = LegacyOutlier::write_wide_quad([bits as u32, (bits >> 32) as u32, 0]);
    row.value[off..off + 4].copy_from_slice(&classid.to_le_bytes());
    row.value[off + 4..off + SLOT_BYTES].copy_from_slice(&payload);
    true
}

/// Read the identity facet back: `(entity_type, osm_id)`.
///
/// Returns `None` when the slot's classid is not a Geo concept — including the
/// all-zero slot of a row that never had an identity written, which reads as
/// *not asserted* rather than as element 0 (the zero-fallback ladder).
#[must_use]
pub fn read_identity(row: &NodeRow) -> Option<(u16, i64)> {
    let off = slab_offset_of_slot(GEO_IDENTITY_SLOT)?;
    let classid = u32::from_le_bytes(row.value[off..off + 4].try_into().ok()?);
    let concept = (classid >> 16) as u16;
    // Round-trip the classid through the same guard the writer used, so a
    // hand-poked or foreign slot cannot be read as a geo identity.
    geo_identity_classid(concept)?;
    let payload: [u8; PAYLOAD_LEN] = row.value[off + 4..off + SLOT_BYTES].try_into().ok()?;
    let [lo, hi, _reserved] = LegacyOutlier::read_wide_quad(&payload);
    Some((concept, ((u64::from(hi) << 32) | u64::from(lo)) as i64))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::{OSM_NODE, OSM_RELATION, OSM_WAY};
    use lance_graph_contract::canonical_node::{EdgeBlock, NodeGuid, TailVariant};

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
    fn an_id_round_trips_exactly_including_negative_draft_ids() {
        // Every case a truncating or unsigned reading would corrupt: the real
        // ~2^34 node id, i64 extremes, and the negative ids JOSM/iD use.
        for id in [
            0i64,
            1,
            -1,
            -2,
            12_000_000_000,
            i64::from(i32::MAX) + 1,
            i64::MAX,
            i64::MIN,
        ] {
            let mut row = blank();
            assert!(write_identity(&mut row, OSM_NODE, id));
            assert_eq!(
                read_identity(&row),
                Some((OSM_NODE, id)),
                "id {id} corrupted"
            );
        }
    }

    #[test]
    fn the_kind_is_recoverable_from_the_slot_alone() {
        // The reason EntityType can go: three kinds, same id, three readings.
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
    fn an_unwritten_slot_reads_as_not_asserted_never_as_element_zero() {
        // Zero-fallback: an all-zero slot is dormant, not "osm_node id 0".
        assert_eq!(read_identity(&blank()), None);
    }

    #[test]
    fn writing_the_identity_touches_only_its_own_slot() {
        // Field isolation (I-LEGACY-API-FEATURE-GATED): the facet must not
        // bleed into the neighbouring slot.
        let mut row = blank();
        assert!(write_identity(&mut row, OSM_WAY, i64::MAX));
        let off = slab_offset_of_slot(GEO_IDENTITY_SLOT).unwrap();
        let rest: usize = row.value[off + SLOT_BYTES..]
            .iter()
            .map(|b| usize::from(*b))
            .sum();
        assert_eq!(rest, 0, "no byte past the identity slot may be written");
    }
}
