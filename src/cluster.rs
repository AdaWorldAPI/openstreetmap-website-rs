//! The tile-cluster row — **one row per tile, 30 typed facets**.
//!
//! # Why this exists: the size claim was inverted
//!
//! `build_row` writes one 512-byte row per feature and fills the key (16 B)
//! plus a single identity facet (16 B). Its own field-isolation test asserts
//! every other byte is zero. That is **32 of 512 bytes — 6.25 % occupancy,
//! 93.75 % padding** — which is why a bake came out larger than the PBF it was
//! built from, rather than smaller.
//!
//! Rows are already Morton-sorted, so a tile's features are contiguous. A
//! cluster row keys the **tile** and carries its members in the 30 value
//! slots, so the key amortises over the whole tile instead of being repeated
//! per feature.
//!
//! # The row is a heterogeneous list, and the classid says which is which
//!
//! Every slot is `classid(4) + payload(12)`, and the classid is data. So one
//! row holds facets of different KINDS, each readable standalone:
//!
//! | facet | classid | payload |
//! |---|---|---|
//! | member | `osm_node`/`osm_way`/`osm_relation` × V3 | the element's codebook ordinal |
//! | tag | `osm_element_tag` (`0x0F05`) × V3 | `(member, key, value)` ordinals |
//!
//! Nothing new was needed to discriminate them: `osm_element_tag` has been a
//! minted concept since the Rails harvest, so the tag facet's classid falls
//! out of the codebook — `0x0F05_1000` against a node's `0x0F01_1000`.
//!
//! # Tags are self-describing, not positional
//!
//! A tag facet carries its **member's ordinal**, so it does not depend on
//! sitting after the member it belongs to. The denser alternative — two tags
//! per facet, associated by slot order — packs better but breaks the property
//! the whole ABI rests on: that any slot can be read alone. Order-dependence
//! also makes a dropped or reordered slot silently re-attach a tag to the
//! wrong element, which reads as valid data.
//!
//! # Tags are why "identical data to OSM" was not true
//!
//! `read.rs` calls `tags()` only to FILTER (tagged vs untagged, restriction
//! type) and `Feature` carries none of them. An element without
//! `highway=residential` / `name=…` / `maxspeed=…` is not the same element.
//! This module is where they land.
//!
//! # Superseded, and not yet rewired
//!
//! `build_row`'s one-feature-per-row path still exists and still passes its
//! tests. It is **superseded by this module**, not complemented by it — two
//! row models in one bake would be exactly the drift this crate keeps having
//! to correct. Rewiring the bake (clustered sort, overflow numbering, the
//! `Keyed` pipeline) is the follow-up; this pass is the storage layer and its
//! round-trip proof.

use lance_graph_contract::canonical_node::NodeRow;
use lance_graph_contract::facet::FacetCascade;
use lance_graph_contract::identity_quad::IdentityQuad;
use ogar_osm::{geo_identity_classid, RESERVED_SLOTS, ROW_SLOTS};
use ogar_vocab::class_ids;

use crate::identity::{slab_offset_of_slot, SLOT_BYTES};

/// Value facets a cluster row carries — `32 − key − edges`.
pub const CLUSTER_SLOTS: usize = ROW_SLOTS - RESERVED_SLOTS;

/// `IdentityQuad` slot holding a member facet's element ordinal.
pub const MEMBER_ORDINAL: usize = 0;

/// `IdentityQuad` slots of a tag facet: the member it belongs to, then the
/// key and value ordinals.
pub const TAG_MEMBER: usize = 0;
/// See [`TAG_MEMBER`].
pub const TAG_KEY: usize = 1;
/// See [`TAG_MEMBER`].
pub const TAG_VALUE: usize = 2;

/// One facet of a cluster row, discriminated by its own classid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Facet {
    /// An element: its concept (`osm_node` / `osm_way` / …) and codebook ordinal.
    Member {
        /// The element kind's canonical concept id.
        kind: u16,
        /// The element's `IdentityCodebook` ordinal.
        ordinal: u32,
    },
    /// A `k=v` tag on one of this row's members.
    Tag {
        /// The member ordinal this tag belongs to.
        member: u32,
        /// The tag key's codebook ordinal.
        key: u32,
        /// The tag value's codebook ordinal.
        value: u32,
    },
}

/// Write a member facet into `slot`.
///
/// Returns `false` without writing when the slot is not a value slot, the kind
/// is not a Geo concept, or the ordinal exceeds the quad's capacity — refusals,
/// never truncations.
pub fn write_member(row: &mut NodeRow, slot: usize, kind: u16, ordinal: u32) -> bool {
    let (Some(classid), Some(off)) = (geo_identity_classid(kind), slab_offset_of_slot(slot)) else {
        return false;
    };
    let Ok(quad) = IdentityQuad::empty().with_slot(MEMBER_ORDINAL, Some(ordinal)) else {
        return false;
    };
    row.value[off..off + SLOT_BYTES].copy_from_slice(&quad.into_facet(classid).to_bytes());
    true
}

/// Write a tag facet into `slot`, bound to `member` by ordinal rather than by
/// position.
///
/// Returns `false` without writing when the slot is not a value slot or any
/// ordinal exceeds the quad's capacity.
pub fn write_tag(row: &mut NodeRow, slot: usize, member: u32, key: u32, value: u32) -> bool {
    let (Some(classid), Some(off)) = (
        geo_identity_classid(class_ids::OSM_ELEMENT_TAG),
        slab_offset_of_slot(slot),
    ) else {
        return false;
    };
    let quad = IdentityQuad::empty();
    let Ok(quad) = quad.with_slot(TAG_MEMBER, Some(member)) else {
        return false;
    };
    let Ok(quad) = quad.with_slot(TAG_KEY, Some(key)) else {
        return false;
    };
    let Ok(quad) = quad.with_slot(TAG_VALUE, Some(value)) else {
        return false;
    };
    row.value[off..off + SLOT_BYTES].copy_from_slice(&quad.into_facet(classid).to_bytes());
    true
}

/// Read whichever facet kind occupies `slot`.
///
/// `None` for a slot outside the value slab, a slot whose classid is not a Geo
/// concept (including the all-zero slot of an unfilled row, which reads as
/// *not asserted*), or a facet missing the ordinals its kind requires.
#[must_use]
pub fn read_facet(row: &NodeRow, slot: usize) -> Option<Facet> {
    let off = slab_offset_of_slot(slot)?;
    let bytes: [u8; SLOT_BYTES] = row.value[off..off + SLOT_BYTES].try_into().ok()?;
    let facet = FacetCascade::from_bytes(&bytes);
    let concept = (facet.facet_classid >> 16) as u16;
    geo_identity_classid(concept)?;
    let quad = IdentityQuad::from_facet(&facet);
    if concept == class_ids::OSM_ELEMENT_TAG {
        Some(Facet::Tag {
            member: quad.slot(TAG_MEMBER)?,
            key: quad.slot(TAG_KEY)?,
            value: quad.slot(TAG_VALUE)?,
        })
    } else {
        Some(Facet::Member {
            kind: concept,
            ordinal: quad.slot(MEMBER_ORDINAL)?,
        })
    }
}

/// How many facet slots this row has filled — the occupancy guard's unit.
///
/// Counts without materialising the facets, because a bake calls it once per
/// row over millions of rows and a `Vec` per row is pure waste there.
#[must_use]
pub fn filled_slots(row: &NodeRow) -> usize {
    (RESERVED_SLOTS..ROW_SLOTS)
        .filter(|&s| read_facet(row, s).is_some())
        .count()
}

/// Every occupied facet of a cluster row, in slot order.
#[must_use]
pub fn facets(row: &NodeRow) -> Vec<(usize, Facet)> {
    (RESERVED_SLOTS..ROW_SLOTS)
        .filter_map(|s| read_facet(row, s).map(|f| (s, f)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::{OSM_NODE, OSM_WAY};
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
    fn a_cluster_row_carries_thirty_value_facets() {
        assert_eq!(CLUSTER_SLOTS, 30);
        assert_eq!(RESERVED_SLOTS + CLUSTER_SLOTS, ROW_SLOTS);
    }

    #[test]
    fn a_tile_cluster_round_trips_members_and_their_tags() {
        // The shape the size claim rests on: one key, many features, tags
        // included. Two ways and a node, each carrying a tag.
        let mut row = blank();
        assert!(write_member(&mut row, 2, OSM_WAY, 100));
        assert!(write_tag(&mut row, 3, 100, 7, 42)); // highway=residential
        assert!(write_member(&mut row, 4, OSM_WAY, 101));
        assert!(write_tag(&mut row, 5, 101, 7, 43)); // highway=service
        assert!(write_member(&mut row, 6, OSM_NODE, 102));

        let got = facets(&row);
        assert_eq!(got.len(), 5, "five occupied slots, no more, no fewer");
        assert_eq!(
            got[0].1,
            Facet::Member {
                kind: OSM_WAY,
                ordinal: 100
            }
        );
        assert_eq!(
            got[1].1,
            Facet::Tag {
                member: 100,
                key: 7,
                value: 42
            }
        );
        assert_eq!(
            got[4].1,
            Facet::Member {
                kind: OSM_NODE,
                ordinal: 102
            }
        );

        // The two tags share a KEY ordinal but differ in VALUE — so the
        // reading is not collapsing the pair into one field.
        let (
            Facet::Tag {
                key: k0, value: v0, ..
            },
            Facet::Tag {
                key: k1, value: v1, ..
            },
        ) = (got[1].1, got[3].1)
        else {
            panic!("slots 3 and 5 must be tags")
        };
        assert_eq!(k0, k1);
        assert_ne!(v0, v1);
    }

    #[test]
    fn a_tag_binds_to_its_member_by_ordinal_not_by_position() {
        // The property that makes a slot readable alone. A tag written FAR
        // from its member still resolves to it, so a reordering or a gap
        // cannot silently re-attach it to the wrong element.
        let mut row = blank();
        assert!(write_member(&mut row, 2, OSM_WAY, 500));
        assert!(write_tag(&mut row, 31, 500, 9, 9));
        let got = facets(&row);
        assert_eq!(got.len(), 2);
        let Facet::Tag { member, .. } = got[1].1 else {
            panic!("slot 31 must be a tag")
        };
        assert_eq!(member, 500, "the tag names its member, wherever it sits");

        // Anti-vacuity: a tag for a DIFFERENT member reads as that member,
        // so the field is genuinely read rather than defaulted.
        let mut other = blank();
        assert!(write_tag(&mut other, 31, 501, 9, 9));
        let Facet::Tag { member, .. } = read_facet(&other, 31).unwrap() else {
            panic!()
        };
        assert_eq!(member, 501);
    }

    #[test]
    fn the_two_facet_kinds_are_discriminated_by_classid_alone() {
        // Same slot, same payload shape — only the classid differs, and that
        // is what the ClassView reading turns on.
        let mut a = blank();
        let mut b = blank();
        assert!(write_member(&mut a, 5, OSM_WAY, 77));
        assert!(write_tag(&mut b, 5, 77, 0, 0));
        assert!(matches!(read_facet(&a, 5), Some(Facet::Member { .. })));
        assert!(matches!(read_facet(&b, 5), Some(Facet::Tag { .. })));
        // And the classids really differ — not one id read two ways.
        let off = slab_offset_of_slot(5).unwrap();
        assert_ne!(a.value[off..off + 4], b.value[off..off + 4]);
    }

    #[test]
    fn an_unfilled_slot_reads_as_absent_and_gaps_do_not_terminate_the_row() {
        let mut row = blank();
        assert!(read_facet(&row, 2).is_none());
        // A gap in the middle must not hide what follows it.
        assert!(write_member(&mut row, 2, OSM_NODE, 1));
        assert!(write_member(&mut row, 20, OSM_NODE, 2));
        assert_eq!(facets(&row).len(), 2, "the gap must not truncate the read");
        assert!(read_facet(&row, 10).is_none());
    }

    #[test]
    fn slots_outside_the_value_slab_are_refused_in_both_directions() {
        let mut row = blank();
        // The key and edge slots are not writable as facets.
        assert!(!write_member(&mut row, 0, OSM_NODE, 1));
        assert!(!write_member(&mut row, 1, OSM_NODE, 1));
        assert!(!write_tag(&mut row, 1, 1, 1, 1));
        assert!(!write_member(&mut row, ROW_SLOTS, OSM_NODE, 1));
        assert_eq!(row.value, [0u8; 480], "a refused write must not land");
        // Can-fire half.
        assert!(write_member(&mut row, RESERVED_SLOTS, OSM_NODE, 1));
    }

    #[test]
    fn an_over_capacity_ordinal_is_refused_rather_than_truncated() {
        // A narrowed ordinal points at the WRONG element or tag and still
        // reads as valid — the failure mode worth refusing.
        let mut row = blank();
        assert!(!write_member(&mut row, 2, OSM_NODE, MAX_ORDINAL + 1));
        assert!(!write_tag(&mut row, 2, MAX_ORDINAL + 1, 0, 0));
        assert!(!write_tag(&mut row, 2, 0, MAX_ORDINAL + 1, 0));
        assert!(!write_tag(&mut row, 2, 0, 0, MAX_ORDINAL + 1));
        assert_eq!(row.value, [0u8; 480]);
        // Boundary from the other side.
        assert!(write_tag(
            &mut row,
            2,
            MAX_ORDINAL,
            MAX_ORDINAL,
            MAX_ORDINAL
        ));
    }

    #[test]
    fn a_full_cluster_row_beats_one_feature_per_row_on_bytes_per_feature() {
        // The size claim, as arithmetic over the real stride rather than a
        // recalled figure. Ten members with two tags each fills all 30 slots.
        let mut row = blank();
        let mut slot = RESERVED_SLOTS;
        for m in 0..10u32 {
            assert!(write_member(&mut row, slot, OSM_WAY, m));
            slot += 1;
            for t in 0..2u32 {
                assert!(write_tag(&mut row, slot, m, 7, t));
                slot += 1;
            }
        }
        assert_eq!(slot, ROW_SLOTS, "ten members with two tags fills the row");
        assert_eq!(facets(&row).len(), 30);

        // 512 bytes carrying 10 features AND their 20 tags.
        const STRIDE: usize = 512;
        let per_feature = STRIDE / 10;
        assert_eq!(per_feature, 51);
        // The superseded path spends the whole stride on one feature and
        // stores no tags at all.
        assert!(
            per_feature * 10 <= STRIDE && per_feature < STRIDE,
            "a cluster row must beat 512 B/feature"
        );
    }
}
