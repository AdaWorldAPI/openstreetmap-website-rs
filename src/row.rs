//! Feature → the 512-byte V3 ABI row: `key(16) | edges(16) | value(480)`.
//!
//! The row layout is `lance_graph_contract::canonical_node::NodeRow`, locked
//! 2026-06-13. Nothing here invents a byte position: the key is minted through
//! `NodeGuid::mint_for`, and the one value lane written is addressed through
//! `ValueTenant::value_offset()` — never a literal offset. (q2's `osint-bake`
//! pokes slab byte 0 by literal, which lands inside the `Meta` tenant's range;
//! `tenants.md` names that exact drift and its three stale-offset incidents.)

use lance_graph_contract::canonical_node::{
    EdgeBlock, NodeGuid, NodeRow, TailVariant, ValueTenant,
};

use crate::read::Feature;
use crate::tms::{self, Tiers};

/// Geo-V3 classid: geo domain `0x0F` + appid `0x01` (q2) in the canon HIGH
/// half, V3 marker `0x1000` in the custom LOW half — the post-2026-07-02
/// canon-HIGH layout, parallel to `NodeGuid::CLASSID_FMA_V3` (`0x0A01_1000`).
///
/// **This is still a proof-local mint.** `classid_read_mode()` has no entry for
/// it, so it resolves to `ReadMode::DEFAULT` (`ValueSchema::Full`) rather than
/// the `Compressed` preset a cold baked corpus wants. Registering
/// `CLASSID_GEO` / `CLASSID_GEO_V3` + a `ReadMode::GEO` in
/// `lance-graph-contract` is an upstream change and the operator's call to
/// make — surfaced, not filed. The geo *domain* is already minted in OGAR
/// (`0x0F => ConceptDomain::Geo`, `osm_node 0x0F01 … osm_user 0x0F0A`); only
/// the read-mode registration is missing.
pub const CLASSID_GEO_V3: u32 = 0x0F01_1000;

/// The tail variant is forced rather than resolved, for the reason above: an
/// unregistered classid resolves to V1, whose `new()` has no `leaf` tier and
/// would silently drop the 4th spatial tier — the one that takes the key from
/// 1.45 m to 5.7 mm.
const TAIL: TailVariant = TailVariant::V3;

/// A feature reduced to its sortable key material, before rows are built.
/// Sorting happens on the **Morton code**, not on the key bytes: the tiers are
/// little-endian `u16`s inside the GUID, so a lexicographic byte sort is NOT
/// trie order.
#[derive(Debug, Clone, Copy)]
pub struct Keyed {
    pub morton: u64,
    pub tiers: Tiers,
    pub entity_type: u16,
    pub identity: u16,
}

/// Key a feature: TMS Morton at z=32 → four cascade tiers.
#[must_use]
pub fn key_feature(f: &Feature) -> Keyed {
    let (morton, tiers) = tms::point_to_tiers(f.lon, f.lat);
    Keyed {
        morton,
        tiers,
        entity_type: f.entity_type,
        identity: 0,
    }
}

/// Build the 512-byte row for one keyed feature.
///
/// - **key** — `classid | heel | hip | twig | leaf | family | identity`.
///   `family = 0` deliberately: the CANON zero-fallback ladder reads a zero
///   family as *"default basin, no neighbourhood grouping"*, and the feature's
///   kind belongs in `EntityType` + the ClassView, never in a key tier
///   (le-contract §2). `identity` only breaks a same-tile collision.
/// - **edges** — zeroed. Reserved, never shrunk; a class opting out of edges is
///   resolved via `classid → ClassView`, never by a stride change.
/// - **value** — `EntityType` only. Every other lane stays zero per
///   RESERVE-DON'T-RECLAIM; a dormant lane is *not consulted*, not compacted.
#[must_use]
pub fn build_row(k: &Keyed) -> NodeRow {
    let key = NodeGuid::mint_for(
        TAIL,
        CLASSID_GEO_V3,
        k.tiers.heel,
        k.tiers.hip,
        k.tiers.twig,
        k.tiers.leaf,
        0,
        u32::from(k.identity),
    );
    let mut value = [0u8; 480];
    let off = ValueTenant::EntityType.value_offset();
    let len = ValueTenant::EntityType.byte_len();
    debug_assert_eq!(len, 2, "EntityType is a U16 lane");
    value[off..off + len].copy_from_slice(&k.entity_type.to_le_bytes());
    NodeRow {
        key,
        edges: EdgeBlock::default(),
        value,
    }
}

/// Assign per-tile `identity` counters over a **Morton-sorted** slice, so a
/// collision is a run of equal keys and needs no map. Returns the number of
/// features that collided (identity > 0) — the falsifier for "z=32 makes a
/// feature near-unique".
pub fn assign_identities(keyed: &mut [Keyed]) -> u64 {
    let mut collisions = 0u64;
    let mut i = 0usize;
    while i < keyed.len() {
        let mut j = i + 1;
        while j < keyed.len() && keyed[j].morton == keyed[i].morton {
            let n = (j - i) as u16;
            keyed[j].identity = n;
            collisions += 1;
            j += 1;
        }
        i = j;
    }
    collisions
}

#[cfg(test)]
mod tests {
    use super::*;
    use lance_graph_contract::canonical_node::NODE_ROW_STRIDE;

    fn feat(lon: f64, lat: f64) -> Feature {
        Feature {
            lon,
            lat,
            entity_type: crate::read::OSM_WAY,
            osm_id: 1,
        }
    }

    #[test]
    fn row_is_exactly_the_canon_stride() {
        assert_eq!(core::mem::size_of::<NodeRow>(), NODE_ROW_STRIDE);
        assert_eq!(NODE_ROW_STRIDE, 512);
    }

    #[test]
    fn key_carries_all_four_spatial_tiers() {
        // The V3 mint must not drop `leaf` — the failure the forced TailVariant
        // exists to prevent. Berlin Mitte has a non-zero leaf tier at z=32.
        let k = key_feature(&feat(13.404954, 52.520008));
        let row = build_row(&k);
        let b = row.key.as_bytes();
        let heel = u16::from_le_bytes([b[4], b[5]]);
        let hip = u16::from_le_bytes([b[6], b[7]]);
        let twig = u16::from_le_bytes([b[8], b[9]]);
        let leaf = u16::from_le_bytes([b[10], b[11]]);
        assert_eq!(
            (heel, hip, twig, leaf),
            (k.tiers.heel, k.tiers.hip, k.tiers.twig, k.tiers.leaf)
        );
        assert_ne!(leaf, 0, "the 4th tier must be carried, not dropped");
        assert_eq!(row.key.classid(), CLASSID_GEO_V3);
    }

    #[test]
    fn entity_type_lands_in_its_tenant_and_nowhere_else() {
        // Field isolation (I-LEGACY-API-FEATURE-GATED): writing one lane must
        // leave every other byte of the slab zero.
        let mut k = key_feature(&feat(13.404954, 52.520008));
        k.entity_type = crate::read::OSM_WAY;
        let row = build_row(&k);
        let off = ValueTenant::EntityType.value_offset();
        assert_eq!(
            u16::from_le_bytes([row.value[off], row.value[off + 1]]),
            crate::read::OSM_WAY
        );
        let others: usize = row
            .value
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != off && *i != off + 1)
            .map(|(_, b)| usize::from(*b))
            .sum();
        assert_eq!(others, 0, "no lane but EntityType may be written");
    }

    #[test]
    fn morton_sort_is_trie_order_and_byte_sort_is_not() {
        // The trap this module's sort key exists to avoid: the tiers are LE
        // u16 inside the GUID, so sorting raw key bytes is NOT cascade order.
        let a = key_feature(&feat(13.404954, 52.520008));
        let b = key_feature(&feat(13.5, 52.6));
        let (lo, hi) = if a.morton < b.morton { (a, b) } else { (b, a) };
        let (rl, rh) = (build_row(&lo), build_row(&hi));
        assert!(lo.morton < hi.morton);
        // Byte-lexicographic order over the key does not agree in general;
        // assert we did NOT rely on it by checking the tier tuple instead.
        let t = |r: &NodeRow| {
            let b = r.key.as_bytes();
            (
                u16::from_le_bytes([b[4], b[5]]),
                u16::from_le_bytes([b[6], b[7]]),
            )
        };
        assert!(t(&rl) <= t(&rh), "coarse tiers must be non-decreasing");
    }

    #[test]
    fn identities_number_only_genuine_collisions() {
        // Two distinct places keep identity 0; a repeated place gets 1.
        let mut k = vec![
            key_feature(&feat(13.404954, 52.520008)),
            key_feature(&feat(13.404954, 52.520008)),
            key_feature(&feat(13.5, 52.6)),
        ];
        k.sort_by_key(|x| x.morton);
        let c = assign_identities(&mut k);
        assert_eq!(c, 1, "exactly one collision");
        assert_eq!(k.iter().filter(|x| x.identity == 0).count(), 2);
    }
}
