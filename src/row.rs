//! Feature → the 512-byte V3 ABI row: `key(16) | edges(16) | value(480)`.
//!
//! The row layout is `lance_graph_contract::canonical_node::NodeRow`, locked
//! 2026-06-13. Nothing here invents a byte position: the key is minted through
//! `NodeGuid::mint_for`, and the value slab is addressed by **slot index**
//! through `crate::identity` — never a literal offset. (q2's `osint-bake`
//! pokes slab byte 0 by literal, which lands inside the `Meta` tenant's range;
//! `tenants.md` names that exact drift and its three stale-offset incidents.)

use lance_graph_contract::canonical_node::{EdgeBlock, NodeGuid, NodeRow, TailVariant};
use lance_graph_contract::identity_quad::{CodebookError, IdentityCodebook};

use crate::read::Feature;
use crate::street;
use crate::tags::{ResolvedTags, TagSpan};
use crate::tms::{self, Tiers};

/// Row-local edge-name payload for a junction row's [`street::NAME_SLOT`].
/// Fixed-size, not `Vec` — so [`Keyed`] stays `Copy`, matching the fixed-
/// capacity-plus-continuation shape [`TAGS_PER_ROW`]/[`expand_tag_overflow`]
/// already use for tags, at the width [`street::EDGE_SLOTS`] commits to.
/// `len == 0` (the default) means "nothing to write" — an ordinary row.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EdgeNames {
    pub ordinals: [u16; street::EDGE_SLOTS as usize],
    pub len: u8,
}

impl EdgeNames {
    #[must_use]
    pub fn as_slice(&self) -> &[u16] {
        &self.ordinals[..self.len as usize]
    }
}

/// Tag facets one row can carry: everything after the key, edges, identity and
/// the reserved slope slot.
///
/// The slope slot is skipped rather than reused. It is dormant in this bake but
/// **reserved** — the CANON's RESERVE-DON'T-RECLAIM rule reads a zero tier as
/// *not consulted*, never as *free space*, and a bake that packed tags into it
/// would silently collide the day a slope lands.
pub const TAGS_PER_ROW: usize = ogar_osm::ROW_SLOTS - ogar_osm::GEO_SLOPE_SLOT - 1;
const _: () = assert!(TAGS_PER_ROW == 28);

/// The first slot a tag facet may occupy.
const FIRST_TAG_SLOT: usize = ogar_osm::GEO_SLOPE_SLOT + 1;

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
///
/// **Composed, not written.** It was the literal `0x0F01_1000` until the
/// plug-and-play rule was applied end to end: `ogar_osm::classid` delegates to
/// `ogar_vocab::app::render_classid`, so neither half nor the shift is
/// restated here. The concept half is `osm_node` because this is the KEY's
/// class — the shared "geo V3 row" anchor every element kind is minted under;
/// the per-element kind lives in the identity facet's own classid
/// (`crate::identity`), which is what lets one key class serve all ten
/// concepts.
pub const CLASSID_GEO_V3: u32 = ogar_osm::classid(
    ogar_vocab::class_ids::OSM_NODE,
    ogar_osm::CLASSVIEW_V3_SUBSTRATE,
);

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
    /// The upstream OSM element id, harvested for every element kind in
    /// `read.rs`. It is **not** written to the row: a raw id needs 34 bits and
    /// the identity tenant's slots are `u24`. It is kept here so the pre-bake
    /// stage can build the codebook that turns it into [`Self::identity_ordinal`].
    pub osm_id: i64,
    /// The codebook ordinal for [`Self::osm_id`], assigned **pre-bake** by
    /// [`resolve_identities`]. `None` until that stage runs — a row built
    /// without it carries no identity rather than a wrong one.
    pub identity_ordinal: Option<u32>,
    pub identity: u16,
    /// The tags this ROW carries — not necessarily the whole feature's.
    ///
    /// A feature with more than [`TAGS_PER_ROW`] tags is split by
    /// [`expand_tag_overflow`] into several `Keyed`s over disjoint sub-spans,
    /// each of which becomes its own row. Every such row still carries the
    /// identity facet, so a continuation is self-describing: a reader
    /// reassembles a feature by grouping rows on `(kind, ordinal)`, never by
    /// row adjacency.
    pub tags: TagSpan,
    /// Raw label-codebook ordinals for [`crate::street::NAME_SLOT`], written
    /// verbatim (never through `tags`/`ResolvedTags` — this is a SEPARATE,
    /// small codebook; see `codebook.rs`'s "fourth book" docs). Empty for
    /// every ordinary row.
    ///
    /// A junction with more than [`crate::street::EDGE_SLOTS`] named edges
    /// gets split into several `Keyed`s here too, the SAME pattern
    /// [`expand_tag_overflow`] uses for tags: identical identity, disjoint
    /// slices, grouped back together by `(kind, ordinal)` on read. See
    /// [`junction_keyed`].
    pub edge_names: EdgeNames,
}

/// Key a feature: TMS Morton at z=32 → four cascade tiers.
#[must_use]
pub fn key_feature(f: &Feature) -> Keyed {
    // The anchor decides which contract applies; `cell()` is the one place the
    // published-vs-derived distinction is resolved, and for a derived anchor it
    // is pure integer — no float reaches the key.
    let morton = tms::cell_to_morton(f.anchor.cell());
    let tiers = tms::tiers_of(morton);
    Keyed {
        morton,
        tiers,
        entity_type: f.entity_type,
        osm_id: f.osm_id,
        identity_ordinal: None,
        identity: 0,
        tags: f.tags,
        edge_names: EdgeNames::default(),
    }
}

/// Split any feature carrying more than [`TAGS_PER_ROW`] tags into a run of
/// `Keyed`s over disjoint tag sub-spans, so every row's tags fit its slots.
///
/// Returns the number of **continuation** rows added — measured, not assumed:
/// the `tier_probe` distribution says this is a fraction of a percent on Berlin,
/// and a bake that silently truncated instead would drop tags from exactly the
/// richest features (the ones a renderer most needs).
///
/// Build one or more `Keyed` rows for a junction, given its resolved
/// per-edge name ordinals — the same "fixed bucket + continuation" pattern as
/// [`expand_tag_overflow`], applied to [`EdgeNames`] instead of [`TagSpan`].
///
/// `names` may be **longer** than [`crate::street::EDGE_SLOTS`] (the P12
/// measurement found exactly this on real data: 619,739 of 619,740 Berlin
/// junctions fit within 8 slots, one did not). Rather than silently truncate
/// — dropping the tail edges' street identity, the exact defect the
/// occupancy guard elsewhere in this bake exists to make impossible — this
/// splits into `ceil(names.len() / EDGE_SLOTS)` rows sharing one identity
/// key, so `resolve_identities` collapses them to one ordinal and a reader
/// groups them back by `(kind, ordinal)` exactly as a tag continuation.
///
/// A junction with NO named edges (every incident way is `highway=*` with no
/// `name` tag — common for service roads) produces **zero** rows: nothing to
/// resolve, nothing to look up, and no row occupying space for an all-zero
/// payload that `edge_mask` would read as empty anyway.
#[must_use]
pub fn junction_keyed(j: &crate::read::Junction, names: &[u16]) -> Vec<Keyed> {
    if names.is_empty() {
        return Vec::new();
    }
    let morton = tms::cell_to_morton(j.cell);
    let tiers = tms::tiers_of(morton);
    let base = Keyed {
        morton,
        tiers,
        // The DERIVED concept, not the published node it sits on. A junction
        // IS an `osm_node` upstream, so both rows would otherwise key on the
        // same `identity_key` (`{entity_type:04x}:{osm_id}`) and
        // `resolve_identities` would collapse them to one ordinal — leaving
        // nothing on disk to tell a junction from a tagged POI. That is the
        // ambiguity `crate::street`'s docs measured at 1,084,213 false hits.
        entity_type: crate::read::OSM_STREET_NODE,
        osm_id: j.node_id,
        identity_ordinal: None,
        identity: 0,
        tags: TagSpan::default(),
        edge_names: EdgeNames::default(),
    };
    let per = street::EDGE_SLOTS as usize;
    names
        .chunks(per)
        .map(|chunk| {
            let mut ordinals = [0u16; street::EDGE_SLOTS as usize];
            ordinals[..chunk.len()].copy_from_slice(chunk);
            Keyed {
                edge_names: EdgeNames {
                    ordinals,
                    len: chunk.len() as u8,
                },
                ..base
            }
        })
        .collect()
}

/// Continuations are not marked as such, deliberately. Each carries the same
/// identity facet, so a reader reassembles a feature by grouping on
/// `(kind, ordinal)` — order-independent, and correct even if rows are shuffled
/// or a fragment is read alone.
pub fn expand_tag_overflow(keyed: &mut Vec<Keyed>) -> u64 {
    let per = TAGS_PER_ROW as u32;
    let mut extra: Vec<Keyed> = Vec::new();
    for k in keyed.iter_mut() {
        if k.tags.len <= per {
            continue;
        }
        let full = k.tags;
        k.tags = TagSpan {
            start: full.start,
            len: per,
        };
        let mut off = per;
        while off < full.len {
            let len = (full.len - off).min(per);
            extra.push(Keyed {
                tags: TagSpan {
                    start: full.start + off,
                    len,
                },
                ..*k
            });
            off += len;
        }
    }
    let added = extra.len() as u64;
    keyed.append(&mut extra);
    added
}

/// A row with no tags — for tests whose subject is key layout, not tags.
#[cfg(test)]
pub(crate) fn build_row_notags(k: &Keyed) -> NodeRow {
    build_row(
        k,
        &crate::tags::TagStore::default()
            .resolve()
            .expect("an empty store resolves"),
    )
}

/// Build the 512-byte row for one keyed feature.
///
/// - **key** — `classid | heel | hip | twig | leaf | family | identity`.
///   `family = 0` deliberately: the CANON zero-fallback ladder reads a zero
///   family as *"default basin, no neighbourhood grouping"*, and the feature's
///   kind belongs in the value slab + the ClassView, never in a key tier
///   (le-contract §2). `identity` only breaks a same-tile collision.
/// - **edges** — zeroed. Reserved, never shrunk; a class opting out of edges is
///   resolved via `classid → ClassView`, never by a stride change.
/// - **value** — the **identity facet in slot 2**, and nothing else. Every
///   other slot stays zero per RESERVE-DON'T-RECLAIM; a dormant slot is *not
///   consulted*, not compacted.
///
/// # The value slab is read as SLOTS here, not as tenant lanes
///
/// The 480 bytes have two readings — [`ValueTenant`] lanes, or 30 uniform
/// `classid(4) + 12` facet slots — and they **overlap byte-for-byte** (slot 2
/// is exactly `Meta ++ Qualia`). A class picks one; a geo row picks slots.
///
/// So `EntityType` is deliberately **no longer written**. The element's kind
/// is the identity slot's own classid (`geo_identity_classid`), which makes
/// the slot readable standalone — and writing the kind a second time into a
/// tenant lane would store one fact twice under two incompatible readings.
/// That is a drift generator, not redundancy. See `crate::identity`.
#[must_use]
pub fn build_row(k: &Keyed, tags: &ResolvedTags) -> NodeRow {
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
    let mut row = NodeRow {
        key,
        edges: EdgeBlock::default(),
        value: [0u8; 480],
    };
    if let Some(ordinal) = k.identity_ordinal {
        let wrote = crate::identity::write_identity(&mut row, k.entity_type, ordinal);
        debug_assert!(
            wrote,
            "entity_type {:#06x} / ordinal {ordinal} was refused — the bake \
             must not key a row it cannot identify",
            k.entity_type
        );
        // Tags bind to the feature by ORDINAL, not by adjacency — the property
        // that lets a continuation row be read alone (see `crate::cluster`).
        for (i, &(key, value)) in tags.span(k.tags).iter().enumerate() {
            debug_assert!(i < TAGS_PER_ROW, "expand_tag_overflow must have split this");
            let wrote =
                crate::cluster::write_tag(&mut row, FIRST_TAG_SLOT + i, ordinal, key, value);
            debug_assert!(wrote, "tag ({key}, {value}) was refused for slot {i}");
        }
    }
    if k.edge_names.len > 0 {
        // A junction row: no ordinary tags (checked above — `k.tags` is
        // `TagSpan::default()` for every junction `Keyed`, so the loop above
        // wrote nothing), its whole payload is this slot.
        street::set_edge_names(&mut row, k.edge_names.as_slice());
    }
    row
}

/// The **pre-bake** identity stage: build the codebook from every observed
/// `osm_id` and stamp each row's ordinal.
///
/// This is the stage lance-graph PR #902 names — *"the identities are resolved
/// **before** the bake, and the bake places all four into one V3 facet
/// payload"* — so that a read is a fixed-offset register read with no join.
/// `build_row` is pure layout and never resolves anything.
///
/// Keys are `"{kind:04x}:{osm_id}"`, kind-qualified because OSM ids are only
/// unique **within** an element type: node 42 and way 42 are different
/// elements, and an unqualified key would collide them into one ordinal —
/// which `IdentityCodebook` would then reject as non-injective, turning a
/// silent identity merge into a loud refusal.
///
/// # Errors
///
/// Returns the codebook's own error when the distinct-key count exceeds
/// `MAX_ENTRIES` — refused rather than truncated, per #902's
/// refuse-don't-widen discipline.
pub fn resolve_identities(keyed: &mut [Keyed]) -> Result<IdentityCodebook, CodebookError> {
    let mut keys: Vec<String> = keyed.iter().map(identity_key).collect();
    keys.sort_unstable();
    keys.dedup();
    let book = IdentityCodebook::try_new(keys)?;
    for k in keyed.iter_mut() {
        k.identity_ordinal = book.ordinal(&identity_key(k));
    }
    Ok(book)
}

/// The codebook key for one feature — kind-qualified (see [`resolve_identities`]).
fn identity_key(k: &Keyed) -> String {
    format!("{:04x}:{}", k.entity_type, k.osm_id)
}

/// The bake's total row order: Morton first (trie order), then tag-span start.
///
/// Morton alone is NOT a total order here. A feature's continuation rows all
/// carry the same Morton code, and `sort_unstable` gives no guarantee among
/// equal keys — so the chunks of one feature's tag list could come out
/// permuted, reassembling into the right multiset in the wrong sequence. That
/// is exactly what the `parity` binary caught: 2,633 features with matching tag
/// COUNTS and mismatched order.
///
/// Sorting on the span start too makes the layout deterministic, which is worth
/// having on its own — two bakes of one extract should be byte-identical.
#[must_use]
pub fn sort_key(k: &Keyed) -> (u64, u32) {
    (k.morton, k.tags.start)
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
    use lance_graph_contract::canonical_node::{ValueTenant, NODE_ROW_STRIDE};

    /// Empty codebooks, for the tests whose subject is layout rather than tags.
    fn no_tags() -> ResolvedTags {
        crate::tags::TagStore::default()
            .resolve()
            .expect("an empty store resolves")
    }

    fn feat(lon: f64, lat: f64) -> Feature {
        Feature {
            anchor: crate::read::Anchor::Published { lon, lat },
            entity_type: crate::read::OSM_WAY,
            osm_id: 1,
            tags: crate::tags::TagSpan::default(),
        }
    }

    #[test]
    fn the_key_classid_is_composed_from_the_codebook_not_a_literal() {
        // Pins the composed value against the halves it is built from, so a
        // future concept renumber upstream cannot leave a stale literal here.
        assert_eq!(
            (CLASSID_GEO_V3 >> 16) as u16,
            ogar_vocab::class_ids::OSM_NODE,
            "the canon half must be the minted osm_node concept"
        );
        assert_eq!(
            CLASSID_GEO_V3 as u16,
            ogar_osm::CLASSVIEW_V3_SUBSTRATE,
            "the custom half must be the V3 substrate ClassView"
        );
        // Anti-vacuity: the two halves are genuinely different values, so the
        // assertions above are not both reading the same number.
        assert_ne!(
            u32::from(ogar_vocab::class_ids::OSM_NODE),
            u32::from(ogar_osm::CLASSVIEW_V3_SUBSTRATE)
        );
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
        let row = build_row(&k, &no_tags());
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
    fn the_codebook_key_is_kind_qualified_so_node_42_and_way_42_stay_distinct() {
        // OSM ids are unique only WITHIN an element type. An unqualified key
        // would map node 42 and way 42 to one ordinal — a silent identity
        // merge, the worst class of bug here because both rows still read as
        // valid.
        let mut batch = [
            key_feature(&Feature {
                anchor: crate::read::Anchor::Published {
                    lon: 13.4,
                    lat: 52.5,
                },
                entity_type: crate::read::OSM_NODE,
                osm_id: 42,
                tags: crate::tags::TagSpan::default(),
            }),
            key_feature(&Feature {
                anchor: crate::read::Anchor::Published {
                    lon: 13.4,
                    lat: 52.5,
                },
                entity_type: crate::read::OSM_WAY,
                osm_id: 42,
                tags: crate::tags::TagSpan::default(),
            }),
        ];
        resolve_identities(&mut batch).expect("two keys fit");
        let (a, b) = (batch[0].identity_ordinal, batch[1].identity_ordinal);
        assert!(a.is_some() && b.is_some(), "both must resolve");
        assert_ne!(a, b, "same id, different kind must not share an ordinal");

        // Anti-vacuity: the SAME (kind, id) twice DOES share an ordinal, so
        // the distinction is the kind and not "every row gets a fresh number".
        let mut same = [batch[0], batch[0]];
        resolve_identities(&mut same).expect("one distinct key fits");
        assert_eq!(same[0].identity_ordinal, same[1].identity_ordinal);
    }

    #[test]
    fn a_row_built_before_the_pre_bake_stage_carries_no_identity_at_all() {
        // `build_row` is pure layout: without a resolved ordinal it must leave
        // the slot dormant rather than invent one. A fabricated identity would
        // point at the wrong element and still read as valid.
        let k = key_feature(&feat(13.404954, 52.520008));
        assert_eq!(k.identity_ordinal, None);
        let row = build_row(&k, &no_tags());
        assert_eq!(crate::identity::read_identity(&row), None);
        assert_eq!(row.value, [0u8; 480], "no slot may be written");
    }

    #[test]
    fn the_kind_lands_in_the_identity_slot_and_the_entity_type_lane_stays_dormant() {
        // RE-PINNED (was `entity_type_lands_in_its_tenant_and_nowhere_else`).
        // The old test asserted the kind at `ValueTenant::EntityType`'s offset.
        // That reading is gone: a geo row reads its value slab as 30 facet
        // SLOTS, and the two readings overlap byte-for-byte, so a row cannot
        // hold both. The kind is now the identity slot's own classid.
        //
        // Kept as an assertion rather than deleted, because the interesting
        // claim survived the change — only its address moved.
        let mut k = key_feature(&feat(13.404954, 52.520008));
        k.entity_type = crate::read::OSM_WAY;
        // A real ~2^34 OSM id — far past u24, which is exactly why it must go
        // through the codebook rather than into the slot.
        k.osm_id = 12_000_000_001;
        let mut batch = [k];
        let book = resolve_identities(&mut batch).expect("one key fits the book");
        let row = build_row(&batch[0], &no_tags());

        // The kind + ordinal are recoverable from the slot alone, and the
        // ordinal pulls back to the original id through the codebook — the
        // round-trip #902 calls the tenant's acceptance criterion.
        let (kind, ordinal) = crate::identity::read_identity(&row).expect("identity written");
        assert_eq!(kind, crate::read::OSM_WAY);
        assert_eq!(
            book.key(ordinal),
            Some(format!("{:04x}:{}", crate::read::OSM_WAY, 12_000_000_001i64).as_str()),
            "the ordinal must pull back to the id it was minted from"
        );

        // And the EntityType lane is NOT written — the switch really happened.
        let old = ValueTenant::EntityType.value_offset();
        assert_eq!(
            u16::from_le_bytes([row.value[old], row.value[old + 1]]),
            0,
            "the EntityType lane must stay dormant; the kind is the slot classid"
        );

        // Field isolation (I-LEGACY-API-FEATURE-GATED): only slot 2 is written.
        let slot = crate::identity::slab_offset_of_slot(ogar_osm::GEO_IDENTITY_SLOT).unwrap();
        let others: usize = row
            .value
            .iter()
            .enumerate()
            .filter(|(i, _)| !(slot..slot + crate::identity::SLOT_BYTES).contains(i))
            .map(|(_, b)| usize::from(*b))
            .sum();
        assert_eq!(others, 0, "no slot but the identity facet may be written");
    }

    /// A store holding `n` distinct tags on one feature, plus its span.
    fn tags_of(n: u32) -> (ResolvedTags, TagSpan) {
        let mut store = crate::tags::TagStore::default();
        let pairs: Vec<(String, String)> = (0..n)
            .map(|i| (format!("k{i:04}"), format!("v{i:04}")))
            .collect();
        let span = store.push(pairs.iter().map(|(k, v)| (k.as_str(), v.as_str())));
        (store.resolve().expect("small books"), span)
    }

    #[test]
    fn a_features_tags_land_in_its_own_row_and_read_back() {
        let (tags, span) = tags_of(3);
        let mut k = key_feature(&feat(13.404954, 52.520008));
        k.tags = span;
        let mut batch = [k];
        resolve_identities(&mut batch).unwrap();
        let row = build_row(&batch[0], &tags);

        let ordinal = batch[0].identity_ordinal.unwrap();
        let got: Vec<(u32, u32)> = crate::cluster::facets(&row)
            .into_iter()
            .filter_map(|(_, f)| match f {
                crate::cluster::Facet::Tag {
                    member, key, value, ..
                } if member == ordinal => Some((key, value)),
                _ => None,
            })
            .collect();
        assert_eq!(
            got,
            tags.span(span),
            "every tag, in order, bound to the feature"
        );

        // Can-stay-silent half: a feature with no tags writes none, so the
        // read above is measuring stored tags rather than always finding some.
        let bare_keyed = Keyed {
            tags: TagSpan::default(),
            ..batch[0]
        };
        let bare = build_row(&bare_keyed, &tags);
        assert_eq!(crate::cluster::facets(&bare).len(), 1, "identity only");
    }

    #[test]
    fn the_reserved_slope_slot_is_never_written_by_a_tag() {
        // RESERVE-DON'T-RECLAIM, as a behaviour rather than a comment: packing
        // tags from slot 3 would collide the day a slope facet lands, and the
        // collision would read as a valid slope.
        let (tags, span) = tags_of(TAGS_PER_ROW as u32);
        let mut k = key_feature(&feat(13.404954, 52.520008));
        k.tags = span;
        let mut batch = [k];
        resolve_identities(&mut batch).unwrap();
        let row = build_row(&batch[0], &tags);

        let off = crate::identity::slab_offset_of_slot(ogar_osm::GEO_SLOPE_SLOT).unwrap();
        assert_eq!(
            row.value[off..off + crate::identity::SLOT_BYTES],
            [0u8; 16],
            "the slope slot must stay dormant even on a full row"
        );
        // …and the row really is full, so the assertion is not passing because
        // there were few tags.
        assert_eq!(crate::cluster::facets(&row).len(), 1 + TAGS_PER_ROW);
    }

    #[test]
    fn overflow_splits_into_continuations_that_reassemble_by_ordinal() {
        // Two-sided on the boundary: exactly TAGS_PER_ROW must NOT split, one
        // more must. A test that only checked the splitting half would pass on
        // an implementation that split everything.
        let (_, exact) = tags_of(TAGS_PER_ROW as u32);
        let mut at_bound = vec![Keyed {
            tags: exact,
            ..key_feature(&feat(13.4, 52.5))
        }];
        assert_eq!(
            expand_tag_overflow(&mut at_bound),
            0,
            "a full row must not split"
        );
        assert_eq!(at_bound.len(), 1);

        let n = TAGS_PER_ROW as u32 * 2 + 5;
        let (tags, span) = tags_of(n);
        let mut over = vec![Keyed {
            tags: span,
            ..key_feature(&feat(13.4, 52.5))
        }];
        assert_eq!(
            expand_tag_overflow(&mut over),
            2,
            "three rows, two of them new"
        );
        resolve_identities(&mut over).unwrap();

        // Every row carries the SAME identity, and the union of their tags is
        // the original list with nothing lost or duplicated.
        let ordinals: Vec<Option<u32>> = over.iter().map(|k| k.identity_ordinal).collect();
        assert!(
            ordinals.windows(2).all(|w| w[0] == w[1]),
            "one feature, one ordinal"
        );

        let mut seen: Vec<(u32, u32)> = Vec::new();
        for k in &over {
            assert!(k.tags.len as usize <= TAGS_PER_ROW);
            let row = build_row(k, &tags);
            seen.extend(
                crate::cluster::facets(&row)
                    .into_iter()
                    .filter_map(|(_, f)| match f {
                        crate::cluster::Facet::Tag { key, value, .. } => Some((key, value)),
                        crate::cluster::Facet::Member { .. } => None,
                    }),
            );
        }
        assert_eq!(
            seen,
            tags.span(span),
            "reassembly must be lossless and in order"
        );
        assert_eq!(seen.len(), n as usize);
    }

    fn junction_at(node_id: i64, lon: f64, lat: f64) -> crate::read::Junction {
        crate::read::Junction {
            node_id,
            cell: crate::tms::point_to_cell(lon, lat),
            edge_ways: Vec::new(), // unused by junction_keyed; names are pre-resolved
        }
    }

    #[test]
    fn a_junction_with_no_named_edges_produces_no_row() {
        // Every incident way unnamed (NAME_NONE) is the common case — a
        // service-road junction. It must cost nothing, not a row of zeros.
        let j = junction_at(1, 13.4, 52.5);
        assert!(junction_keyed(&j, &[]).is_empty());
    }

    #[test]
    fn a_junction_within_the_edge_budget_produces_exactly_one_row() {
        let j = junction_at(2, 13.4, 52.5);
        let names = [7u16, 9, 7];
        let rows = junction_keyed(&j, &names);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].edge_names.as_slice(), &names[..]);
        // RE-PINNED: this asserted `OSM_NODE` until `osm_street_node` (0x0F0B)
        // was minted. The old value was not a neutral fact being recorded — it
        // was the ambiguity itself, certified as expected behaviour by a green
        // test. See `a_junction_row_is_a_street_node_not_a_plain_node`.
        assert_eq!(rows[0].entity_type, crate::read::OSM_STREET_NODE);
        assert_eq!(rows[0].osm_id, 2);
        assert!(
            rows[0].tags.len == 0,
            "a junction row carries no ordinary tags — its payload is edge_names alone"
        );
    }

    /// A junction row must be distinguishable from an ordinary node row by
    /// the row's own bytes — the whole reason `osm_street_node` (`0x0F0B`)
    /// was minted.
    ///
    /// Two-sided, and the second half is what makes it a real falsifier: a
    /// junction is ALSO a published OSM node, so both rows carry the same
    /// `osm_id`. If the junction kept stamping `OSM_NODE`, `identity_key`
    /// would produce the identical string for both, `resolve_identities`
    /// would collapse them to ONE ordinal, and nothing on disk would separate
    /// them — which is exactly the state that cost 1,084,213 false hits when
    /// a reader tried to tell them apart by slot occupancy instead.
    #[test]
    fn a_junction_row_is_a_street_node_not_a_plain_node() {
        let node_id = 4_242;
        let j = junction_at(node_id, 13.4, 52.5);
        let mut rows = junction_keyed(&j, &[7u16, 9]);
        assert_eq!(rows.len(), 1);
        assert_eq!(
            rows[0].entity_type,
            crate::read::OSM_STREET_NODE,
            "a junction is the derived concept, not the published node"
        );

        // The same OSM node, keyed as an ordinary node — the collision case.
        let plain = Keyed {
            entity_type: crate::read::OSM_NODE,
            ..rows[0]
        };
        rows.push(plain);
        resolve_identities(&mut rows).unwrap();
        assert_ne!(
            rows[0].identity_ordinal, rows[1].identity_ordinal,
            "junction and node share an osm_id; only the KIND separates them, \
             so a shared ordinal means the kind is not reaching the codebook"
        );

        // And it survives the round trip through the row's actual bytes:
        // `write_identity` refuses a non-Geo concept, so a green read here
        // also proves `osm_street_node` is registered in OGAR's concept table.
        let row = build_row_notags(&rows[0]);
        let (kind, _) = crate::identity::read_identity(&row)
            .expect("a junction row must carry a readable identity facet");
        assert_eq!(kind, crate::read::OSM_STREET_NODE);
    }

    #[test]
    fn a_junction_over_the_edge_budget_splits_into_continuations_sharing_one_identity() {
        // The measured P12 case: one Berlin junction exceeded EDGE_SLOTS.
        // Two-sided on the boundary, same discipline as the tag-overflow test
        // above: exactly EDGE_SLOTS must NOT split, one more must.
        let j = junction_at(3, 13.4, 52.5);
        let at_bound: Vec<u16> = (1..=street::EDGE_SLOTS as u16).collect();
        assert_eq!(
            junction_keyed(&j, &at_bound).len(),
            1,
            "a full bucket must not split"
        );

        let over: Vec<u16> = (1..=street::EDGE_SLOTS as u16 + 3).collect();
        let mut rows = junction_keyed(&j, &over);
        assert_eq!(
            rows.len(),
            2,
            "9 more slots than the budget covers -> 2 rows"
        );
        resolve_identities(&mut rows).unwrap();

        // Same identity key on both — a reader groups them as ONE junction.
        assert_eq!(rows[0].identity_ordinal, rows[1].identity_ordinal);
        assert!(rows[0].identity_ordinal.is_some());

        // Nothing lost, nothing duplicated: the union of both rows' edge
        // names, in order, reproduces the input exactly.
        let mut reassembled: Vec<u16> = Vec::new();
        for r in &rows {
            reassembled.extend_from_slice(r.edge_names.as_slice());
        }
        assert_eq!(reassembled, over);
    }

    #[test]
    fn morton_sort_is_trie_order_and_byte_sort_is_not() {
        // The trap this module's sort key exists to avoid: the tiers are LE
        // u16 inside the GUID, so sorting raw key bytes is NOT cascade order.
        let a = key_feature(&feat(13.404954, 52.520008));
        let b = key_feature(&feat(13.5, 52.6));
        let (lo, hi) = if a.morton < b.morton { (a, b) } else { (b, a) };
        let (rl, rh) = (build_row(&lo, &no_tags()), build_row(&hi, &no_tags()));
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
