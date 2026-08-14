//! A **street is an ABI DTO over edges** — a masked projection, not a row.
//!
//! Nothing named "street" is stored. The DTO is two `u16`s of identity, and
//! every question it answers is a mask read or a walk over bytes that already
//! exist:
//!
//! | | |
//! |---|---|
//! | identity | a **name ordinal** into the label codebook |
//! | membership | a [`WideFieldMask`] over a junction's ≤ 8 edge slots |
//! | geometry | a walk over `EdgeBlock` adjacency in Morton order |
//!
//! # Why this is cheap, measured
//!
//! Berlin's drivable network: **96,340 named ways over 10,481 distinct
//! names** — 9.2 ways per street. Those ways form a *connected corridor*, and
//! the corpus is Morton-sorted, so a street's rows sit near each other and the
//! walk touches a few pages rather than scanning. Interning the 10,481 names
//! once is what makes the label free; the walk is what makes the entity
//! unnecessary.
//!
//! Max junction degree is **7** (measured, none above 12), so a junction's
//! edges fit 8 slots and the membership mask is one byte of bits.
//!
//! This is the `stockfish-rs` access model on a road graph: `ButterflyHistory`
//! is `[from·64 + to]`, a **computed index into a flat array** — no hash, no
//! search — and here the next junction is a computed address through the
//! 16-byte edge block.
//!
//! # Status of the name lane
//!
//! The per-edge name ordinals are read from a **value slot**, not a
//! `ValueTenant`: the canonical tenant table has no edge-name lane, and
//! inventing a tenant ordinal upstream is the operator's call, surfaced not
//! filed. Slot addressing is the canon's own 32-slot reading of the row
//! (`GUIDS_PER_NODE = 32`; key + edges take 2, leaving 30 slots for the
//! class-resolved layout — *"Tetris it across the slots"*). [`NAME_SLOT`] is
//! therefore a **proposed slot pending a ClassView**, addressed by slot index
//! and never by a literal tenant offset.
//!
//! **`NAME_SLOT` = 1 (`ogar_osm::GEO_SLOPE_SLOT`) — measured, not guessed.**
//! An earlier draft of this module moved it to value-local slot 2
//! (`row::FIRST_TAG_SLOT`), reasoning that a junction row carries no ordinary
//! tags so nothing else would ever write there. **That reasoning was correct
//! about junction rows and wrong about every OTHER `osm_node` row.** Slot 2
//! is where an ORDINARY tagged node's *first tag* lands — and this format has
//! no byte that distinguishes "a junction row using NAME_SLOT" from "a
//! tagged-POI row using its first tag slot" once the bytes are on disk. A
//! reader (`parity`'s junction check) that tried slot 2 on every `osm_node`
//! row measured **1,084,213 false hits on real Berlin data** — the exact
//! count of tagged nodes carrying at least one tag, reinterpreting each
//! node's first `Facet::Tag` bytes as eight raw `u16` name ordinals.
//!
//! Slot 1 has no such reader ambiguity: `GEO_SLOPE_SLOT` is written by
//! **nothing** in the ordinary tag path (`row::FIRST_TAG_SLOT` starts one
//! slot later, deliberately skipping it — see `row.rs`'s own doc, "a bake
//! that packed tags into it would silently collide the day a slope lands").
//! Junction rows are its first real, deliberate consumer; a future slope
//! feature is a **later** decision that gets to pick where it lands with this
//! precedent on the table, not a collision this module can silently walk
//! into. Measured: `crates/tests` junction parity is clean at slot 1, and
//! `row::tests::the_reserved_slope_slot_is_never_written_by_a_tag` (an
//! ORDINARY row, `edge_names.len == 0`) is unaffected either way — the
//! junction-write path never runs for that row regardless of which slot
//! `NAME_SLOT` names.

use lance_graph_contract::canonical_node::NodeRow;
use lance_graph_contract::class_view::WideFieldMask;

/// Edge slots a junction can carry — matches `EdgeBlock`'s 12 in-family
/// slots with room, and the measured maximum degree of **7**.
pub const EDGE_SLOTS: u8 = 8;

/// Value-slab slot holding this row's per-edge name ordinals, as
/// `EDGE_SLOTS × u16`. Value-local slot 1 = `ogar_osm::GEO_SLOPE_SLOT` — see
/// the module docs for why this is the ONLY slot with no reader ambiguity.
///
/// Proposed, pending a ClassView — see the module docs.
pub const NAME_SLOT: usize = 1;

// The invariant the module doc argues for, checked against the real
// constant rather than restated as a number: NAME_SLOT must stay exactly on
// the reserved slope slot (the one slot no ordinary tagged row ever writes),
// never drift onto the tag range that starts immediately after it.
const _: () = assert!(
    NAME_SLOT == ogar_osm::GEO_SLOPE_SLOT - ogar_osm::RESERVED_SLOTS,
    "NAME_SLOT must be exactly GEO_SLOPE_SLOT (value-local) — see module docs"
);

/// A name ordinal that means "no name" — the zero-fallback default, so an
/// unnamed edge is *absent* rather than a member of street 0.
pub const NAME_NONE: u16 = 0;

/// Byte offset of a value slot within [`NodeRow::value`].
#[inline]
#[must_use]
const fn slot_offset(slot: usize) -> usize {
    slot * 16
}

/// Is this row a junction — i.e. do the edge lanes mean anything on it?
///
/// Reads the kind from the identity facet, which is the ONLY thing on disk
/// that distinguishes a junction row from an ordinary node row: both are 512
/// bytes, and before `osm_street_node` (`0x0F0B`) existed they were
/// indistinguishable. See this module's docs for what that cost.
#[must_use]
pub fn is_street_node(row: &NodeRow) -> bool {
    matches!(
        crate::identity::read_identity(row),
        Some((kind, _)) if kind == crate::read::OSM_STREET_NODE
    )
}

/// The name ordinal on edge `slot` of this junction.
///
/// Returns [`NAME_NONE`] for any row that is not a junction — the gate lives
/// here, in the accessor, rather than in a rule callers are asked to
/// remember. Every other read in this module ([`edge_mask`], [`StreetDto`])
/// funnels through it, so one check covers the surface.
#[must_use]
pub fn edge_name(row: &NodeRow, edge: u8) -> u16 {
    if !is_street_node(row) {
        return NAME_NONE;
    }
    edge_name_unchecked(row, edge)
}

/// [`edge_name`] without the per-edge kind check — for callers that have
/// already established the row is a junction. Private on purpose: the gate is
/// only reliable if the public surface cannot bypass it.
#[inline]
fn edge_name_unchecked(row: &NodeRow, edge: u8) -> u16 {
    if edge >= EDGE_SLOTS {
        return NAME_NONE;
    }
    let o = slot_offset(NAME_SLOT) + (edge as usize) * 2;
    u16::from_le_bytes([row.value[o], row.value[o + 1]])
}

/// **The masked projection**: which of this junction's edges carry `name`.
///
/// Bit `i` set means edge slot `i` belongs to the street. `NAME_NONE` never
/// matches, so unnamed edges are excluded rather than pooled.
#[must_use]
pub fn edge_mask(row: &NodeRow, name: u16) -> WideFieldMask {
    let mut m = WideFieldMask::EMPTY;
    // Gate ONCE for the whole sweep, then read unchecked — the kind cannot
    // change between edges of the same row, so re-parsing the identity facet
    // eight times would be pure cost.
    if name == NAME_NONE || !is_street_node(row) {
        return m;
    }
    for e in 0..EDGE_SLOTS {
        if edge_name_unchecked(row, e) == name {
            m = m.with(e);
        }
    }
    m
}

/// A street — identity only. Everything else is projection.
///
/// Four bytes wide by construction: this is a DTO over edges, not a record of
/// them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreetDto {
    /// Codebook ordinal — the street's identity and its label address.
    pub name: u16,
    /// Edge slot on the seed junction where the walk starts.
    pub seed_edge: u8,
}

impl StreetDto {
    /// The street a given edge belongs to, or `None` if that edge is unnamed.
    #[must_use]
    pub fn at(row: &NodeRow, edge: u8) -> Option<Self> {
        let name = edge_name(row, edge);
        (name != NAME_NONE).then_some(Self {
            name,
            seed_edge: edge,
        })
    }

    /// This street's edges at `row` — the projection, as a mask.
    #[must_use]
    pub fn mask_at(&self, row: &NodeRow) -> WideFieldMask {
        edge_mask(row, self.name)
    }

    /// Does the street continue through this junction? True when it has **at
    /// least two** edges here — one in, one out. A single edge is an endpoint,
    /// not a continuation, and that distinction is what terminates a walk.
    #[must_use]
    pub fn continues_through(&self, row: &NodeRow) -> bool {
        self.mask_at(row).count() >= 2
    }
}

/// Write per-edge name ordinals into a row's name slot.
///
/// Baker-side; kept here so the read and the write share one layout.
pub fn set_edge_names(row: &mut NodeRow, names: &[u16]) {
    let base = slot_offset(NAME_SLOT);
    for (e, n) in names.iter().take(EDGE_SLOTS as usize).enumerate() {
        let o = base + e * 2;
        row.value[o..o + 2].copy_from_slice(&n.to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::row::{build_row_notags, Keyed};
    use crate::tms::tiers_of;

    fn junction(names: &[u16]) -> NodeRow {
        let mut r = build_row_notags(&Keyed {
            morton: 42,
            tiers: tiers_of(42),
            // A REAL junction row: the kind the bake stamps, plus an ordinal
            // so the identity facet is actually written. The previous helper
            // used `OSM_WAY` with no ordinal — a row that cannot exist in a
            // bake, and which only passed because nothing read the kind.
            entity_type: crate::read::OSM_STREET_NODE,
            osm_id: 0,
            identity_ordinal: Some(1),
            identity: 0,
            tags: crate::tags::TagSpan::default(),
            edge_names: crate::row::EdgeNames::default(),
        });
        set_edge_names(&mut r, names);
        r
    }

    /// The edge lanes are readable ONLY on a row whose identity facet says
    /// `osm_street_node`. This is the invariant that makes the rest of the
    /// class-resolved layout safe to build on.
    ///
    /// Until `osm_street_node` (`0x0F0B`) existed, the name lane's only
    /// protection was HIDING — it sat at the one slot the tag path never
    /// writes, so a misdirected read found zeros. That is not a gate, it is a
    /// vacancy, and the module docs above record what happened when a reader
    /// tried the next slot along: **1,084,213 false hits** on real Berlin
    /// data. Every new lane (bearing, turn matrix, access) needs slots past
    /// that vacancy, so the protection has to become an actual check.
    ///
    /// It lives INSIDE the accessor on purpose. A documented "callers must
    /// check the kind first" is a rule that holds until someone forgets;
    /// `edge_name` refusing to answer for a non-junction row cannot be
    /// forgotten.
    #[test]
    fn edge_lanes_are_mute_on_a_row_that_is_not_a_street_node() {
        // A tagged POI, with bytes in the name slot — the shape a misdirected
        // read would find. It must decode to nothing, not to eight streets.
        let mut poi = build_row_notags(&Keyed {
            morton: 42,
            tiers: tiers_of(42),
            entity_type: crate::read::OSM_NODE,
            osm_id: 7,
            identity_ordinal: Some(7),
            identity: 0,
            tags: crate::tags::TagSpan::default(),
            edge_names: crate::row::EdgeNames::default(),
        });
        set_edge_names(&mut poi, &[11u16, 12, 13]);

        assert!(!is_street_node(&poi), "an osm_node row is not a junction");
        for e in 0..EDGE_SLOTS {
            assert_eq!(
                edge_name(&poi, e),
                NAME_NONE,
                "edge {e} must be mute on a non-junction row"
            );
        }
        assert_eq!(edge_mask(&poi, 11).count(), 0);
        assert!(StreetDto::at(&poi, 0).is_none());

        // Two-sided: the identical bytes on a real junction row DO decode, so
        // this test cannot pass by the lanes being broken for everyone.
        let j = junction(&[11u16, 12, 13]);
        assert!(is_street_node(&j));
        assert_eq!(edge_name(&j, 0), 11);
        assert_eq!(edge_mask(&j, 11).count(), 1);
    }

    #[test]
    fn the_mask_selects_exactly_the_edges_carrying_that_name() {
        // A 4-way junction where two edges are Hauptstrasse (7) and two are
        // a cross street (9) — the ordinary case.
        let r = junction(&[7, 9, 7, 9]);
        let m = edge_mask(&r, 7);
        assert_eq!(m.count(), 2);
        assert!(m.has(0) && m.has(2));
        assert!(
            !m.has(1) && !m.has(3),
            "the cross street must not be included"
        );
        assert_eq!(edge_mask(&r, 9).count(), 2);
    }

    #[test]
    fn unnamed_edges_are_absent_rather_than_pooled_into_street_zero() {
        // 54% of Berlin's drivable ways are unnamed (mostly service roads).
        // If NAME_NONE matched, they would all become one enormous street.
        let r = junction(&[0, 0, 5, 0]);
        assert_eq!(
            edge_mask(&r, NAME_NONE).count(),
            0,
            "street 0 must be empty"
        );
        assert_eq!(edge_mask(&r, 5).count(), 1);
        assert!(
            StreetDto::at(&r, 0).is_none(),
            "an unnamed edge has no street"
        );
        // ...paired with the named one, so the guard discriminates.
        assert_eq!(StreetDto::at(&r, 2).unwrap().name, 5);
    }

    #[test]
    fn a_street_continues_through_a_junction_only_with_two_or_more_edges() {
        let through = junction(&[7, 9, 7, 9]);
        let ends = junction(&[7, 9, 9, 9]);
        let s = StreetDto {
            name: 7,
            seed_edge: 0,
        };
        assert!(s.continues_through(&through));
        assert!(
            !s.continues_through(&ends),
            "one edge is an endpoint, not a through"
        );
    }

    #[test]
    fn the_dto_is_identity_only_and_stays_four_bytes() {
        // The point of the design: a street is not a record of its edges.
        assert!(core::mem::size_of::<StreetDto>() <= 4);
    }

    #[test]
    fn edge_slots_past_the_budget_read_as_unnamed_not_out_of_bounds() {
        // Max measured junction degree is 7; slot 8+ must be a clean miss
        // rather than a panic or a read into the next slot.
        let r = junction(&[7, 7, 7, 7, 7, 7, 7, 7]);
        assert_eq!(edge_name(&r, EDGE_SLOTS), NAME_NONE);
        assert_eq!(edge_name(&r, 200), NAME_NONE);
        assert_eq!(edge_mask(&r, 7).count(), u32::from(EDGE_SLOTS));
    }

    #[test]
    fn names_land_in_the_name_slot_and_touch_no_other_slot() {
        // Field isolation: writing names must not disturb any neighbouring
        // slot, including the EntityType lane the row already carries.
        let mut r = junction(&[]);
        let before = r.value;
        set_edge_names(&mut r, &[1, 2, 3]);
        let base = slot_offset(NAME_SLOT);
        for (i, (a, b)) in before.iter().zip(r.value.iter()).enumerate() {
            if !(base..base + 16).contains(&i) {
                assert_eq!(a, b, "byte {i} changed outside the name slot");
            }
        }
    }
}
