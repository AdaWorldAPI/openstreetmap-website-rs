//! The **masked projection** — reading a row through a `WideFieldMask`,
//! without materialising anything.
//!
//! `lance_graph_contract::class_view::WideFieldMask` is the projection. Bit `i`
//! is a **layout address**, exactly as `ogar-render-askama`'s `FieldView`
//! documents it: *"`position` is the field's `WideFieldMask` bit index — its
//! layout address on the screen."*
//!
//! So a projection is bit arithmetic, and a value is a byte read from the
//! mmap'd row at the address the bit names. Nothing is gathered, nothing is
//! decoded, and no field that the mask excludes is ever touched.
//!
//! ```text
//! ClassView declares the field set
//!   ∩ role mask            → bit ops, fail-closed
//!   for each set bit i     → facet byte i, read from the mapped row
//!   → NodeDelta { key, mask_words, values }     the wire (charter T3)
//!   → FieldView → askama                        the membrane, set bits only
//! ```
//!
//! Materialisation happens only at the last line, only for bits the mask
//! already admitted. That is the bound: **the mask decides what exists**, so
//! the render cost is the projection, not the row.
//!
//! # Fail-closed
//!
//! [`project`] intersects. An absent or narrow role mask yields *fewer*
//! fields, never more — `WideFieldMask::full_for` is a render convenience and
//! must never be a substitute for a role mask (a2ui charter C1.4).

use lance_graph_contract::canonical_node::NodeRow;
use lance_graph_contract::class_view::WideFieldMask;

/// Bytes of the V3 content-blind facet payload — the register a ClassView
/// projects (le-contract §3). Field positions `0..12` address these bytes.
pub const FACET_PAYLOAD_BYTES: u8 = 12;

/// One projected field: its layout address and the byte at it.
///
/// Deliberately `Copy` and two bytes wide — this is a *view*, not a record.
/// The display label and formatting live in the ClassView, above the SoA;
/// the codebook is what makes them amortised rather than per-row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Projected {
    /// The `WideFieldMask` bit index — the layout address.
    pub position: u8,
    /// The facet byte at that address.
    pub byte: u8,
}

/// Project a row's facet payload through `surface ∩ role`.
///
/// Yields one [`Projected`] per admitted position, reading the row in place.
/// The iterator borrows; nothing is allocated, and positions outside the
/// 12-byte facet register are **skipped rather than folded** onto a byte —
/// the same rule `a2ui-server`'s `render_stream` applies (positions ≥ 12 are
/// value-slab fields read through a different surface).
pub fn project<'a>(
    row: &'a NodeRow,
    surface: &WideFieldMask,
    role: &WideFieldMask,
) -> impl Iterator<Item = Projected> + 'a {
    let admitted = surface.intersect(role);
    let key = row.key.as_bytes();
    (0..FACET_PAYLOAD_BYTES)
        .filter(move |&position| admitted.has(position))
        .map(move |position| Projected {
            position,
            // Facet payload begins after the 4-byte classid prefix.
            byte: key[4 + position as usize],
        })
}

/// The mask of positions this projection would actually yield — the surface
/// restricted to the facet register.
///
/// Useful as the `mask_words` half of a `NodeDelta`: the wire carries the
/// changed positions plus their LE bytes, and an unauthorised field is
/// **absent from the values**, not hidden downstream.
#[must_use]
pub fn projected_mask(surface: &WideFieldMask, role: &WideFieldMask) -> WideFieldMask {
    let admitted = surface.intersect(role);
    let mut out = WideFieldMask::EMPTY;
    for p in 0..FACET_PAYLOAD_BYTES {
        if admitted.has(p) {
            out = out.with(p);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::row::{build_row, Keyed};
    use crate::tms::tiers_of;

    fn row_with(code: u64) -> NodeRow {
        build_row(&Keyed {
            morton: code,
            tiers: tiers_of(code),
            entity_type: crate::read::OSM_WAY,
            osm_id: 0,
            identity: 0,
        })
    }

    #[test]
    fn a_projection_reads_the_facet_bytes_at_the_addresses_the_mask_names() {
        let row = row_with(0x0123_4567_89AB_CDEF);
        let full = WideFieldMask::full_for(FACET_PAYLOAD_BYTES as usize);
        let got: Vec<Projected> = project(&row, &full, &full).collect();
        assert_eq!(got.len(), FACET_PAYLOAD_BYTES as usize);
        // Each projected byte IS the row's key byte at that address — read in
        // place, not copied through an intermediate.
        for p in &got {
            assert_eq!(p.byte, row.key.as_bytes()[4 + p.position as usize]);
        }
    }

    #[test]
    fn the_role_mask_narrows_and_never_widens() {
        // Fail-closed: intersecting can only remove positions.
        let row = row_with(0xDEAD_BEEF_CAFE_F00D);
        let surface = WideFieldMask::full_for(FACET_PAYLOAD_BYTES as usize);
        let role = WideFieldMask::from_positions(&[1, 4, 9]);
        let got: Vec<u8> = project(&row, &surface, &role).map(|p| p.position).collect();
        assert_eq!(got, vec![1, 4, 9]);
        // A role naming positions the surface does not offer adds nothing.
        let greedy = WideFieldMask::from_positions(&[1, 200]);
        let got2: Vec<u8> = project(&row, &surface, &greedy)
            .map(|p| p.position)
            .collect();
        assert_eq!(got2, vec![1], "a role cannot widen the surface");
    }

    #[test]
    fn an_empty_role_projects_nothing_and_that_is_discriminating() {
        let row = row_with(1);
        let surface = WideFieldMask::full_for(FACET_PAYLOAD_BYTES as usize);
        assert_eq!(project(&row, &surface, &WideFieldMask::EMPTY).count(), 0);
        // ...paired with a role that DOES admit, so "nothing" means something.
        let one = WideFieldMask::from_positions(&[3]);
        assert_eq!(project(&row, &surface, &one).count(), 1);
    }

    #[test]
    fn positions_past_the_facet_register_are_skipped_not_folded() {
        // Position 12+ is a value-slab field read through a different surface.
        // Folding it onto a facet byte would silently show the wrong value.
        let row = row_with(0xFFFF_FFFF_FFFF_FFFF);
        let surface = WideFieldMask::from_positions(&[11, 12, 13, 63]);
        let got: Vec<u8> = project(&row, &surface, &surface)
            .map(|p| p.position)
            .collect();
        assert_eq!(got, vec![11], "only the in-register position projects");
    }

    #[test]
    fn projected_mask_agrees_with_what_project_yields() {
        // The wire's mask_words and the values must describe the same set, or
        // a client decodes values against the wrong addresses.
        let row = row_with(0x1122_3344_5566_7788);
        let surface = WideFieldMask::from_positions(&[0, 5, 11, 40]);
        let role = WideFieldMask::from_positions(&[0, 11, 40]);
        let m = projected_mask(&surface, &role);
        let yielded: Vec<u8> = project(&row, &surface, &role).map(|p| p.position).collect();
        assert_eq!(yielded, vec![0, 11]);
        assert_eq!(m.count() as usize, yielded.len());
        for p in &yielded {
            assert!(m.has(*p));
        }
        assert!(!m.has(40), "out-of-register positions are not advertised");
    }
}
