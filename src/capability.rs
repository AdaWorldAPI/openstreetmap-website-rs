//! The consumer half of the OGAR plug-and-play roundtrip.
//!
//! `ogar_vocab::geo_actions` declares WHAT the geo surface exposes and WHO is
//! expected to execute it; this module is `osm-soa-bake` **plugging in** —
//! [`activate`] hands its classids to the port
//! (`resolve_hotplug`) and gets back the activated vocab rows and capability
//! names. Drift bangs once, here: a capability added upstream with no arm, an
//! arm for a capability upstream dropped, or a wrong classid set.
//!
//! An earlier version of this module built a `CapabilityRegistration` and
//! called a verifier that `ogar-osm` also owned — a consumer checking itself
//! against a table that was never in `domain_tables()`. Both halves passed
//! while the real port answered `NoCapabilitiesFor(0x0F01)`.
//!
//! # Why the covered list is derived, not written
//!
//! A hand-written `covered: &["locate_point", …]` slice would satisfy the
//! registry while naming functions that do not exist — the registration would
//! be *structurally* valid and *semantically* fiction. So [`Capability`] is an
//! enum with one variant per capability, [`Capability::ALL`] is the covered
//! set, and every variant has a real dispatch function below that delegates to
//! the primitive it names. Adding a name to the enum without a function does
//! not compile; adding a function without the enum variant leaves it
//! unregistered and the roundtrip fails.
//!
//! # These are thin, on purpose
//!
//! Each arm is a one-line delegation. The dispatch surface exists so the
//! registration is checkable, not to add a layer: a caller that already has
//! the module in scope should keep calling `tms::point_to_tiers` directly.

use lance_graph_contract::canonical_node::NodeRow;
use lance_graph_contract::class_view::WideFieldMask;
use ogar_osm::{plug_in, GEO_SUBJECT_CLASSIDS};
use ogar_vocab::capability_registry::{HotplugActivation, HotplugDrift};

use crate::fragment::BoundaryIndex;
use crate::project::Projected;
use crate::slab::RowSlab;
use crate::tms::Tiers;
use crate::{geodesy, project, street, tms};

/// This crate's name, as the upstream table's expected-executor list spells it.
pub const CONSUMER: &str = "osm-soa-bake";

/// One capability this crate executes. The variant set IS the covered set —
/// see the module doc for why it is not a hand-written slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Capability {
    /// `lon, lat` → Morton + the four cascade tiers.
    LocatePoint,
    /// `z, x, y` → the contiguous row range of that tile.
    LocateTile,
    /// A Morton interval → the fragments spanning it.
    LocateFragment,
    /// A row × `surface ∩ role` → the admitted facet positions and bytes.
    ProjectFields,
    /// A junction row × street name → the mask of edges carrying it.
    StreetEdges,
    /// An ordered polyline → its geodesic length in metres.
    PolylineLength,
}

impl Capability {
    /// Every capability, in the upstream table's order.
    pub const ALL: &'static [Capability] = &[
        Capability::LocatePoint,
        Capability::LocateTile,
        Capability::LocateFragment,
        Capability::ProjectFields,
        Capability::StreetEdges,
        Capability::PolylineLength,
    ];

    /// The upstream capability name this variant executes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Capability::LocatePoint => "locate_point",
            Capability::LocateTile => "locate_tile",
            Capability::LocateFragment => "locate_fragment",
            Capability::ProjectFields => "project_fields",
            Capability::StreetEdges => "street_edges",
            Capability::PolylineLength => "polyline_length",
        }
    }
}

/// The covered-capability names, derived from [`Capability::ALL`].
///
/// A `const` cannot iterate, so this is built once at first use rather than
/// duplicated as a literal — duplicating it is exactly the drift the enum
/// exists to prevent.
#[must_use]
pub fn covered() -> Vec<&'static str> {
    Capability::ALL.iter().map(|c| c.name()).collect()
}

/// Plug this crate's geo classids into the OGAR port and activate.
///
/// The consumer supplies three things and nothing else: who it is, which
/// classids it is plugging in, and which capability arms it covers. OGAR
/// verifies every id is minted, that each contributes at least one capability,
/// that this consumer is expected by every contributing table, and that
/// coverage holds in both directions — then returns the activated vocab rows
/// and capability names.
///
/// # Errors
///
/// The [`HotplugDrift`] the port reported.
pub fn activate() -> Result<HotplugActivation, HotplugDrift> {
    plug_in(CONSUMER, GEO_SUBJECT_CLASSIDS, &covered())
}

// ── The dispatch arms ───────────────────────────────────────────────

/// [`Capability::LocatePoint`] — a coordinate to its cascade address.
#[must_use]
pub fn locate_point(lon: f64, lat: f64) -> (u64, Tiers) {
    tms::point_to_tiers(lon, lat)
}

/// [`Capability::LocateTile`] — a tile to its contiguous row range.
///
/// Containment, not search: a Morton prefix IS the tile, so the endpoints are
/// computed by binary search over an already-sorted slab.
#[must_use]
pub fn locate_tile(slab: &RowSlab<'_>, z: u32, x: u32, y: u32) -> core::ops::Range<usize> {
    slab.tile_range(z, x, y)
}

/// [`Capability::LocateFragment`] — a Morton interval to the fragments
/// spanning it, so only those files are fetched.
#[must_use]
pub fn locate_fragment(index: &BoundaryIndex, lo: u64, hi: u64) -> core::ops::Range<usize> {
    index.locate_range(lo, hi)
}

/// [`Capability::ProjectFields`] — the masked projection, `surface ∩ role`.
///
/// Fail-closed: an unauthorised position is absent from the result, not hidden
/// by a caller.
#[must_use]
pub fn project_fields(
    row: &NodeRow,
    surface: &WideFieldMask,
    role: &WideFieldMask,
) -> Vec<Projected> {
    project::project(row, surface, role).collect()
}

/// [`Capability::StreetEdges`] — which edges of a junction carry a street.
#[must_use]
pub fn street_edges(row: &NodeRow, name: u16) -> WideFieldMask {
    street::edge_mask(row, name)
}

/// [`Capability::PolylineLength`] — geodesic length of an ordered polyline.
#[must_use]
pub fn polyline_length(points: &[(f64, f64)]) -> f64 {
    geodesy::polyline_metres(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::row::{build_row, key_feature};
    use ogar_osm::GEO_ACTION_NAMES;

    #[test]
    fn plugging_in_activates_against_the_real_port() {
        let (concepts, capabilities) = activate().expect("the geo domain must activate");
        assert_eq!(capabilities.len(), GEO_ACTION_NAMES.len());
        assert_eq!(concepts.len(), GEO_SUBJECT_CLASSIDS.len());
        // The activation names the concepts back, so the consumer learns what
        // it plugged into rather than only that it was accepted.
        let names: Vec<&str> = concepts.iter().map(|&(n, _)| n).collect();
        assert!(names.contains(&"osm_node") && names.contains(&"osm_way"));
    }

    #[test]
    fn the_port_refuses_an_arm_this_crate_does_not_actually_cover() {
        // Can-fire half: activation above is not "the port accepts anything".
        // Dropping one arm from the covered set must be rejected, which is
        // what makes the passing case meaningful.
        let short: Vec<&str> = covered().into_iter().take(1).collect();
        assert!(matches!(
            plug_in(CONSUMER, GEO_SUBJECT_CLASSIDS, &short),
            Err(HotplugDrift::Uncovered(_))
        ));
        assert!(matches!(
            plug_in("some-other-crate", GEO_SUBJECT_CLASSIDS, &covered()),
            Err(HotplugDrift::UnexpectedConsumer(_))
        ));
    }

    #[test]
    fn the_covered_set_is_derived_from_the_enum_and_matches_upstream_exactly() {
        // Both directions, so neither side can grow silently.
        let mine = covered();
        assert_eq!(mine.len(), Capability::ALL.len());
        for name in &mine {
            assert!(
                GEO_ACTION_NAMES.contains(name),
                "{name} has an arm here but is not declared upstream"
            );
        }
        for name in GEO_ACTION_NAMES {
            assert!(
                mine.contains(name),
                "{name} is declared upstream but has no arm here"
            );
        }
    }

    #[test]
    fn capability_names_are_distinct() {
        // A copy-paste that gave two variants the same name would still
        // satisfy the roundtrip if the duplicate matched a real entry.
        let mut names = covered();
        names.sort_unstable();
        let before = names.len();
        names.dedup();
        assert_eq!(names.len(), before, "two variants share a name");
    }

    #[test]
    fn every_arm_actually_executes_its_primitive() {
        // The non-vacuity check: a registration naming six functions is
        // fiction unless each one runs and returns something real. Each
        // assertion below would fail on a stub returning a default.
        let berlin = (13.404954, 52.520008);

        let (morton, tiers) = locate_point(berlin.0, berlin.1);
        assert_ne!(morton, 0);
        assert_ne!(tiers.leaf, 0, "the finest tier must be carried");

        // A slab of one real row, and the tile that contains it.
        let k = key_feature(&crate::read::Feature {
            lon: berlin.0,
            lat: berlin.1,
            entity_type: crate::read::OSM_NODE,
            osm_id: 1,
        });
        let row = build_row(&k);
        let bytes = unsafe {
            core::slice::from_raw_parts(
                core::ptr::from_ref(&row).cast::<u8>(),
                core::mem::size_of::<NodeRow>(),
            )
        };
        let slab = RowSlab::new(bytes).expect("one row is a valid slab");
        assert_eq!(locate_tile(&slab, 0, 0, 0), 0..1, "z=0 contains everything");

        let index = BoundaryIndex::new(vec![0, morton]).expect("sorted firsts");
        let found = locate_fragment(&index, morton, morton + 1);
        assert!(!found.is_empty(), "the point's own fragment must be found");

        // Projection: a full surface admits the facet register; an empty role
        // admits nothing. Both halves, so neither is vacuous.
        let full = WideFieldMask::full_for(crate::project::FACET_PAYLOAD_BYTES as usize);
        let empty = WideFieldMask::full_for(0);
        assert_eq!(
            project_fields(&row, &full, &full).len(),
            crate::project::FACET_PAYLOAD_BYTES as usize
        );
        assert!(project_fields(&row, &full, &empty).is_empty());

        // Street edges: an unnamed junction yields nothing; a named one does.
        let mut named = row;
        street::set_edge_names(&mut named, &[7, 7]);
        assert_eq!(street_edges(&row, 7).count(), 0);
        assert_eq!(street_edges(&named, 7).count(), 2);

        // Length: a real Berlin-scale segment, not a zero-length degenerate.
        let m = polyline_length(&[berlin, (13.5, 52.6)]);
        assert!(
            (8_000.0..14_000.0).contains(&m),
            "expected a ~10 km segment, got {m} m"
        );
    }
}
