//! Per-edge access + oneway + bicycle contraflow — the attribute half that
//! [`crate::heading`] doesn't cover, and what an Abbiegeverbot bitmask needs
//! a slot-addressable edge next to.
//!
//! One byte per edge, same shape as `heading::pack_edge`:
//!
//! ```text
//! bit  0..3  access   (car | bike | foot, OR'd)
//! bit  3..5  oneway   (0 none, 1 forward, 2 backward)
//! bit  5     bicycle contraflow (allowed against a general oneway)
//! bit  6..8  spare
//! ```
//!
//! **Explicit tags only — no highway-class-implied defaults.** A router
//! commonly infers `oneway=yes` for `highway=motorway` even when untagged, or
//! `bicycle=no` for `highway=trunk`. Those conventions vary by region and
//! router, and guessing one here risks a silently wrong restriction — the
//! same reasoning that ruled out `Signed360` for headings. This module reads
//! only tags that are actually present; a way with none of these tags reads
//! as fully open, which is OSM's own default-access convention, not this
//! module's invention.
//!
//! ## Turn restrictions
//!
//! [`RestrictionMask`] is the P12-probe finding turned into a real type: a
//! restriction whose `from` and `to` ways are both incident at the `via`
//! junction (measured 99.1% of real restrictions, `junction_probe`) collapses
//! to a `(from_slot, to_slot)` bit in an 8×8 mask — one bit-test per turn, no
//! relation lookup at query time. The bitmask pack/unpack is proven here in
//! isolation; wiring real slot indices from actual junction rows is pending
//! the same row-writer `heading.rs` is waiting on.

/// Car may use this edge.
pub const ACCESS_CAR: u8 = 0b001;
/// Bicycle may use this edge.
pub const ACCESS_BIKE: u8 = 0b010;
/// Foot may use this edge.
pub const ACCESS_FOOT: u8 = 0b100;
/// All three modes — the default for a way that carries no access-denying tag.
pub const ACCESS_ALL: u8 = ACCESS_CAR | ACCESS_BIKE | ACCESS_FOOT;

/// No oneway restriction (or `oneway=no`).
pub const ONEWAY_NONE: u8 = 0;
/// `oneway=yes` / `oneway=1` / `oneway=true` — traversable way-order only.
pub const ONEWAY_FORWARD: u8 = 1;
/// `oneway=-1` — traversable against way order only.
pub const ONEWAY_BACKWARD: u8 = 2;

/// Access mask from a way's tags. Denial tags (`access=no|private`,
/// `motor_vehicle=no`, `bicycle=no`, `foot=no`) clear their bit; nothing
/// present leaves [`ACCESS_ALL`]. A blanket `access=no|private` is applied
/// FIRST, so a specific `bicycle=yes` after it still re-opens that one mode
/// — matching OSM's own tag-specificity convention (`access=private;
/// bicycle=yes` means "closed except to bikes"), not a blanket override.
#[must_use]
pub fn access_from_tags<'a>(tags: impl IntoIterator<Item = (&'a str, &'a str)>) -> u8 {
    let mut mask = ACCESS_ALL;
    let mut blanket_deny = false;
    let tags: Vec<(&str, &str)> = tags.into_iter().collect();
    for &(k, v) in &tags {
        if k == "access" && matches!(v, "no" | "private") {
            blanket_deny = true;
        }
    }
    if blanket_deny {
        mask = 0;
    }
    for (k, v) in tags {
        let allow = matches!(v, "yes" | "designated" | "permissive" | "destination");
        let deny = v == "no";
        match k {
            "motor_vehicle" | "motorcar" if allow => mask |= ACCESS_CAR,
            "motor_vehicle" | "motorcar" if deny => mask &= !ACCESS_CAR,
            "bicycle" if allow => mask |= ACCESS_BIKE,
            "bicycle" if deny => mask &= !ACCESS_BIKE,
            "foot" if allow => mask |= ACCESS_FOOT,
            "foot" if deny => mask &= !ACCESS_FOOT,
            _ => {}
        }
    }
    mask
}

/// Oneway class from a way's `oneway` tag.
#[must_use]
pub fn oneway_from_tags<'a>(tags: impl IntoIterator<Item = (&'a str, &'a str)>) -> u8 {
    for (k, v) in tags {
        if k != "oneway" {
            continue;
        }
        return match v {
            "yes" | "1" | "true" => ONEWAY_FORWARD,
            "-1" => ONEWAY_BACKWARD,
            _ => ONEWAY_NONE, // "no", "0", "false", or an unrecognised value
        };
    }
    ONEWAY_NONE
}

/// `oneway:bicycle=no` — the common German "Radfahrer frei" exception: a
/// street closed to general contraflow traffic that still permits cyclists
/// against the flow. Without this bit a bike route silently inherits the car
/// restriction, which is a real wrong answer, not a missing feature — a
/// router that cannot tell "closed" from "closed except to bikes" will route
/// a cyclist the long way around a street they were allowed to use directly.
#[must_use]
pub fn bicycle_contraflow_from_tags<'a>(
    tags: impl IntoIterator<Item = (&'a str, &'a str)>,
) -> bool {
    tags.into_iter()
        .any(|(k, v)| k == "oneway:bicycle" && v == "no")
}

/// Pack one edge's access byte.
#[must_use]
pub fn pack_access(access: u8, oneway: u8, contraflow: bool) -> u8 {
    (access & 0b111) | ((oneway & 0b11) << 3) | (u8::from(contraflow) << 5)
}

/// Unpack an access byte back into `(access, oneway, contraflow)`.
#[must_use]
pub fn unpack_access(b: u8) -> (u8, u8, bool) {
    (b & 0b111, (b >> 3) & 0b11, (b >> 5) & 1 == 1)
}

/// One way's whole access byte, from its tags — the composition the bake
/// writes into [`crate::street::EDGE_GEOMETRY_SLOT`]'s access half.
///
/// Exists so the producer has ONE call and the three readings cannot be
/// combined differently in different places. The tag iterator is walked three
/// times rather than once on purpose: each `*_from_tags` owns its own
/// precedence rule (access has OSM's specificity convention, oneway does not),
/// and fusing them into a single pass would mean re-implementing those rules
/// here — the duplication this function exists to prevent.
///
/// A way with no access tags at all yields the permissive default, which is
/// correct for OSM: absence means "not restricted", never "unknown".
#[must_use]
pub fn way_access_byte<'a, I>(tags: I) -> u8
where
    I: IntoIterator<Item = (&'a str, &'a str)> + Clone,
{
    pack_access(
        access_from_tags(tags.clone()),
        oneway_from_tags(tags.clone()),
        bicycle_contraflow_from_tags(tags),
    )
}

/// An 8×8 forbidden-transition bitmask for one junction: bit `from*8+to` set
/// means "entering on edge slot `from`, turning onto edge slot `to`, is
/// forbidden" — the P12 collapse for a restriction incident at its own via
/// junction. `u64` is exactly 8×8 bits; no allocation, one word per junction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RestrictionMask(pub u64);

impl RestrictionMask {
    /// Empty mask: no turn forbidden.
    pub const EMPTY: Self = Self(0);

    /// Forbid the `from -> to` transition. Slots >= 8 are silently dropped —
    /// `EDGE_SLOTS` bounds every junction to 8 edges by construction, so an
    /// out-of-range slot here is a caller bug, not routing data to reject
    /// loudly (the caller controls both indices).
    #[must_use]
    pub fn with_forbidden(self, from: u8, to: u8) -> Self {
        if from >= 8 || to >= 8 {
            return self;
        }
        Self(self.0 | (1u64 << (u32::from(from) * 8 + u32::from(to))))
    }

    /// Is `from -> to` forbidden at this junction?
    #[must_use]
    pub fn is_forbidden(self, from: u8, to: u8) -> bool {
        if from >= 8 || to >= 8 {
            return false;
        }
        (self.0 >> (u32::from(from) * 8 + u32::from(to))) & 1 == 1
    }

    #[must_use]
    pub fn to_bytes(self) -> [u8; 8] {
        self.0.to_le_bytes()
    }

    #[must_use]
    pub fn from_bytes(b: [u8; 8]) -> Self {
        Self(u64::from_le_bytes(b))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn untagged_way_is_fully_open() {
        assert_eq!(access_from_tags([("highway", "residential")]), ACCESS_ALL);
    }

    #[test]
    fn blanket_deny_then_specific_reopen_matches_osm_specificity_convention() {
        // access=private;bicycle=yes means "closed except to bikes" — the
        // MORE SPECIFIC tag wins over the blanket one, tag order in the way
        // notwithstanding (OSM tags are unordered key=value pairs).
        let mask = access_from_tags([("access", "private"), ("bicycle", "yes")]);
        assert_eq!(
            mask, ACCESS_BIKE,
            "bicycle=yes must re-open after access=private"
        );
    }

    #[test]
    fn blanket_deny_alone_closes_everything() {
        assert_eq!(access_from_tags([("access", "no")]), 0);
    }

    #[test]
    fn bicycle_no_closes_only_bicycle() {
        let mask = access_from_tags([("bicycle", "no")]);
        assert_eq!(mask, ACCESS_CAR | ACCESS_FOOT);
    }

    #[test]
    fn oneway_reads_the_three_real_encodings() {
        assert_eq!(oneway_from_tags([("oneway", "yes")]), ONEWAY_FORWARD);
        assert_eq!(oneway_from_tags([("oneway", "1")]), ONEWAY_FORWARD);
        assert_eq!(oneway_from_tags([("oneway", "-1")]), ONEWAY_BACKWARD);
        assert_eq!(oneway_from_tags([("oneway", "no")]), ONEWAY_NONE);
        assert_eq!(oneway_from_tags(std::iter::empty()), ONEWAY_NONE);
    }

    #[test]
    fn contraflow_needs_the_exact_tag_not_a_guess() {
        assert!(bicycle_contraflow_from_tags([("oneway:bicycle", "no")]));
        assert!(!bicycle_contraflow_from_tags([("oneway:bicycle", "yes")]));
        assert!(!bicycle_contraflow_from_tags([("oneway", "yes")]));
    }

    #[test]
    fn pack_unpack_round_trips_every_combination() {
        for access in 0..8u8 {
            for oneway in 0..4u8 {
                for contraflow in [false, true] {
                    let b = pack_access(access, oneway, contraflow);
                    assert_eq!(unpack_access(b), (access, oneway & 0b11, contraflow));
                }
            }
        }
    }

    #[test]
    fn restriction_mask_forbids_exactly_the_pair_set_not_its_neighbours() {
        let m = RestrictionMask::EMPTY.with_forbidden(2, 5);
        assert!(m.is_forbidden(2, 5));
        // Two-sided: a mask that forbids everything would pass the line
        // above for the wrong reason. These must all read as ALLOWED.
        assert!(!m.is_forbidden(5, 2), "reverse direction must not alias");
        assert!(!m.is_forbidden(2, 2), "self-loop must not alias");
        assert!(!m.is_forbidden(2, 6), "adjacent slot must not alias");
        assert!(!m.is_forbidden(3, 5), "adjacent from-slot must not alias");
    }

    #[test]
    fn restriction_mask_out_of_range_slots_are_inert_not_a_panic() {
        let m = RestrictionMask::EMPTY.with_forbidden(9, 3);
        assert_eq!(
            m,
            RestrictionMask::EMPTY,
            "an invalid write must not corrupt the mask"
        );
        assert!(!m.is_forbidden(9, 3));
    }

    #[test]
    fn restriction_mask_byte_round_trips() {
        let m = RestrictionMask::EMPTY
            .with_forbidden(0, 7)
            .with_forbidden(3, 1)
            .with_forbidden(7, 7);
        assert_eq!(RestrictionMask::from_bytes(m.to_bytes()), m);
    }

    #[test]
    fn restriction_mask_can_hold_every_pair_at_once_the_real_worst_case() {
        // A degree-8 junction (the one Brandenburg row this session already
        // found sitting at EDGE_SLOTS capacity) could in principle forbid
        // every transition. 64 bits must hold all 64 without collision.
        let mut m = RestrictionMask::EMPTY;
        for from in 0..8u8 {
            for to in 0..8u8 {
                m = m.with_forbidden(from, to);
            }
        }
        assert_eq!(m.0, u64::MAX);
        for from in 0..8u8 {
            for to in 0..8u8 {
                assert!(m.is_forbidden(from, to));
            }
        }
    }

    /// The producer's one call must carry all three readings — a composition
    /// that silently dropped one would still return a plausible byte.
    ///
    /// Each case therefore differs from the permissive default in a DIFFERENT
    /// field, so dropping any single `*_from_tags` call fails at least one.
    #[test]
    fn the_way_access_byte_carries_all_three_readings() {
        // Untagged: permissive, and NOT zero — absence means "not restricted"
        // in OSM, so a zero byte here would forbid everything by accident.
        let (a, o, c) = unpack_access(way_access_byte([("highway", "residential")]));
        assert_eq!(a, ACCESS_ALL, "an untagged way must not be closed");
        assert_eq!(o, ONEWAY_NONE);
        assert!(!c);

        // Oneway only — access and contraflow must stay at their defaults.
        let (a, o, c) = unpack_access(way_access_byte([
            ("highway", "residential"),
            ("oneway", "yes"),
        ]));
        assert_eq!(a, ACCESS_ALL);
        assert_eq!(o, ONEWAY_FORWARD, "the oneway reading was dropped");
        assert!(!c);

        // Access only — the OSM specificity convention: closed EXCEPT bikes.
        let (a, o, c) = unpack_access(way_access_byte([("access", "private"), ("bicycle", "yes")]));
        assert_eq!(a, ACCESS_BIKE, "the access reading was dropped");
        assert_eq!(o, ONEWAY_NONE);
        assert!(!c);

        // Contraflow only — the field a router needs to avoid sending a
        // cyclist the long way round a street they may legally use.
        let (a, o, c) = unpack_access(way_access_byte([
            ("highway", "residential"),
            ("oneway", "yes"),
            ("oneway:bicycle", "no"),
        ]));
        assert_eq!(a, ACCESS_ALL);
        assert_eq!(o, ONEWAY_FORWARD);
        assert!(c, "the contraflow reading was dropped");

        // All three at once, each non-default, so the packing cannot be
        // satisfied by any two of them.
        let byte = way_access_byte([
            ("access", "private"),
            ("bicycle", "yes"),
            ("oneway", "-1"),
            ("oneway:bicycle", "no"),
        ]);
        assert_eq!(unpack_access(byte), (ACCESS_BIKE, ONEWAY_BACKWARD, true));
    }
}
