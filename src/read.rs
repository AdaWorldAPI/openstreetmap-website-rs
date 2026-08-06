//! `.osm.pbf` → anchored, tagged features.
//!
//! Two passes over the extract: pass 1 indexes every node's `(lon, lat)`;
//! pass 2 emits one [`Feature`] per **tagged** element, anchored at a point.
//!
//! # What gets a row, and what does not
//!
//! Only tagged elements. Berlin's extract carries 7,870,234 nodes of which
//! 1,190,010 are tagged — the other 85% are pure *geometry* (way node-refs,
//! 10.4 M of them at 7.8 per way). Giving each a 512-byte ABI row would cost
//! 4.7 GB to store what is, per the CANON node's own doctrine 2, bulk that is
//! **addressed, not inlined**. Tagged features are 2,527,304 rows ≈ 1.2 GiB.
//!
//! # Anchoring
//!
//! A node anchors at itself (exact). A way anchors at its node centroid.
//!
//! **A relation anchors at its `via` member when it has one.** For a turn
//! restriction that is not an approximation — the `via` node *is* the junction
//! the restriction applies at, so the anchor is exact. Measured on the Berlin
//! extract: of 1,490 `type=restriction*` relations, 1,392 carry a `via` **node**
//! (93.4%), 79 a `via` **way** (5.3%), and 19 carry no `via` at all — those fall
//! back to the member centroid and are counted, never silently dropped.
//!
//! Relations without a `via` (multipolygons, routes, …) anchor at the centroid
//! of whatever members resolve.
//!
//! Restrictions are why this matters more than the 0.7% share suggests. For a
//! feature bake, dropping 18,128 relations is a rounding error. For a router it
//! is a **correctness hole**: 1,490 absent turn restrictions produce routes that
//! turn where turning is forbidden, and "turn left where you may not" is exactly
//! the failure a Fahrtenbuch cannot have.

use std::collections::HashMap;

use osmpbf::{Element, ElementReader};

/// OGAR concept ids for the OSM element kinds (`ogar_codebook`, geo domain
/// `0x0F`). These land in the `EntityType` value tenant — the element's kind is
/// a *class*, never a slot in the key (le-contract §2 slot purity).
pub const OSM_NODE: u16 = 0x0F01;
pub const OSM_WAY: u16 = 0x0F02;
pub const OSM_RELATION: u16 = 0x0F03;

/// A turn restriction is an `osm_relation` like any other — the OGAR codebook
/// (geo domain `0x0F`) has no `osm_turn_restriction` concept, and inventing an
/// ordinal that no codebook mints would be exactly the fabricated-data defect
/// the workspace rules forbid. Distinguishing a restriction from a multipolygon
/// is the ClassView's job (le-contract §2: the kind is a class, never a slot).
/// The bake reports the count so the population is visible.
fn is_restriction(t: Option<&str>) -> bool {
    t.is_some_and(|t| t.starts_with("restriction"))
}

/// One tagged OSM feature, reduced to what a row needs: an anchor point and a
/// kind. Tag *content* is deliberately absent — it belongs to a ClassView /
/// tag-identity lane, not to this struct (Q1 Tag-as-Class).
#[derive(Debug, Clone, Copy)]
pub struct Feature {
    pub lon: f64,
    pub lat: f64,
    pub entity_type: u16,
    /// The source OSM id, kept only so a bake can be audited back to the
    /// extract. It is NOT stored in the row (raw ids need 34 bits; the
    /// identity-quad codebook is the sanctioned home — lance-graph PR #902).
    pub osm_id: i64,
}

/// Counts reported by a read, so a bake can state what it covered.
#[derive(Debug, Default, Clone, Copy)]
pub struct ReadStats {
    pub nodes_indexed: u64,
    pub tagged_nodes: u64,
    pub tagged_ways: u64,
    /// Relations that produced a row.
    pub relations_anchored: u64,
    /// Relations whose members resolved to no geometry at all — counted, never
    /// silently dropped.
    pub relations_unanchorable: u64,
    /// `type=restriction*` relations anchored (subset of `relations_anchored`).
    pub restrictions: u64,
    /// Restrictions anchored exactly at a `via` **node** — the junction itself.
    pub restrictions_via_node: u64,
    /// Restrictions anchored at a `via` **way** centroid.
    pub restrictions_via_way: u64,
    /// Restrictions carrying no `via` member; fell back to the member centroid.
    pub restrictions_no_via: u64,
    pub ways_unresolved: u64,
}

/// One relation member, reduced to what anchoring needs.
struct Member {
    is_node: bool,
    id: i64,
    via: bool,
}

/// A relation held until every way centroid is known. Collecting the member
/// lists and resolving afterwards means the bake makes **no assumption about
/// element order in the file** — a relation appearing before its member ways
/// resolves identically.
struct RawRelation {
    restriction: bool,
    members: Vec<Member>,
}

/// An OSM element id → its resolved position. Nodes map to their own
/// coordinate; ways to their centroid.
type PosMap = HashMap<i64, (f64, f64)>;

/// Which branch of the anchor rule fired — reported so the bake shows the
/// population rather than asserting it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AnchorVia {
    /// Anchored at a `via` **node**: the junction itself, exact.
    Node,
    /// Anchored at a `via` **way**'s centroid.
    Way,
    /// No `via` member resolved; fell back to the member centroid.
    Absent,
}

/// A member's position: a node's own coordinate, or a way's centroid.
fn member_pos(m: &Member, coords: &PosMap, ways: &PosMap) -> Option<(f64, f64)> {
    if m.is_node {
        coords.get(&m.id).copied()
    } else {
        ways.get(&m.id).copied()
    }
}

/// Anchor one relation.
///
/// **The `via` member wins when it resolves.** For a turn restriction the `via`
/// node is the junction the restriction applies at, so the anchor is exact
/// rather than a centroid sitting somewhere along the approach roads.
/// Everything else — and a restriction whose `via` is missing or clipped out of
/// the extract — falls back to the centroid of whatever members resolve.
///
/// Returns `None` only when nothing resolves at all: a super-relation whose
/// members are themselves relations (route_master and friends carry no geometry
/// of their own), or a member set entirely outside the extract. The caller
/// counts those; they are never silently dropped.
fn anchor_relation(
    rel: &RawRelation,
    coords: &PosMap,
    ways: &PosMap,
) -> (Option<(f64, f64)>, AnchorVia) {
    if let Some(via) = rel.members.iter().find(|m| m.via) {
        if let Some(p) = member_pos(via, coords, ways) {
            let how = if via.is_node {
                AnchorVia::Node
            } else {
                AnchorVia::Way
            };
            return (Some(p), how);
        }
    }
    let (mut slon, mut slat, mut cnt) = (0.0f64, 0.0f64, 0u32);
    for m in &rel.members {
        if let Some((lon, lat)) = member_pos(m, coords, ways) {
            slon += lon;
            slat += lat;
            cnt += 1;
        }
    }
    let c = (cnt > 0).then(|| (slon / f64::from(cnt), slat / f64::from(cnt)));
    (c, AnchorVia::Absent)
}

/// Read every tagged feature from `path`, anchored.
pub fn read_features(path: &str) -> Result<(Vec<Feature>, ReadStats), Box<dyn std::error::Error>> {
    let mut stats = ReadStats::default();

    // ── Pass 1: node id → (lon, lat). ──
    let mut coords: PosMap = HashMap::with_capacity(8_000_000);
    ElementReader::from_path(path)?.for_each(|el| match el {
        Element::Node(n) => {
            coords.insert(n.id(), (n.lon(), n.lat()));
        }
        Element::DenseNode(n) => {
            coords.insert(n.id(), (n.lon(), n.lat()));
        }
        _ => {}
    })?;
    stats.nodes_indexed = coords.len() as u64;
    eprintln!("pass 1: {} nodes indexed", stats.nodes_indexed);

    // ── Pass 2: tagged elements → anchored features. ──
    let mut out: Vec<Feature> = Vec::with_capacity(2_600_000);
    let mut ways_unresolved = 0u64;
    let mut way_centroid: PosMap = HashMap::with_capacity(1_400_000);
    let mut pending: Vec<(i64, RawRelation)> = Vec::new();
    ElementReader::from_path(path)?.for_each(|el| match el {
        Element::Node(n) => {
            if n.tags().next().is_some() {
                out.push(Feature {
                    lon: n.lon(),
                    lat: n.lat(),
                    entity_type: OSM_NODE,
                    osm_id: n.id(),
                });
            }
        }
        Element::DenseNode(n) => {
            if n.tags().next().is_some() {
                out.push(Feature {
                    lon: n.lon(),
                    lat: n.lat(),
                    entity_type: OSM_NODE,
                    osm_id: n.id(),
                });
            }
        }
        Element::Way(w) => {
            // Centroid is computed for EVERY way, tagged or not: a relation may
            // reference an untagged way (a multipolygon inner ring, a via way),
            // and that member still has to resolve.
            let (mut slon, mut slat, mut cnt) = (0.0f64, 0.0f64, 0u32);
            for id in w.refs() {
                if let Some(&(lon, lat)) = coords.get(&id) {
                    slon += lon;
                    slat += lat;
                    cnt += 1;
                }
            }
            let tagged = w.tags().next().is_some();
            if cnt == 0 {
                if tagged {
                    ways_unresolved += 1;
                }
                return;
            }
            let c = (slon / f64::from(cnt), slat / f64::from(cnt));
            way_centroid.insert(w.id(), c);
            if tagged {
                out.push(Feature {
                    lon: c.0,
                    lat: c.1,
                    entity_type: OSM_WAY,
                    osm_id: w.id(),
                });
            }
        }
        Element::Relation(r) => {
            if r.tags().next().is_none() {
                return;
            }
            let restriction = is_restriction(r.tags().find(|(k, _)| *k == "type").map(|(_, v)| v));
            let members = r
                .members()
                .filter_map(|m| match m.member_type {
                    osmpbf::RelMemberType::Node => Some(Member {
                        is_node: true,
                        id: m.member_id,
                        via: m.role().is_ok_and(|s| s == "via"),
                    }),
                    osmpbf::RelMemberType::Way => Some(Member {
                        is_node: false,
                        id: m.member_id,
                        via: m.role().is_ok_and(|s| s == "via"),
                    }),
                    // A relation-of-relations member carries no geometry of its
                    // own here; it is not an anchor candidate.
                    osmpbf::RelMemberType::Relation => None,
                })
                .collect();
            pending.push((
                r.id(),
                RawRelation {
                    restriction,
                    members,
                },
            ));
        }
    })?;

    // Recount by kind from what was actually emitted (never from an assumption).
    for f in &out {
        match f.entity_type {
            OSM_NODE => stats.tagged_nodes += 1,
            OSM_WAY => stats.tagged_ways += 1,
            _ => {}
        }
    }
    stats.ways_unresolved = ways_unresolved;

    // ── Anchor the relations, now that every way centroid is known. ──
    for (rid, rel) in &pending {
        let (anchor, how) = anchor_relation(rel, &coords, &way_centroid);
        if rel.restriction {
            match how {
                AnchorVia::Node => stats.restrictions_via_node += 1,
                AnchorVia::Way => stats.restrictions_via_way += 1,
                AnchorVia::Absent => stats.restrictions_no_via += 1,
            }
        }
        match anchor {
            Some((lon, lat)) => {
                stats.relations_anchored += 1;
                if rel.restriction {
                    stats.restrictions += 1;
                }
                out.push(Feature {
                    lon,
                    lat,
                    entity_type: OSM_RELATION,
                    osm_id: *rid,
                });
            }
            None => stats.relations_unanchorable += 1,
        }
    }
    eprintln!(
        "pass 2: {} tagged nodes + {} tagged ways + {} relations = {} features \
         ({} restrictions: {} via-node, {} via-way, {} no-via; {} unanchorable)",
        stats.tagged_nodes,
        stats.tagged_ways,
        stats.relations_anchored,
        out.len(),
        stats.restrictions,
        stats.restrictions_via_node,
        stats.restrictions_via_way,
        stats.restrictions_no_via,
        stats.relations_unanchorable,
    );
    Ok((out, stats))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn maps() -> (PosMap, PosMap) {
        // A junction at (13.40, 52.50) with two approach roads.
        //
        // The way centroids are deliberately ASYMMETRIC about the junction. A
        // first draft used (13.30, 52.40) and (13.50, 52.60), which are
        // symmetric — so the from/via/to centroid landed exactly on the via
        // node and the discriminating assertion in
        // `via_node_anchors_at_the_junction_not_the_member_centroid` could not
        // fail no matter what the code did. The `assert_ne!` caught it. Keep
        // these asymmetric or that test goes vacuous.
        let coords = HashMap::from([(1i64, (13.40, 52.50)), (2i64, (13.60, 52.70))]);
        let ways = HashMap::from([(10i64, (13.32, 52.44)), (20i64, (13.50, 52.60))]);
        (coords, ways)
    }

    fn m(is_node: bool, id: i64, via: bool) -> Member {
        Member { is_node, id, via }
    }

    #[test]
    fn via_node_anchors_at_the_junction_not_the_member_centroid() {
        // The load-bearing case: 93.4% of Berlin's restrictions take this path.
        let (c, w) = maps();
        let rel = RawRelation {
            restriction: true,
            members: vec![m(false, 10, false), m(true, 1, true), m(false, 20, false)],
        };
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(how, AnchorVia::Node);
        assert_eq!(anchor, Some((13.40, 52.50)), "must be the via node exactly");
        // Two-sided: it must NOT be the centroid of from/via/to, which is where
        // a naive implementation would put it.
        let centroid = ((13.32 + 13.40 + 13.50) / 3.0, (52.44 + 52.50 + 52.60) / 3.0);
        assert_ne!(
            centroid.0, 13.40,
            "fixture must not be symmetric about the via"
        );
        assert_ne!(
            anchor,
            Some(centroid),
            "a centroid anchor would be wrong here"
        );
    }

    #[test]
    fn via_way_anchors_at_that_ways_centroid() {
        let (c, w) = maps();
        let rel = RawRelation {
            restriction: true,
            members: vec![m(false, 10, false), m(false, 20, true)],
        };
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(how, AnchorVia::Way);
        assert_eq!(anchor, Some((13.50, 52.60)));
    }

    #[test]
    fn no_via_falls_back_to_a_genuine_member_centroid() {
        let (c, w) = maps();
        let rel = RawRelation {
            restriction: false,
            members: vec![m(true, 1, false), m(true, 2, false)],
        };
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(how, AnchorVia::Absent);
        // A real average of two distinct members, not just the first one.
        assert_eq!(anchor, Some((13.50, 52.60)));
        assert_ne!(
            anchor,
            Some((13.40, 52.50)),
            "must not collapse to member 0"
        );
    }

    #[test]
    fn an_unresolvable_via_falls_back_rather_than_failing() {
        // A via clipped out of the extract must not sink the whole relation.
        let (c, w) = maps();
        let rel = RawRelation {
            restriction: true,
            members: vec![m(true, 999, true), m(true, 1, false), m(true, 2, false)],
        };
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(
            how,
            AnchorVia::Absent,
            "an absent via is reported as absent"
        );
        assert_eq!(
            anchor,
            Some((13.50, 52.60)),
            "still anchored, via the centroid"
        );
    }

    #[test]
    fn a_super_relation_is_unanchorable_and_that_is_discriminating() {
        let (c, w) = maps();
        // route_master and friends: every member is another relation, filtered
        // out before this point, so the member list is empty. 689 of Berlin's
        // 18,128 tagged relations are this shape.
        let empty = RawRelation {
            restriction: false,
            members: vec![],
        };
        assert_eq!(anchor_relation(&empty, &c, &w).0, None);
        // ...and members that exist but resolve to nothing.
        let ghosts = RawRelation {
            restriction: false,
            members: vec![m(true, 777, false), m(false, 888, false)],
        };
        assert_eq!(anchor_relation(&ghosts, &c, &w).0, None);
        // Can-stay-silent paired with can-fire: an ordinary relation DOES
        // anchor, so `None` means something.
        let ok = RawRelation {
            restriction: false,
            members: vec![m(true, 1, false)],
        };
        assert!(anchor_relation(&ok, &c, &w).0.is_some());
    }

    #[test]
    fn only_the_first_via_is_consulted_and_ordering_does_not_matter() {
        // The via is found by role, not by position, so a relation whose via is
        // last resolves identically to one whose via is first. This is what
        // makes collecting members and resolving afterwards safe regardless of
        // element order in the file.
        let (c, w) = maps();
        let first = RawRelation {
            restriction: true,
            members: vec![m(true, 1, true), m(false, 10, false)],
        };
        let last = RawRelation {
            restriction: true,
            members: vec![m(false, 10, false), m(true, 1, true)],
        };
        assert_eq!(
            anchor_relation(&first, &c, &w),
            anchor_relation(&last, &c, &w)
        );
        assert_eq!(anchor_relation(&last, &c, &w).1, AnchorVia::Node);
    }

    #[test]
    fn restriction_type_detection_discriminates() {
        assert!(is_restriction(Some("restriction")));
        assert!(is_restriction(Some("restriction:hgv")));
        // ...and does not fire on everything, which is the half that matters:
        // multipolygon is the single most common relation type in the extract.
        assert!(!is_restriction(Some("multipolygon")));
        assert!(!is_restriction(Some("route_master")));
        assert!(!is_restriction(None));
    }
}
