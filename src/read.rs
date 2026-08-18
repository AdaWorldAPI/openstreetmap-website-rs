//! `.osm.pbf` → anchored, tagged features.
//!
//! Two passes over the extract: pass 1 indexes every node's `(lon, lat)`;
//! pass 2 emits one [`Feature`] per **tagged** element, anchored at a point.
//!
//! # What gets a row, and what does not
//!
//! Only tagged elements. Berlin's extract carries 7,852,192 nodes of which
//! 1,185,287 are tagged — the other 85% are pure *geometry* (way node-refs).
//! Giving each a 512-byte ABI row would cost 4.7 GB to store what is, per the
//! CANON node's own doctrine 2, bulk that is **addressed, not inlined**.
//! Tagged features are 2,518,465 (1,185,287 nodes + 1,315,767 ways + 17,411
//! relations), carrying 11,679,125 tag pairs. Re-measured 2026-08-06; the
//! earlier figures in this doc were from a different extract vintage.
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

use ogar_vocab::class_ids;
use osmpbf::{Element, ElementReader};

use crate::tags::{TagSpan, TagStore};
use crate::tms::{mean_cell, point_to_cell, TileXy};

/// OGAR concept ids for the OSM element kinds — **pulled** from
/// [`ogar_vocab::class_ids`], never restated.
///
/// These were local literals (`0x0F01` …) until the plug-and-play rule was
/// applied: a consumer pulls the codebook, it does not copy it. A copy is not
/// merely redundant — it is a second source of truth that drifts silently,
/// because nothing fails when upstream changes and the local literal does not.
///
/// The kind reaches a row as the **identity facet's classid**
/// (`crate::identity`), not as a key tier — le-contract §2 slot purity: the
/// element's kind is a *class*, never a slot in the key. (It previously landed
/// in the `EntityType` value tenant; that lane was retired when the geo
/// ClassView adopted the facet-slot reading of the value slab.)
///
/// [`OsmPort`](ogar_vocab::ports::OsmPort) is the port surface over the same
/// ids; [`tests::the_port_and_these_constants_agree`] pins them together, so
/// the port is the checked authority and these are a checked mirror rather
/// than an independent declaration.
pub const OSM_NODE: u16 = class_ids::OSM_NODE;
pub const OSM_WAY: u16 = class_ids::OSM_WAY;
pub const OSM_RELATION: u16 = class_ids::OSM_RELATION;

/// The one **derived** kind: a junction in the routable-way graph. OSM has no
/// junction element, so unlike the three above this mirrors no Rails class —
/// `ogar_osm::OSM_CONCEPTS` carries it with a deliberately empty rails list.
///
/// It exists as its own concept for a reason this bake measured the hard way.
/// A junction row and an ordinary tagged node are the same 512 bytes with
/// different payloads, and before this concept there was **no byte that told
/// them apart**: `crate::street`'s module docs record a reader that tried the
/// edge-name lane at value-local slot 2 and hit **1,084,213 false positives**
/// on real Berlin data — one per tagged node, reinterpreting each node's first
/// tag as eight raw name ordinals. The lane survives today only by hiding at
/// slot 1, the last gap the tag path never writes, which does not scale: the
/// route planner's bearing, turn-matrix and access lanes need several more
/// slots and there is no second gap.
///
/// Stamping this as the identity facet's classid makes the layout
/// **class-resolved** — a reader asks `read_identity` what the row IS instead
/// of guessing from which slots look occupied.
pub const OSM_STREET_NODE: u16 = class_ids::OSM_STREET_NODE;

/// A turn restriction is an `osm_relation` like any other — the OGAR codebook
/// (geo domain `0x0F`) has no `osm_turn_restriction` concept, and inventing an
/// ordinal that no codebook mints would be exactly the fabricated-data defect
/// the workspace rules forbid. Distinguishing a restriction from a multipolygon
/// is the ClassView's job (le-contract §2: the kind is a class, never a slot).
/// The bake reports the count so the population is visible.
fn is_restriction(t: Option<&str>) -> bool {
    t.is_some_and(|t| t.starts_with("restriction"))
}

/// Where a feature sits, and under WHICH contract.
///
/// The two are genuinely different claims and the type says so, rather than
/// leaving a reader to infer it from the element kind:
///
/// - [`Anchor::Published`] — the coordinate OSM itself stores, on its 1e-7
///   degree grid. Parity means reproducing that integer, exactly.
/// - [`Anchor::Derived`] — an anchor this crate computed, already a z=32 grid
///   cell. It is a grid point **by construction** (integer mean of member
///   cells, `tms::mean_cell`), so parity means reproducing that cell, exactly —
///   and the derivation contains no float.
///
/// The first version had only `lon: f64, lat: f64` for both, which forced the
/// derived case to be checked against OSM's grid — the wrong grid for a value
/// that was never an OSM coordinate — and could only ever hold "within one
/// step" (435,426 of 1,333,178 differed). Splitting the type is what lets both
/// checks be `assert_eq!`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Anchor {
    /// A node's own published coordinate.
    Published { lon: f64, lat: f64 },
    /// A way or relation anchor, computed in the key's own grid.
    Derived(TileXy),
}

impl Anchor {
    /// The cell this anchor keys to. Integer for [`Anchor::Derived`]; the one
    /// ingest float for [`Anchor::Published`].
    #[must_use]
    pub fn cell(self) -> TileXy {
        match self {
            Anchor::Published { lon, lat } => point_to_cell(lon, lat),
            Anchor::Derived(c) => c,
        }
    }
}

/// One tagged OSM feature, reduced to what a row needs: an anchor point, a
/// kind, and its tags.
///
/// The tags are a [`TagSpan`] into the read's [`TagStore`], not owned text —
/// the struct stays `Copy` (the read loop relies on it) and the extract's 8.5 M
/// tag pairs stay one flat allocation instead of 2.5 M small ones. See
/// [`crate::tags`].
///
/// An earlier version of this struct carried no tags at all, on the reasoning
/// that tag *content* belongs to a ClassView rather than to the reader. That
/// was right about where tags are *read* and wrong about whether they are
/// *carried*: dropping them at the reader put them out of reach of the bake
/// entirely, so "identical data to OSM" could not be true no matter what the
/// row layout did. The ClassView still owns the reading.
#[derive(Debug, Clone, Copy)]
pub struct Feature {
    pub anchor: Anchor,
    pub entity_type: u16,
    /// The source OSM id, kept only so a bake can be audited back to the
    /// extract. It is NOT stored in the row (raw ids need 34 bits; the
    /// identity-quad codebook is the sanctioned home — lance-graph PR #902).
    pub osm_id: i64,
    /// This feature's tags, as a span into the read's [`TagStore`].
    pub tags: TagSpan,
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
    /// Tag pairs interned across every emitted feature.
    pub tag_pairs: u64,
    /// Distinct tag keys — the size of the key codebook.
    pub distinct_tag_keys: u64,
    /// Distinct tag values — the size of the value codebook, and the one that
    /// has to clear the `u24` bound (see [`crate::tags`]).
    pub distinct_tag_values: u64,
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
    /// Interned during the read pass, because the relation's own tag text is
    /// only borrowable while the element is in scope — anchoring happens later.
    tags: TagSpan,
}

/// An OSM element id → its resolved **cell**. Nodes map to their own cell;
/// ways to their integer centroid.
///
/// Cells, not coordinates: every downstream use is an anchor, and keeping the
/// f64 here is what made the derived anchor float-dependent in the first place.
type PosMap = HashMap<i64, TileXy>;

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
fn member_pos(m: &Member, coords: &PosMap, ways: &PosMap) -> Option<TileXy> {
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
) -> (Option<TileXy>, AnchorVia) {
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
    let cells: Vec<TileXy> = rel
        .members
        .iter()
        .filter_map(|m| member_pos(m, coords, ways))
        .collect();
    (mean_cell(&cells), AnchorVia::Absent)
}

/// Read every tagged feature from `path`, anchored, with its tags interned.
///
/// The [`TagStore`] is returned alongside the features because a [`TagSpan`] is
/// meaningless without it — handing back only the features would be handing
/// back offsets into a dropped allocation.
pub fn read_features(
    path: &str,
) -> Result<(Vec<Feature>, TagStore, ReadStats), Box<dyn std::error::Error>> {
    let (features, store, stats, _chains, _junctions) = read_features_with_chains(path)?;
    Ok((features, store, stats))
}

/// What [`read_features_with_chains`] yields: features, the tag store, read
/// stats, and each tagged way's vertex chain keyed by OSM way id.
pub type ReadWithChains = (
    Vec<Feature>,
    TagStore,
    ReadStats,
    HashMap<i64, Vec<TileXy>>,
    Vec<Junction>,
);

/// `highway=*` values that carry no traffic of any mode — the same list
/// `junction_probe`/`graph_probe` use, so the junction set this pipeline
/// actually bakes matches the set those probes measured against. Kept as its
/// own copy rather than a shared import: the probes are investigation tools
/// that may drift independently, and the bake's copy is the one that must
/// stay correct.
const NON_ROUTABLE: &[&str] = &[
    "construction",
    "proposed",
    "platform",
    "raceway",
    "bus_stop",
    "street_lamp",
    "traffic_sign",
    "rest_area",
    "services",
];

fn is_routable<'a>(mut tags: impl Iterator<Item = (&'a str, &'a str)>) -> bool {
    match tags.find(|(k, _)| *k == "highway") {
        Some((_, v)) => !NON_ROUTABLE.contains(&v),
        None => false,
    }
}

/// A node shared by two or more routable ways — the population
/// `junction_probe` (P12) measures as `routable_ref >= 2`, computed the same
/// way here: **occurrences**, not distinct ways. A way that touches the same
/// node twice (a loop back through it) contributes two occurrences, and both
/// become their own edge slot — the node really does carry two of that way's
/// segment-ends there.
#[derive(Debug, Clone)]
pub struct Junction {
    pub node_id: i64,
    pub cell: TileXy,
    /// Way ids touching this junction, one entry per occurrence, in the
    /// deterministic (way id, then position within that way) order
    /// [`build_junctions`] walks them — never insertion order from the PBF
    /// reader, which is not guaranteed stable across reads.
    pub edge_ways: Vec<i64>,
    /// `edge_ways[i]`'s position within THAT way's own node-ref list — the
    /// piece a bearing needs and the reason it was not computed before this
    /// field existed. Parallel to `edge_ways`, same index, same length. A way
    /// touching this junction twice (the loop-back case in this struct's own
    /// doc) gets two DIFFERENT indices here, one per occurrence — without
    /// this, both occurrences would resolve to the same neighbor and report
    /// an identical heading, silently wrong for the second one.
    pub edge_way_index: Vec<usize>,
}

/// Build the junction set from a routable way's `(id, ordered node refs)`
/// list. Two passes over already-in-memory data (no PBF re-read): count
/// occurrences per node, then re-walk to collect each junction's edge-way
/// list in a fixed order.
fn build_junctions(routable_ways: &mut [(i64, Vec<i64>)], coords: &PosMap) -> Vec<Junction> {
    routable_ways.sort_unstable_by_key(|(id, _)| *id);

    let mut occurrences: HashMap<i64, u32> = HashMap::with_capacity(routable_ways.len() * 4);
    for (_, nodes) in routable_ways.iter() {
        for n in nodes {
            *occurrences.entry(*n).or_insert(0) += 1;
        }
    }

    // (way_id, index-within-that-way's-node-list) per occurrence — the index
    // is what lets a bearing be computed later (see `Junction::edge_way_index`'s
    // doc for why the way id alone is not enough).
    let mut edges: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
    for (way_id, nodes) in routable_ways.iter() {
        for (idx, n) in nodes.iter().enumerate() {
            if occurrences.get(n).copied().unwrap_or(0) >= 2 {
                edges.entry(*n).or_default().push((*way_id, idx));
            }
        }
    }

    let mut out: Vec<Junction> = edges
        .into_iter()
        .filter_map(|(node_id, occs)| {
            coords.get(&node_id).map(|&cell| Junction {
                node_id,
                cell,
                edge_ways: occs.iter().map(|&(w, _)| w).collect(),
                edge_way_index: occs.iter().map(|&(_, i)| i).collect(),
            })
        })
        .collect();
    // Deterministic output order: two bakes of one extract must be
    // byte-identical, matching the guarantee `row::sort_key` already gives
    // the rest of the pipeline.
    out.sort_unstable_by_key(|j| j.node_id);
    out
}

/// [`read_features`], additionally returning each **tagged way's** full vertex
/// chain (`osm way id -> z=32 cells, in ref order`).
///
/// The chain is the `cells` vec the way arm ALREADY builds to derive the
/// anchor — this keeps it for tagged ways instead of dropping it, which is the
/// entire difference. `bake` joins it to identity ordinals after
/// `resolve_identities` and persists it as the `.chains` sidecar
/// ([`crate::chains`]); the probes keep calling the dropping wrapper.
///
/// # Errors
///
/// Same as [`read_features`].
pub fn read_features_with_chains(path: &str) -> Result<ReadWithChains, Box<dyn std::error::Error>> {
    let mut stats = ReadStats::default();
    let mut store = TagStore::default();

    // ── Pass 1: node id → (lon, lat). ──
    let mut coords: PosMap = HashMap::with_capacity(8_000_000);
    ElementReader::from_path(path)?.for_each(|el| match el {
        Element::Node(n) => {
            coords.insert(n.id(), point_to_cell(n.lon(), n.lat()));
        }
        Element::DenseNode(n) => {
            coords.insert(n.id(), point_to_cell(n.lon(), n.lat()));
        }
        _ => {}
    })?;
    stats.nodes_indexed = coords.len() as u64;
    eprintln!("pass 1: {} nodes indexed", stats.nodes_indexed);

    // ── Pass 2: tagged elements → anchored features. ──
    let mut out: Vec<Feature> = Vec::with_capacity(2_600_000);
    let mut ways_unresolved = 0u64;
    let mut way_centroid: PosMap = HashMap::with_capacity(1_400_000);
    let mut way_chains: HashMap<i64, Vec<TileXy>> = HashMap::with_capacity(1_400_000);
    let mut pending: Vec<(i64, RawRelation)> = Vec::new();
    // Routable ways' raw node-id lists, for junction detection after this
    // pass — kept separate from `way_chains` (which holds resolved CELLS for
    // every tagged way, routable or not) because junction detection needs
    // node IDENTITY, which a cell cannot give back once two ids round to the
    // same quantized cell.
    let mut routable_ways: Vec<(i64, Vec<i64>)> = Vec::with_capacity(480_000);
    ElementReader::from_path(path)?.for_each(|el| match el {
        Element::Node(n) => {
            if n.tags().next().is_some() {
                let tags = store.push(n.tags());
                out.push(Feature {
                    anchor: Anchor::Published {
                        lon: n.lon(),
                        lat: n.lat(),
                    },
                    entity_type: OSM_NODE,
                    osm_id: n.id(),
                    tags,
                });
            }
        }
        Element::DenseNode(n) => {
            if n.tags().next().is_some() {
                let tags = store.push(n.tags());
                out.push(Feature {
                    anchor: Anchor::Published {
                        lon: n.lon(),
                        lat: n.lat(),
                    },
                    entity_type: OSM_NODE,
                    osm_id: n.id(),
                    tags,
                });
            }
        }
        Element::Way(w) => {
            // Centroid is computed for EVERY way, tagged or not: a relation may
            // reference an untagged way (a multipolygon inner ring, a via way),
            // and that member still has to resolve.
            let cells: Vec<TileXy> = w.refs().filter_map(|id| coords.get(&id).copied()).collect();
            let tagged = w.tags().next().is_some();
            // Routability needs the ORIGINAL tag iterator, so this is checked
            // before `store.push(w.tags())` below consumes it — `w.tags()`
            // itself is a fresh iterator per call (cheap; the way is already
            // decoded), so calling it twice costs no re-parse.
            if is_routable(w.tags()) {
                routable_ways.push((w.id(), w.refs().collect()));
            }
            let Some(c) = mean_cell(&cells) else {
                if tagged {
                    ways_unresolved += 1;
                }
                return;
            };
            way_centroid.insert(w.id(), c);
            if tagged {
                let tags = store.push(w.tags());
                out.push(Feature {
                    anchor: Anchor::Derived(c),
                    entity_type: OSM_WAY,
                    osm_id: w.id(),
                    tags,
                });
                // The shape was computed to derive the anchor; keep it instead
                // of dropping it. Unresolved refs were already filtered above,
                // so the chain holds only vertices pass 1 could place.
                way_chains.insert(w.id(), cells);
            }
        }
        Element::Relation(r) => {
            if r.tags().next().is_none() {
                return;
            }
            let restriction = is_restriction(r.tags().find(|(k, _)| *k == "type").map(|(_, v)| v));
            let tags = store.push(r.tags());
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
                    tags,
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
            Some(cell) => {
                stats.relations_anchored += 1;
                if rel.restriction {
                    stats.restrictions += 1;
                }
                out.push(Feature {
                    anchor: Anchor::Derived(cell),
                    entity_type: OSM_RELATION,
                    osm_id: *rid,
                    tags: rel.tags,
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
    stats.tag_pairs = store.pair_count() as u64;
    stats.distinct_tag_keys = store.distinct_keys() as u64;
    stats.distinct_tag_values = store.distinct_values() as u64;
    eprintln!(
        "tags: {} pairs, {} distinct keys, {} distinct values",
        stats.tag_pairs, stats.distinct_tag_keys, stats.distinct_tag_values,
    );
    let junctions = build_junctions(&mut routable_ways, &coords);
    eprintln!(
        "junctions: {} routable ways -> {} junction nodes",
        routable_ways.len(),
        junctions.len(),
    );
    Ok((out, store, stats, way_chains, junctions))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ogar_vocab::ports::{OsmPort, PortSpec};

    #[test]
    fn the_port_and_these_constants_agree() {
        // The plug-and-play pin. `OsmPort` is the authority — the Rails name a
        // harvest produced, mapped to the minted concept — and the constants
        // above are a mirror of it. Pulling both from `ogar-vocab` means this
        // can only fail if the two upstream surfaces disagree with each other,
        // which is the drift worth catching.
        assert_eq!(OsmPort::class_id("Node"), Some(OSM_NODE));
        assert_eq!(OsmPort::class_id("Way"), Some(OSM_WAY));
        assert_eq!(OsmPort::class_id("Relation"), Some(OSM_RELATION));

        // Anti-vacuity: the port genuinely discriminates rather than mapping
        // every name to one id, and it refuses a name it does not carry.
        assert_ne!(OsmPort::class_id("Node"), OsmPort::class_id("Way"));
        assert_eq!(OsmPort::class_id("Tracepoint"), None);
    }

    fn cell(x: u32, y: u32) -> TileXy {
        TileXy { x, y_xyz: y }
    }

    #[test]
    fn a_node_shared_by_two_ways_is_a_junction_the_others_are_not() {
        // A T: way 1 runs 10-11-12, way 2 runs 20-11-21. Node 11 is the
        // shared vertex; every other node belongs to exactly one way.
        let mut coords: PosMap = HashMap::new();
        for (id, xy) in [
            (10, cell(0, 0)),
            (11, cell(1, 0)),
            (12, cell(2, 0)),
            (20, cell(1, 1)),
            (21, cell(1, 2)),
        ] {
            coords.insert(id, xy);
        }
        let mut ways = vec![(1i64, vec![10, 11, 12]), (2i64, vec![20, 11, 21])];

        let junctions = build_junctions(&mut ways, &coords);

        assert_eq!(junctions.len(), 1, "exactly one shared node");
        let j = &junctions[0];
        assert_eq!(j.node_id, 11);
        assert_eq!(j.cell, cell(1, 0));
        // Deterministic order: ways sorted by id, so way 1 before way 2.
        assert_eq!(j.edge_ways, vec![1, 2]);
        // Node 11 sits at index 1 in way 1's list (`[10, 11, 12]`) and index
        // 1 in way 2's list (`[20, 11, 21]`) too — a coincidence of this
        // fixture, not a rule; the loop-back test below is what proves the
        // field discriminates when the indices genuinely differ.
        assert_eq!(j.edge_way_index, vec![1, 1]);

        // ...and it's genuinely discriminating: nodes 10/12/20/21, each
        // touched by exactly one way exactly once, must NOT appear at all.
        for solo in [10, 12, 20, 21] {
            assert!(
                !junctions.iter().any(|j| j.node_id == solo),
                "node {solo} is not shared and must not be a junction"
            );
        }
    }

    #[test]
    fn a_way_that_loops_back_through_one_node_contributes_two_edge_occurrences() {
        // Way 1 visits node 5 twice (a lasso shape: 1-2-5-3-4-5-1-back-out via
        // a second way so 5 also qualifies via a real second reference — a
        // SOLE way revisiting its own node must still not, by itself, count
        // as "shared" under the >=2-DISTINCT-touch population definition this
        // module measures against junction_probe; what's under test here is
        // that once a node DOES qualify (via way 2), every occurrence across
        // ALL ways — including a repeated one within a single way — becomes
        // its own edge slot, not deduplicated to one.
        let mut coords: PosMap = HashMap::new();
        for (id, xy) in [(1, cell(0, 0)), (2, cell(1, 0)), (5, cell(2, 0))] {
            coords.insert(id, xy);
        }
        let mut ways = vec![(1i64, vec![1, 5, 2, 5]), (2i64, vec![5, 1])];

        let junctions = build_junctions(&mut ways, &coords);
        let j = junctions
            .iter()
            .find(|j| j.node_id == 5)
            .expect("node 5 qualifies (way 2 also touches it)");
        // Way 1 visits node 5 at two positions -> two entries of way id 1;
        // way 2 visits it once -> one entry of way id 2. Three total, not
        // deduplicated to "which ways touch this node".
        assert_eq!(j.edge_ways, vec![1, 1, 2]);
        // The two way-1 occurrences must carry DIFFERENT indices (positions 1
        // and 3 in `[1, 5, 2, 5]`) — this is the field a bearing needs: two
        // identical indices here would make both occurrences resolve to the
        // same neighbor and report an identical heading, which is exactly the
        // silently-wrong outcome this field exists to prevent.
        assert_eq!(j.edge_way_index, vec![1, 3, 0]);

        // Node 1 is touched once by way 1 and once by way 2 -> a genuine
        // 2-way junction too, one entry each.
        let j1 = junctions
            .iter()
            .find(|j| j.node_id == 1)
            .expect("node 1 also qualifies");
        assert_eq!(j1.edge_ways, vec![1, 2]);
        assert_eq!(j1.edge_way_index, vec![0, 1]);

        // Node 2 is touched ONLY by way 1, at one position -> not a junction.
        assert!(!junctions.iter().any(|j| j.node_id == 2));
    }

    #[test]
    fn a_junction_node_with_no_resolved_coordinate_is_dropped_not_fabricated() {
        // Pass 1 indexes every node it sees; a way that references a node id
        // pass 1 never saw (a real, if rare, extract inconsistency) must not
        // produce a junction row keyed at a made-up position.
        let mut coords: PosMap = HashMap::new();
        coords.insert(10, cell(0, 0)); // node 11 deliberately absent
        let mut ways = vec![(1i64, vec![10, 11]), (2i64, vec![10, 11])];
        let junctions = build_junctions(&mut ways, &coords);
        assert_eq!(junctions.len(), 1, "only the resolvable junction ships");
        assert_eq!(junctions[0].node_id, 10);
    }

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
        let cell = |lon, lat| point_to_cell(lon, lat);
        let coords = HashMap::from([(1i64, cell(13.40, 52.50)), (2i64, cell(13.60, 52.70))]);
        let ways = HashMap::from([(10i64, cell(13.32, 52.44)), (20i64, cell(13.50, 52.60))]);
        (coords, ways)
    }

    fn m(is_node: bool, id: i64, via: bool) -> Member {
        Member { is_node, id, via }
    }

    /// A relation fixture. Anchoring never reads tags, so an empty span keeps
    /// these fixtures about geometry — which is the only thing they test.
    fn rel(restriction: bool, members: Vec<Member>) -> RawRelation {
        RawRelation {
            restriction,
            members,
            tags: TagSpan::default(),
        }
    }

    #[test]
    fn via_node_anchors_at_the_junction_not_the_member_centroid() {
        // The load-bearing case: 93.4% of Berlin's restrictions take this path.
        let (c, w) = maps();
        let rel = rel(
            true,
            vec![m(false, 10, false), m(true, 1, true), m(false, 20, false)],
        );
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(how, AnchorVia::Node);
        assert_eq!(
            anchor,
            Some(point_to_cell(13.40, 52.50)),
            "must be the via node exactly"
        );
        // Two-sided: it must NOT be the centroid of from/via/to, which is where
        // a naive implementation would put it.
        let centroid = mean_cell(&[
            point_to_cell(13.32, 52.44),
            point_to_cell(13.40, 52.50),
            point_to_cell(13.50, 52.60),
        ]);
        assert_ne!(
            centroid,
            Some(point_to_cell(13.40, 52.50)),
            "fixture must not be symmetric about the via"
        );
        assert_ne!(anchor, centroid, "a centroid anchor would be wrong here");
    }

    #[test]
    fn via_way_anchors_at_that_ways_centroid() {
        let (c, w) = maps();
        let rel = rel(true, vec![m(false, 10, false), m(false, 20, true)]);
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(how, AnchorVia::Way);
        assert_eq!(anchor, Some(point_to_cell(13.50, 52.60)));
    }

    #[test]
    fn no_via_falls_back_to_a_genuine_member_centroid() {
        let (c, w) = maps();
        let rel = rel(false, vec![m(true, 1, false), m(true, 2, false)]);
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(how, AnchorVia::Absent);
        // A real average of two distinct members, not just the first one — and
        // the average is taken in CELL space, which is what makes the anchor a
        // grid point by construction.
        let want = mean_cell(&[point_to_cell(13.40, 52.50), point_to_cell(13.60, 52.70)]);
        assert_eq!(anchor, want);
        assert_ne!(
            anchor,
            Some(point_to_cell(13.40, 52.50)),
            "must not collapse to member 0"
        );
        // Worth pinning explicitly: the cell-space mean is NOT the cell of the
        // lon/lat midpoint. Mercator's y is non-linear in latitude, so the mean
        // of projections is not the projection of the mean. Over this fixture's
        // deliberately wide 0.2° span the two differ; over a real feature's
        // extent they converge (Mercator is conformal, so cells are locally
        // isotropic). `tier_probe` measures the real distribution.
        assert_ne!(
            anchor,
            Some(point_to_cell(13.50, 52.60)),
            "cell-space mean differs from the coordinate midpoint at this span"
        );
    }

    #[test]
    fn an_unresolvable_via_falls_back_rather_than_failing() {
        // A via clipped out of the extract must not sink the whole relation.
        let (c, w) = maps();
        let rel = rel(
            true,
            vec![m(true, 999, true), m(true, 1, false), m(true, 2, false)],
        );
        let (anchor, how) = anchor_relation(&rel, &c, &w);
        assert_eq!(
            how,
            AnchorVia::Absent,
            "an absent via is reported as absent"
        );
        assert_eq!(
            anchor,
            mean_cell(&[point_to_cell(13.40, 52.50), point_to_cell(13.60, 52.70)]),
            "still anchored, via the cell-space centroid"
        );
    }

    #[test]
    fn a_super_relation_is_unanchorable_and_that_is_discriminating() {
        let (c, w) = maps();
        // route_master and friends: every member is another relation, filtered
        // out before this point, so the member list is empty. 689 of Berlin's
        // 18,128 tagged relations are this shape.
        let empty = rel(false, vec![]);
        assert_eq!(anchor_relation(&empty, &c, &w).0, None);
        // ...and members that exist but resolve to nothing.
        let ghosts = rel(false, vec![m(true, 777, false), m(false, 888, false)]);
        assert_eq!(anchor_relation(&ghosts, &c, &w).0, None);
        // Can-stay-silent paired with can-fire: an ordinary relation DOES
        // anchor, so `None` means something.
        let ok = rel(false, vec![m(true, 1, false)]);
        assert!(anchor_relation(&ok, &c, &w).0.is_some());
    }

    #[test]
    fn only_the_first_via_is_consulted_and_ordering_does_not_matter() {
        // The via is found by role, not by position, so a relation whose via is
        // last resolves identically to one whose via is first. This is what
        // makes collecting members and resolving afterwards safe regardless of
        // element order in the file.
        let (c, w) = maps();
        let first = rel(true, vec![m(true, 1, true), m(false, 10, false)]);
        let last = rel(true, vec![m(false, 10, false), m(true, 1, true)]);
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
