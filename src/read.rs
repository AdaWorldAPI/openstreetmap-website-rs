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
//! Relations are **not** baked in v1 — resolving their member geometry needs a
//! third pass; they are 18,128 of 2,527,304 (0.7%) and are reported as skipped
//! rather than silently dropped.

use std::collections::HashMap;

use osmpbf::{Element, ElementReader};

/// OGAR concept ids for the OSM element kinds (`ogar_codebook`, geo domain
/// `0x0F`). These land in the `EntityType` value tenant — the element's kind is
/// a *class*, never a slot in the key (le-contract §2 slot purity).
pub const OSM_NODE: u16 = 0x0F01;
pub const OSM_WAY: u16 = 0x0F02;
pub const OSM_RELATION: u16 = 0x0F03;

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
    pub relations_skipped: u64,
    pub ways_unresolved: u64,
}

/// Read every tagged feature from `path`, anchored.
pub fn read_features(path: &str) -> Result<(Vec<Feature>, ReadStats), Box<dyn std::error::Error>> {
    let mut stats = ReadStats::default();

    // ── Pass 1: node id → (lon, lat). ──
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(8_000_000);
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
    let mut rel_skipped = 0u64;
    let mut ways_unresolved = 0u64;
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
            if w.tags().next().is_none() {
                return;
            }
            let (mut slon, mut slat, mut cnt) = (0.0f64, 0.0f64, 0u32);
            for id in w.refs() {
                if let Some(&(lon, lat)) = coords.get(&id) {
                    slon += lon;
                    slat += lat;
                    cnt += 1;
                }
            }
            if cnt == 0 {
                ways_unresolved += 1;
                return;
            }
            out.push(Feature {
                lon: slon / f64::from(cnt),
                lat: slat / f64::from(cnt),
                entity_type: OSM_WAY,
                osm_id: w.id(),
            });
        }
        Element::Relation(r) => {
            // Not baked in v1: a relation's anchor needs its member ways'
            // geometry, i.e. a third pass. Counted, never silently dropped.
            if r.tags().next().is_some() {
                rel_skipped += 1;
            }
        }
    })?;
    stats.relations_skipped = rel_skipped;

    // Recount by kind from what was actually emitted (never from an assumption).
    for f in &out {
        match f.entity_type {
            OSM_NODE => stats.tagged_nodes += 1,
            OSM_WAY => stats.tagged_ways += 1,
            _ => {}
        }
    }
    stats.ways_unresolved = ways_unresolved;
    eprintln!(
        "pass 2: {} tagged nodes + {} tagged ways = {} features",
        stats.tagged_nodes,
        stats.tagged_ways,
        out.len()
    );
    Ok((out, stats))
}
