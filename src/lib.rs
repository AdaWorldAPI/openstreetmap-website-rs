//! `osm-soa-bake` — OpenStreetMap → the V3 ABI SoA row.
//!
//! The substrate behind the cockpit `/osm` endpoint. Three stages:
//! [`read`] (`.osm.pbf` → anchored tagged features), [`tms`] (point → the
//! Cesium-TMS HHTL cascade key), [`row`] (feature → a 512-byte [`NodeRow`]).
//!
//! [`NodeRow`]: lance_graph_contract::canonical_node::NodeRow

pub mod geodesy;
pub mod read;
pub mod row;
pub mod tms;
