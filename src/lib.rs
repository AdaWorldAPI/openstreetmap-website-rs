//! `osm-soa-bake` — OpenStreetMap → the V3 ABI SoA row.
//!
//! The substrate behind the cockpit `/osm` endpoint. Three stages:
//! [`read`] (`.osm.pbf` → anchored tagged features), [`tms`] (point → the
//! Cesium-TMS HHTL cascade key), [`row`] (feature → a 512-byte [`NodeRow`]).
//!
//! [`NodeRow`]: lance_graph_contract::canonical_node::NodeRow

pub mod capability;
pub mod chains;
pub mod cluster;
pub mod codebook;
pub mod curve;
pub mod fragment;
pub mod geodesy;
pub mod identity;
pub mod project;
pub mod read;
pub mod row;
pub mod slab;
pub mod street;
pub mod tags;
pub mod tms;
