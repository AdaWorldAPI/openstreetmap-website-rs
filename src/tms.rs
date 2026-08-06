//! WebMercator `(lon, lat)` → **Cesium TMS** quadkey → the four HHTL tiers.
//!
//! The key is TMS, not OSM-XYZ. That is the ratified choice, not a preference:
//! `lance-graph/.claude/plans/cesium-osm-substrate-v1.md` §2 Q2/Q3 (locked
//! 2026-06-05) chose the Cesium TMS quadkey so that "all OSM features inside
//! this Cesium tile" is a single prefix scan against `implicit_tiling` subtree
//! coordinates, and mandates the OSM-XYZ → TMS Y-flip **at the ingest
//! boundary** (Q3) so the runtime only ever sees one key.
//!
//! The two orderings differ only in Y: OSM-XYZ counts Y top-down (web
//! standard), Cesium TMS counts Y bottom-up. One subtract per feature, here.
//!
//! # Why z = 32
//!
//! Four tiers × 8 quadtree levels = 32 levels, filling the V3 facet's
//! `heel|hip|twig|leaf`. Measured round-trip error (exact lon/lat → tile index
//! → tile centre → lon/lat) at z=32: **1.13 mm** at Berlin, 6.59 mm at the
//! equator (worst case). That is ~1000× finer than a GNSS fix, so **the key
//! carries the exact coordinate** and a position needs no value lane. At z=24
//! (a 3-tier key) the same round trip errs 0.27–1.69 m — the same order as the
//! fix itself, which is why the 3-tier form is not used here.

/// Native depth: 4 tiers × 8 quadtree levels.
pub const HHTL_DEPTH4: u32 = 32;

const PI: f64 = std::f64::consts::PI;

/// The four spatial tiers of the V3 facet, each a 256×256 centroid tile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Tiers {
    pub heel: u16,
    pub hip: u16,
    pub twig: u16,
    pub leaf: u16,
}

/// WebMercator forward: `(lon, lat)` → slippy tile `(x, y)` at zoom `z`.
/// `x` grows east, `y` grows south (the XYZ convention; the TMS flip is
/// applied separately by [`xyz_to_tms_y`]). Clamped to `0..2^z`.
#[must_use]
pub fn lonlat_to_tile(lon: f64, lat: f64, z: u32) -> (u32, u32) {
    let n = f64::from(2u32.saturating_pow(z.min(31)));
    let n = if z >= 32 { 4_294_967_296.0 } else { n };
    let x = ((lon + 180.0) / 360.0 * n).floor();
    // Clamp latitude to the Mercator validity band (±85.051129°) before tan/cos.
    let lat = lat.clamp(-85.051_128_78, 85.051_128_78);
    let lat_rad = lat.to_radians();
    // asinh(tan φ) = ln(tan φ + sec φ) — the Mercator y, no PROJ needed.
    let merc_y = (lat_rad.tan() + 1.0 / lat_rad.cos()).ln();
    let y = ((1.0 - merc_y / PI) / 2.0 * n).floor();
    let max = n - 1.0;
    (x.clamp(0.0, max) as u32, y.clamp(0.0, max) as u32)
}

/// OSM-XYZ tile `y` → Cesium TMS `y` at zoom `z` — the Q3 boundary flip.
/// Self-inverse.
#[must_use]
pub fn xyz_to_tms_y(z: u32, y: u32) -> u32 {
    let n: u64 = 1u64 << z.min(32);
    ((n - 1).saturating_sub(u64::from(y))) as u32
}

/// Interleave two 32-bit lanes into a 64-bit Morton code (`x`→even bits,
/// `y`→odd bits). This is the trie: a shared prefix IS a shared tile, exactly.
#[must_use]
pub fn morton64(x: u32, y: u32) -> u64 {
    let mut code = 0u64;
    for i in 0..HHTL_DEPTH4 {
        code |= u64::from((x >> i) & 1) << (2 * i);
        code |= u64::from((y >> i) & 1) << (2 * i + 1);
    }
    code
}

/// `(lon, lat)` → the Cesium-TMS Morton code at z=32.
#[must_use]
pub fn point_to_tms_morton(lon: f64, lat: f64) -> u64 {
    let (x, y_xyz) = lonlat_to_tile(lon, lat, HHTL_DEPTH4);
    morton64(x, xyz_to_tms_y(HHTL_DEPTH4, y_xyz))
}

/// Split a 64-bit Morton code into the four `u16` cascade tiers, coarse first.
#[must_use]
pub fn tiers_of(code: u64) -> Tiers {
    Tiers {
        heel: (code >> 48) as u16,
        hip: (code >> 32) as u16,
        twig: (code >> 16) as u16,
        leaf: code as u16,
    }
}

/// The whole answer: a geographic point → its four TMS-keyed HHTL tiers.
#[must_use]
pub fn point_to_tiers(lon: f64, lat: f64) -> (u64, Tiers) {
    let code = point_to_tms_morton(lon, lat);
    (code, tiers_of(code))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tms_y_flip_is_its_own_inverse() {
        for z in [0u32, 4, 12, 20, 32] {
            for y in [0u32, 1, 7, 100] {
                let n = 1u64 << z;
                if u64::from(y) < n {
                    assert_eq!(xyz_to_tms_y(z, xyz_to_tms_y(z, y)), y);
                }
            }
        }
    }

    #[test]
    fn tms_key_differs_from_xyz_off_the_equator_row() {
        // The ratified difference this module exists to preserve (Q2/Q3). If
        // these ever coincide the Y-flip has been lost.
        let (x, y) = lonlat_to_tile(13.404954, 52.520008, HHTL_DEPTH4);
        let xyz = morton64(x, y);
        let tms = point_to_tms_morton(13.404954, 52.520008);
        assert_ne!(
            tms, xyz,
            "TMS and XYZ keys must diverge off the equator row"
        );
    }

    #[test]
    fn morton_prefix_is_tile_containment() {
        // Two points in the same coarse tile share the HEEL tier; two points
        // far apart do not. This is the trie property the whole design rests on.
        let (_, a) = point_to_tiers(13.404954, 52.520008); // Berlin Mitte
        let (_, b) = point_to_tiers(13.404960, 52.520010); // ~0.4 m away
        let (_, c) = point_to_tiers(-122.3472, 47.598); // Seattle
        assert_eq!(a.heel, b.heel);
        assert_eq!(a.hip, b.hip);
        assert_ne!(a.heel, c.heel);
    }

    #[test]
    fn tiers_roundtrip_the_morton_code() {
        let code = point_to_tms_morton(13.404954, 52.520008);
        let t = tiers_of(code);
        let back = (u64::from(t.heel) << 48)
            | (u64::from(t.hip) << 32)
            | (u64::from(t.twig) << 16)
            | u64::from(t.leaf);
        assert_eq!(code, back);
    }

    #[test]
    fn key_recovers_the_coordinate_to_millimetres() {
        // The measured claim in the module doc, as a test: the z=32 key is a
        // lossless-enough carrier of a GNSS-grade coordinate.
        let (lon, lat) = (13.404954, 52.520008);
        let (x, y_xyz) = lonlat_to_tile(lon, lat, HHTL_DEPTH4);
        let n = 4_294_967_296.0f64;
        let rlon = (f64::from(x) + 0.5) / n * 360.0 - 180.0;
        let rlat = (PI * (1.0 - 2.0 * (f64::from(y_xyz) + 0.5) / n))
            .sinh()
            .atan()
            .to_degrees();
        // ~1.1 mm at Berlin; assert well inside a centimetre.
        let dlat_m = (rlat - lat).abs() * 111_320.0;
        let dlon_m = (rlon - lon).abs() * 111_320.0 * lat.to_radians().cos();
        assert!(dlat_m < 0.01, "lat err {dlat_m} m");
        assert!(dlon_m < 0.01, "lon err {dlon_m} m");
    }
}
