//! Position parity: does the key hand back OSM's own coordinate integer?
//!
//! The unit tests in `tms` pin the mechanism on curated points. This is the
//! breadth half — a deterministic sweep over the whole representable band,
//! because a projection failure would show up at *some* latitude, not at the
//! handful anyone thinks to write down.
//!
//! The claim under test is exactly the one "identical data to OSM" needs: a
//! `.osm.pbf` stores every coordinate as an integer number of `1e-7` degrees,
//! and the z=32 TMS key must reproduce that integer. Not close — equal.

use osm_soa_bake::tms::{
    morton_to_osm_grid, osm_grid_is_representable, osm_grid_to_morton, MERCATOR_LAT_LIMIT,
    OSM_GRID_PER_DEGREE,
};

/// SplitMix64 — a deterministic stream, so a failure is reproducible from the
/// seed alone rather than from "it failed once in CI".
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A uniform OSM grid point inside the Mercator band.
    fn grid_point(&mut self) -> (i32, i32) {
        let lon_span = 2 * 1_800_000_000i64; // ±180 deg, in 1e-7 units
        let lat_span = 2 * (MERCATOR_LAT_LIMIT * OSM_GRID_PER_DEGREE) as i64;
        let lon = (self.next() % lon_span as u64) as i64 - lon_span / 2;
        let lat = (self.next() % lat_span as u64) as i64 - lat_span / 2;
        (lon as i32, lat as i32)
    }
}

#[test]
fn a_million_osm_grid_points_survive_the_key_exactly() {
    let mut rng = SplitMix64(0x0DE1_0000_0000_0001);

    // Coverage witnesses. Without these the test would still pass if the
    // generator degenerated to one point — a million assertions about the same
    // coordinate is one assertion, and it would read as breadth.
    let (mut lon_lo, mut lon_hi) = (i32::MAX, i32::MIN);
    let (mut lat_lo, mut lat_hi) = (i32::MAX, i32::MIN);

    for _ in 0..1_000_000 {
        let (lon, lat) = rng.grid_point();
        assert!(osm_grid_is_representable(lat));

        let code = osm_grid_to_morton(lon, lat);
        assert_eq!(
            morton_to_osm_grid(code),
            (lon, lat),
            "key lost OSM coordinate ({lon}, {lat})"
        );

        lon_lo = lon_lo.min(lon);
        lon_hi = lon_hi.max(lon);
        lat_lo = lat_lo.min(lat);
        lat_hi = lat_hi.max(lat);
    }

    // Both hemispheres, near both Mercator limits, near both meridians.
    assert!(
        lon_lo < -1_799_000_000 && lon_hi > 1_799_000_000,
        "longitude span {lon_lo}..{lon_hi}"
    );
    assert!(
        lat_lo < -850_000_000 && lat_hi > 850_000_000,
        "latitude span {lat_lo}..{lat_hi}"
    );
}
