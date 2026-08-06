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
//! equator (worst case). At z=24 (a 3-tier key) the same round trip errs
//! 0.27–1.69 m — the same order as a GNSS fix, which is why the 3-tier form is
//! not used here.
//!
//! # Position parity with OSM (measured, not inferred)
//!
//! A millimetre bound says the recovered coordinate is *close*. OSM parity
//! needs it *equal*: a `.osm.pbf` stores every coordinate as an integer number
//! of `1e-7` degrees (`int64` nanodegrees at the default granularity 100), so
//! reproducing the file means reproducing that integer.
//!
//! It does, and the reason is arithmetic rather than luck. The z=32 cell is
//! **narrower than one OSM step everywhere** — `360/2³² = 8.382e-8°` in
//! longitude against OSM's `1.000e-7°`, and narrower still in latitude away
//! from the equator, since Mercator cells shrink with `cos φ`:
//!
//! | latitude | z=32 cell | vs OSM step |
//! |---|---|---|
//! | 0° (worst case) | 8.382e-8° | 0.84× |
//! | 52.52° (Berlin) | 5.100e-8° | 0.51× |
//! | 71° | 2.729e-8° | 0.27× |
//!
//! So the cell centre lies within `4.19e-8°` of any point inside it — inside
//! the `5e-8°` half-step that decides which grid point is nearest. Snapping is
//! therefore exact, not approximate. Verified over **1,000,000** grid points
//! spanning both hemispheres and both meridians
//! (`tests/position_parity.rs`), and shown to be a property of z=32
//! specifically: one level coarser and the cell (1.68e-7°) exceeds OSM's own
//! step, so adjacent published coordinates collide
//! (`a_coarser_key_would_lose_the_coordinate`).
//!
//! **The key therefore carries the position and no value lane is needed** —
//! with one named boundary: beyond ±85.051129° the Mercator projection ends,
//! [`lonlat_to_tile`] clamps, and distinct coordinates share a key. A bake
//! covering those latitudes needs a position lane; see
//! [`osm_grid_is_representable`].

/// Native depth: 4 tiers × 8 quadtree levels.
pub const HHTL_DEPTH4: u32 = 32;

/// Steps per degree on OSM's own coordinate grid.
///
/// A `.osm.pbf` stores coordinates as `int64` nanodegrees scaled by the file's
/// `granularity` (default **100**), so every published coordinate is an integer
/// number of `1e-7` degrees. Parity with OSM therefore means reproducing that
/// integer, not merely landing close to it.
pub const OSM_GRID_PER_DEGREE: f64 = 1e7;

/// The Mercator validity band, in degrees.
///
/// Outside it the projection is undefined and [`lonlat_to_tile`] clamps — so a
/// key cannot carry a position beyond this latitude. See
/// [`osm_grid_is_representable`].
pub const MERCATOR_LAT_LIMIT: f64 = 85.051_128_78;

const PI: f64 = std::f64::consts::PI;

/// `2^32` as an exact `f64` — the z=32 grid side.
const N32: f64 = 4_294_967_296.0;

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
    let n = if z >= 32 { N32 } else { n };
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

// ── The inverse: key → position ─────────────────────────────────────
//
// The key is only a *carrier* of the coordinate if the coordinate can be read
// back out of it. Everything below is that read. It exists so "identical data
// to OSM" is a measured property (see the round-trip tests) rather than an
// inference from the millimetre error figure in the module doc — a millimetre
// bound says the answer is *close*, and OSM parity needs it *exact*.

/// De-interleave a 64-bit Morton code into its two 32-bit lanes.
/// Exact inverse of [`morton64`].
#[must_use]
pub fn demorton64(code: u64) -> (u32, u32) {
    let mut x = 0u32;
    let mut y = 0u32;
    for i in 0..HHTL_DEPTH4 {
        x |= (((code >> (2 * i)) & 1) as u32) << i;
        y |= (((code >> (2 * i + 1)) & 1) as u32) << i;
    }
    (x, y)
}

/// XYZ tile `(x, y)` at z=32 → the **centre** of that tile, in degrees.
///
/// The centre, not a corner: it is the point of the cell furthest from every
/// edge, so it minimises the worst-case distance to whatever real coordinate
/// produced the tile. That is what makes the snap in [`morton_to_osm_grid`]
/// land on the right grid point rather than a neighbour.
#[must_use]
pub fn tile_to_lonlat(x: u32, y_xyz: u32) -> (f64, f64) {
    let lon = (f64::from(x) + 0.5) / N32 * 360.0 - 180.0;
    let merc_y = PI * (1.0 - 2.0 * (f64::from(y_xyz) + 0.5) / N32);
    let lat = merc_y.sinh().atan().to_degrees();
    (lon, lat)
}

/// A TMS Morton code → the centre of the cell it names.
#[must_use]
pub fn morton_to_lonlat(code: u64) -> (f64, f64) {
    let (x, y_tms) = demorton64(code);
    tile_to_lonlat(x, xyz_to_tms_y(HHTL_DEPTH4, y_tms))
}

/// A TMS Morton code → the **OSM grid point** it recovers, in units of `1e-7`
/// degrees — the integers a `.osm.pbf` actually stores.
///
/// Snapping the cell centre to OSM's grid is exact rather than approximate
/// because the z=32 cell is *narrower than one OSM step everywhere*: 8.38e-8°
/// in longitude against OSM's 1.00e-7°, and narrower still in latitude away
/// from the equator (Mercator cells shrink with `cos φ`). The centre therefore
/// sits within 4.19e-8° of any point in its cell — inside the half-step
/// (5e-8°) that decides which grid point is nearest. The equator is the worst
/// case and it is the one the tests hammer.
#[must_use]
pub fn morton_to_osm_grid(code: u64) -> (i32, i32) {
    let (lon, lat) = morton_to_lonlat(code);
    (
        (lon * OSM_GRID_PER_DEGREE).round() as i32,
        (lat * OSM_GRID_PER_DEGREE).round() as i32,
    )
}

/// An OSM grid point (units of `1e-7` degrees) → its TMS Morton code.
/// The forward half of the parity round trip.
#[must_use]
pub fn osm_grid_to_morton(lon_e7: i32, lat_e7: i32) -> u64 {
    point_to_tms_morton(
        f64::from(lon_e7) / OSM_GRID_PER_DEGREE,
        f64::from(lat_e7) / OSM_GRID_PER_DEGREE,
    )
}

/// Whether an OSM grid point is inside the band the key can carry.
///
/// Latitudes beyond ±85.051129° leave the Mercator projection's domain and
/// [`lonlat_to_tile`] clamps them, so the key stops distinguishing them. This
/// is a real parity boundary, named rather than hidden: a bake covering the
/// high Arctic or Antarctica needs a position lane, not just a key.
#[must_use]
pub fn osm_grid_is_representable(lat_e7: i32) -> bool {
    (f64::from(lat_e7) / OSM_GRID_PER_DEGREE).abs() < MERCATOR_LAT_LIMIT
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

    /// One OSM grid point through the whole key and back out again.
    fn grid_roundtrip(lon_e7: i32, lat_e7: i32) {
        let code = osm_grid_to_morton(lon_e7, lat_e7);
        assert_eq!(
            morton_to_osm_grid(code),
            (lon_e7, lat_e7),
            "the key lost the OSM coordinate ({lon_e7}, {lat_e7})"
        );
    }

    #[test]
    fn morton_deinterleave_is_the_exact_inverse() {
        // Everything below rests on this; a lane that does not come back out
        // makes every position claim vacuous.
        for (x, y) in [
            (0u32, 0u32),
            (1, 0),
            (0, 1),
            (u32::MAX, 0),
            (0, u32::MAX),
            (u32::MAX, u32::MAX),
            (0x1234_5678, 0x9abc_def0),
            (0xdead_beef, 0x0bad_f00d),
        ] {
            assert_eq!(demorton64(morton64(x, y)), (x, y));
        }
    }

    #[test]
    fn osm_grid_points_survive_the_key_exactly() {
        // OSM parity, measured: a `.osm.pbf` coordinate is an integer number
        // of 1e-7 degrees, and the z=32 key must hand back THAT INTEGER — not
        // a millimetre-close neighbour. If this fails, "identical data to OSM"
        // needs a position lane and the key alone is not enough.
        let lats_e7 = [
            0, // the equator — the widest Mercator cell, worst case
            1, // one grid step off it
            -1,
            525_200_080,  // Berlin
            -338_688_000, // Sydney
            600_000_000,
            -600_000_000,
            800_000_000, // deep in the band, cells are tiny here
            850_000_000, // just inside the Mercator limit
        ];
        let lons_e7 = [
            0,
            1,
            -1,
            134_049_540, // Berlin
            1_799_999_999,
            -1_799_999_999,
            -1_223_472_000,
        ];
        for &lat in &lats_e7 {
            for &lon in &lons_e7 {
                grid_roundtrip(lon, lat);
            }
        }
    }

    #[test]
    fn consecutive_grid_points_stay_distinct_along_the_equator() {
        // The equator is where Mercator cells are widest, so it is where two
        // adjacent OSM coordinates are likeliest to collapse into one cell.
        // Sweep a contiguous run of real grid points in both axes and assert
        // both distinctness (no two share a key) and exact recovery.
        let mut seen = std::collections::HashSet::new();
        for step in 0..4_000i32 {
            let lon = 134_049_540 + step;
            let code = osm_grid_to_morton(lon, 0);
            assert!(seen.insert(code), "two OSM longitudes share one cell");
            assert_eq!(morton_to_osm_grid(code), (lon, 0));
        }
        seen.clear();
        for step in 0..4_000i32 {
            let lat = step - 2_000;
            let code = osm_grid_to_morton(134_049_540, lat);
            assert!(seen.insert(code), "two OSM latitudes share one cell");
            assert_eq!(morton_to_osm_grid(code), (134_049_540, lat));
        }
    }

    #[test]
    fn a_coarser_key_would_lose_the_coordinate() {
        // Can-fail half: the exactness above is a property of z=32, not of the
        // method. One level coarser and the cell (1.68e-7 deg) is wider than
        // OSM's own step (1.00e-7), so adjacent published coordinates must
        // collide. Without this, the passing tests would read as "any quadkey
        // carries OSM positions", which is false.
        let a = lonlat_to_tile(13.404_954_0, 0.0, 31);
        let b = lonlat_to_tile(13.404_954_1, 0.0, 31);
        assert_eq!(a, b, "z=31 must merge two distinct OSM longitudes");
        // …and z=32 must not.
        let a32 = lonlat_to_tile(13.404_954_0, 0.0, HHTL_DEPTH4);
        let b32 = lonlat_to_tile(13.404_954_1, 0.0, HHTL_DEPTH4);
        assert_ne!(a32, b32, "z=32 must separate them");
    }

    #[test]
    fn the_equator_fit_is_tight_and_the_margin_is_measured() {
        // Not "it passes" but "by how much" — the headroom is 16%, so a future
        // change to the grid (a zoom, a projection, a granularity) that eats
        // it silently breaks parity. This pins the number.
        let lon_step_deg = 360.0 / N32;
        let osm_step_deg = 1.0 / OSM_GRID_PER_DEGREE;
        let ratio = lon_step_deg / osm_step_deg;
        assert!(
            (0.83..0.84).contains(&ratio),
            "cell/step ratio moved: {ratio}"
        );
        // Worst-case centre offset is half a cell; it must clear the half-step
        // that decides the nearest grid point — but only just.
        assert!(lon_step_deg / 2.0 < osm_step_deg / 2.0);
    }

    #[test]
    fn beyond_the_mercator_band_the_key_cannot_carry_the_position() {
        // The honest boundary. Inside the band, parity; outside it the tile
        // clamps and distinct Arctic coordinates share one key. A bake that
        // covers those latitudes needs a position lane, and this test is where
        // that is written down rather than discovered.
        let inside = 850_000_000; // 85.0000000 deg
        let outside = 860_000_000; // 86.0000000 deg
        assert!(osm_grid_is_representable(inside));
        assert!(!osm_grid_is_representable(outside));

        grid_roundtrip(134_049_540, inside);

        let a = osm_grid_to_morton(134_049_540, outside);
        let b = osm_grid_to_morton(134_049_540, outside + 10_000_000); // 87 deg
        assert_eq!(a, b, "outside the band the key stops discriminating");
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
