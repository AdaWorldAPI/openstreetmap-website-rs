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

/// Spread the low 32 bits of `x` so a zero bit sits between every original
/// bit — the classic O(1) "binary magic numbers" interleave step (5 fixed
/// shift-mask-or stages instead of a 32-iteration loop). Exact bit-for-bit
/// replacement for what the old scalar loop computed one bit at a time;
/// verified against it in `morton_fast_matches_the_scalar_reference` below.
#[inline]
#[must_use]
fn spread_bits(x: u64) -> u64 {
    let mut x = x & 0x0000_0000_ffff_ffff;
    x = (x | (x << 16)) & 0x0000_ffff_0000_ffff;
    x = (x | (x << 8)) & 0x00ff_00ff_00ff_00ff;
    x = (x | (x << 4)) & 0x0f0f_0f0f_0f0f_0f0f;
    x = (x | (x << 2)) & 0x3333_3333_3333_3333;
    x = (x | (x << 1)) & 0x5555_5555_5555_5555;
    x
}

/// Inverse of [`spread_bits`]: compact every other bit (starting at bit 0)
/// of `x` back into the low 32 bits.
#[inline]
#[must_use]
fn compact_bits(x: u64) -> u64 {
    let mut x = x & 0x5555_5555_5555_5555;
    x = (x | (x >> 1)) & 0x3333_3333_3333_3333;
    x = (x | (x >> 2)) & 0x0f0f_0f0f_0f0f_0f0f;
    x = (x | (x >> 4)) & 0x00ff_00ff_00ff_00ff;
    x = (x | (x >> 8)) & 0x0000_ffff_0000_ffff;
    x = (x | (x >> 16)) & 0x0000_0000_ffff_ffff;
    x
}

/// Interleave two 32-bit lanes into a 64-bit Morton code (`x`→even bits,
/// `y`→odd bits). This is the trie: a shared prefix IS a shared tile, exactly.
///
/// **O(1), not O(depth).** This used to be a 32-iteration bit-by-bit loop;
/// baking a Berlin-class region calls this once per node (millions of
/// times), and `demorton64` — the same trick, below — sits on q2's
/// tile-serving hot path (`osm_features::query_tile` decodes one Morton
/// code per served row, up to hundreds of thousands per city-zoom
/// response). [`spread_bits`] replaces the loop with 5 fixed shift-mask-or
/// stages; verified bit-for-bit equivalent to the old loop, not merely
/// assumed faster-and-equal.
#[must_use]
pub fn morton64(x: u32, y: u32) -> u64 {
    spread_bits(u64::from(x)) | (spread_bits(u64::from(y)) << 1)
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

// ── Integer-native cells: the derived-anchor contract ───────────────
//
// A way or relation has no position OSM stores; this crate derives one. The
// first version derived it in the f64 continuum — mean of member lon/lat — and
// then compared the result against OSM's 1e-7 grid, which is the WRONG GRID for
// a value that was never an OSM coordinate. That comparison could only ever
// hold "within one step" (measured: 435,426 of 1,333,178 differed), and the
// derivation carried a float that had to be reproduced bit-for-bit on every
// platform for the key to be stable.
//
// The fix is to derive in the grid the key already uses. A member's cell is an
// integer; the mean of integers under a FIXED rounding rule is an integer; so a
// derived anchor is a grid point **by construction** rather than by tolerance.
// The parity check becomes `assert_eq!` on integers, and the derivation
// contains no float at all — deterministic across platforms without certifying
// any transcendental kernel.
//
// The one float that remains is the ingest step ([`point_to_cell`]), which maps
// a published coordinate to its cell. That is the same `merc_y` call a node
// already pays, once per node, and it is the node contract's item — not this
// one's.

/// A z=32 tile, in the **XYZ** convention (`y` grows south).
///
/// The TMS flip is applied when the cell becomes a key ([`cell_to_morton`]), so
/// arithmetic here stays in one convention and the flip happens exactly once —
/// the Q3 ingest-boundary rule, honoured by having a single crossing point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Hash)]
pub struct TileXy {
    pub x: u32,
    pub y_xyz: u32,
}

/// A published coordinate → its z=32 cell. **The ingest float.**
#[must_use]
pub fn point_to_cell(lon: f64, lat: f64) -> TileXy {
    let (x, y_xyz) = lonlat_to_tile(lon, lat, HHTL_DEPTH4);
    TileXy { x, y_xyz }
}

/// A cell → its TMS Morton key. Pure integer.
#[must_use]
pub fn cell_to_morton(c: TileXy) -> u64 {
    morton64(c.x, xyz_to_tms_y(HHTL_DEPTH4, c.y_xyz))
}

/// A TMS Morton key → its cell. Pure integer, exact inverse of
/// [`cell_to_morton`].
#[must_use]
pub fn morton_to_cell(code: u64) -> TileXy {
    let (x, y_tms) = demorton64(code);
    TileXy {
        x,
        y_xyz: xyz_to_tms_y(HHTL_DEPTH4, y_tms),
    }
}

/// How [`mean_cell`] rounds — part of the **artifact contract**, not an
/// implementation detail.
///
/// A derived anchor is only reproducible if the rounding rule is known, so the
/// rule travels with the bake (the codebook sidecar's header carries this
/// discriminant) instead of living only in Rust doc-comments. A reader in
/// another language reads the contract rather than reconstructing it.
///
/// Wire values are stable and `0` is deliberately not one of them: an
/// unspecified rule must be refused, never defaulted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum AnchorRounding {
    /// `(Σ + n/2) / n` — the rule this crate bakes with.
    HalfUp = 1,
}

impl AnchorRounding {
    /// The rule in force. A bake stamps this; a reader checks it.
    pub const CURRENT: AnchorRounding = AnchorRounding::HalfUp;

    /// The stable wire value.
    #[must_use]
    pub const fn wire(self) -> u32 {
        self as u32
    }

    /// A wire value back to a rule. `None` for `0` (unspecified) or anything
    /// this build does not implement — refused, never defaulted.
    #[must_use]
    pub const fn from_wire(v: u32) -> Option<Self> {
        match v {
            1 => Some(AnchorRounding::HalfUp),
            _ => None,
        }
    }
}

/// The derived-anchor rounding rule: **round half up**, per axis.
///
/// Fixed here, in the contract, rather than left to whatever integer division
/// a call site happens to write. Either rule would do — what matters is that
/// exactly one is named, versioned with the bake, and reproducible in any
/// language. Half-up is chosen because "the nearest cell" is what *centroid*
/// means; plain floor would systematically bias every derived anchor half a
/// cell toward the origin.
///
/// No overflow: each term is below `2^32` and `n` is a member count, so the sum
/// stays below `2^64` for any `n` this crate could ever see (a way caps around
/// 2,000 refs).
fn mean_axis(sum: u64, n: u64) -> u32 {
    debug_assert!(n > 0, "mean_axis needs at least one member");
    u32::try_from((sum + n / 2) / n).expect("a mean of u32 values fits a u32")
}

/// The integer centroid of a member set — a grid point **by construction**.
///
/// `None` for an empty set: a relation whose members all resolve to nothing has
/// no anchor, and inventing `(0, 0)` would place it off West Africa while
/// reading as valid. The caller counts those.
#[must_use]
pub fn mean_cell(cells: &[TileXy]) -> Option<TileXy> {
    let n = u64::try_from(cells.len()).ok().filter(|&n| n > 0)?;
    let (mut sx, mut sy) = (0u64, 0u64);
    for c in cells {
        sx += u64::from(c.x);
        sy += u64::from(c.y_xyz);
    }
    Some(TileXy {
        x: mean_axis(sx, n),
        y_xyz: mean_axis(sy, n),
    })
}

// ── The inverse: key → position ─────────────────────────────────────
//
// The key is only a *carrier* of the coordinate if the coordinate can be read
// back out of it. Everything below is that read. It exists so "identical data
// to OSM" is a measured property (see the round-trip tests) rather than an
// inference from the millimetre error figure in the module doc — a millimetre
// bound says the answer is *close*, and OSM parity needs it *exact*.

/// De-interleave a 64-bit Morton code into its two 32-bit lanes.
/// Exact inverse of [`morton64`]. See that function's doc for why this is
/// O(1) rather than a 32-iteration loop — this is the half of the trick
/// that runs on the tile-serving hot path.
#[must_use]
pub fn demorton64(code: u64) -> (u32, u32) {
    (compact_bits(code) as u32, compact_bits(code >> 1) as u32)
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
    fn a_cell_round_trips_through_the_key_with_no_float_at_all() {
        // The derived-anchor contract: cell → key → cell is the identity, by
        // integer arithmetic. This is what upgrades the way/relation check from
        // "within one grid step" to `assert_eq!`.
        for c in [
            TileXy { x: 0, y_xyz: 0 },
            TileXy { x: 1, y_xyz: 0 },
            TileXy { x: 0, y_xyz: 1 },
            TileXy {
                x: u32::MAX,
                y_xyz: u32::MAX,
            },
            point_to_cell(13.404_954, 52.520_008),
            point_to_cell(-122.3472, 47.598),
        ] {
            assert_eq!(morton_to_cell(cell_to_morton(c)), c);
        }
        // …and it agrees with the float path for a real coordinate, so the two
        // entry points cannot drift apart.
        let (lon, lat) = (13.404_954, 52.520_008);
        assert_eq!(
            cell_to_morton(point_to_cell(lon, lat)),
            point_to_tms_morton(lon, lat)
        );
    }

    #[test]
    fn the_integer_centroid_is_a_grid_point_and_rounds_half_up() {
        // A single member is its own centroid.
        let a = TileXy { x: 10, y_xyz: 20 };
        assert_eq!(mean_cell(&[a]), Some(a));

        // Two members average exactly.
        let b = TileXy { x: 20, y_xyz: 40 };
        assert_eq!(mean_cell(&[a, b]), Some(TileXy { x: 15, y_xyz: 30 }));

        // The rounding rule is HALF UP, and it is genuinely exercised: 10 and
        // 11 average to 10.5. Floor would give 10; the contract says 11.
        let c = TileXy { x: 11, y_xyz: 21 };
        assert_eq!(
            mean_cell(&[a, c]),
            Some(TileXy { x: 11, y_xyz: 21 }),
            "half-up: 10.5 -> 11, not 10"
        );

        // Empty means no anchor, never (0, 0) — which would sit off West
        // Africa and read as a valid position.
        assert_eq!(mean_cell(&[]), None);
    }

    #[test]
    fn the_integer_centroid_does_not_overflow_at_the_top_of_the_grid() {
        // Every member at the maximum cell: a u32-summing implementation would
        // wrap here and place the anchor near the origin.
        let top = TileXy {
            x: u32::MAX,
            y_xyz: u32::MAX,
        };
        let many = vec![top; 4096];
        assert_eq!(mean_cell(&many), Some(top));
        // Mixed extremes average without wrapping either.
        let mixed = vec![top, TileXy { x: 0, y_xyz: 0 }];
        assert_eq!(
            mean_cell(&mixed),
            Some(TileXy {
                x: 2_147_483_648,
                y_xyz: 2_147_483_648
            })
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

    /// The scalar, bit-by-bit loop `morton64`/`demorton64` used before the
    /// O(1) magic-bits rewrite above — kept ONLY here, as the falsifiable
    /// reference the fast version is checked against. Never call this from
    /// non-test code; it exists so "faster" and "identical" are both proven,
    /// not just the first one.
    fn morton64_scalar_reference(x: u32, y: u32) -> u64 {
        let mut code = 0u64;
        for i in 0..HHTL_DEPTH4 {
            code |= u64::from((x >> i) & 1) << (2 * i);
            code |= u64::from((y >> i) & 1) << (2 * i + 1);
        }
        code
    }

    fn demorton64_scalar_reference(code: u64) -> (u32, u32) {
        let mut x = 0u32;
        let mut y = 0u32;
        for i in 0..HHTL_DEPTH4 {
            x |= (((code >> (2 * i)) & 1) as u32) << i;
            y |= (((code >> (2 * i + 1)) & 1) as u32) << i;
        }
        (x, y)
    }

    /// Deterministic, dependency-free PRNG (splitmix64) — this crate has no
    /// `rand`/`proptest` dev-dependency, and a fixed seed makes a failure
    /// reproducible without one.
    fn splitmix64(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// **The falsifier for the O(1) rewrite.** Proves `morton64`/`demorton64`
    /// (magic-bits) agree bit-for-bit with the original scalar loop across
    /// 200,000 pseudo-random 32-bit pairs and codes, plus the boundary
    /// patterns a random sweep is least likely to hit by chance. A silent
    /// divergence here would corrupt the Morton SORT KEY every baked slab is
    /// ordered by — this is not a performance nice-to-have, it is the proof
    /// the speedup changed no observable behaviour.
    #[test]
    fn morton_fast_matches_the_scalar_reference() {
        let boundary_pairs = [
            (0u32, 0u32),
            (u32::MAX, u32::MAX),
            (u32::MAX, 0),
            (0, u32::MAX),
            (0xAAAA_AAAA, 0x5555_5555),
            (0x5555_5555, 0xAAAA_AAAA),
            (1, 1 << 31),
            (1 << 31, 1),
        ];
        for (x, y) in boundary_pairs {
            assert_eq!(
                morton64(x, y),
                morton64_scalar_reference(x, y),
                "morton64({x:#010x}, {y:#010x}) diverged from the scalar reference"
            );
        }
        let boundary_codes = [0u64, u64::MAX, 0xAAAA_AAAA_AAAA_AAAA, 0x5555_5555_5555_5555];
        for code in boundary_codes {
            assert_eq!(
                demorton64(code),
                demorton64_scalar_reference(code),
                "demorton64({code:#018x}) diverged from the scalar reference"
            );
        }

        let mut state = 0x2026_0812_u64; // fixed seed — reproducible on failure
        for _ in 0..200_000 {
            let x = splitmix64(&mut state) as u32;
            let y = splitmix64(&mut state) as u32;
            assert_eq!(
                morton64(x, y),
                morton64_scalar_reference(x, y),
                "morton64({x:#010x}, {y:#010x}) diverged from the scalar reference"
            );
            let code = splitmix64(&mut state);
            assert_eq!(
                demorton64(code),
                demorton64_scalar_reference(code),
                "demorton64({code:#018x}) diverged from the scalar reference"
            );
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
