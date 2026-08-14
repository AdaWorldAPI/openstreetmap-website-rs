//! Per-edge exit heading + bending, quantized — the plain alternative to
//! `helix::Signed360` for a junction's edges.
//!
//! `Signed360` is the φ-spiral / Zeckendorf place-value residue encoder this
//! workspace uses for VSA fingerprints (golden azimuth, `RollingFloor`-keyed
//! construction). Reusing it for "which way does this street leave the
//! junction" would need a bearing→place-value bridge that does not exist —
//! guessing at one risks a silently wrong heading, which is worse than not
//! having one. This module is the honest, small alternative: **exact angle is
//! false precision for routing** (nobody routes on a fraction of a degree),
//! so direction is quantized to 16 compass points, and bending is the worst
//! perpendicular deviation from the edge's own chord — a plain, cheap
//! Bézier-sagitta-style approximation — rather than an unavailable DIN
//! curve-radius classification table.
//!
//! One byte per edge: `[bending:4][direction:4]`. 8 edges fit in 8 bytes —
//! half of one 16-byte value slot, with room left over.

/// 16 compass points, 22.5° apart. 4 bits.
pub const DIRECTIONS: u8 = 16;

/// Bearing in degrees `[0, 360)` -> nearest of 16 compass points, `[0, 16)`.
///
/// Rounds to nearest, wrapping 360 back to 0 rather than overflowing to 16 —
/// the boundary a naive `/ 22.5` floor gets wrong.
#[must_use]
pub fn exit_direction(bearing_deg: f64) -> u8 {
    let step = 360.0 / f64::from(DIRECTIONS);
    (((bearing_deg.rem_euclid(360.0)) / step).round() as u8) % DIRECTIONS
}

/// Compass-point bearing back to its representative angle, degrees.
#[must_use]
pub fn direction_to_bearing(dir: u8) -> f64 {
    f64::from(dir % DIRECTIONS) * (360.0 / f64::from(DIRECTIONS))
}

/// Bending class, 4 bits (0..16): the worst perpendicular deviation of the
/// edge's own points from its CHORD (the straight line junction-to-junction),
/// as a fraction of chord length — dimensionless, so it needs no CRS and
/// works at any zoom.
///
/// This is deliberately NOT [`crate::curve::fit_cubic_bezier`], which answers
/// a different question ("how well does a smooth curve fit these points",
/// i.e. is the path noisy or clean) — a sharp U-turn can fit a wild S-curve
/// closely and score as barely-deviating, which a first version of this
/// function did and a test caught. Bending needs "how far does the road
/// wander from straight", which is chord distance, not curve-fit residual.
///
/// Buckets widen geometrically (doubling per step) because a road's bend is
/// naturally log-distributed: the overwhelming majority are near-straight,
/// and a linear scale would waste most of its range on that majority. Class 0
/// is "straight enough to ignore" (< 0.1% of chord); class 15 saturates
/// rather than overflowing.
#[must_use]
pub fn bending_class(pts: &[(f64, f64)]) -> u8 {
    if pts.len() < 2 {
        return 0;
    }
    let (x0, y0) = pts[0];
    let (x1, y1) = pts[pts.len() - 1];
    let (dx, dy) = (x1 - x0, y1 - y0);
    let chord = (dx * dx + dy * dy).sqrt();
    if chord <= 0.0 {
        return 0; // a closed or degenerate loop has no chord to bend against
    }
    // Perpendicular distance of point p from the infinite line through the
    // chord: |cross(p - p0, chord)| / |chord| — the standard point-to-line
    // formula, taking the WORST point rather than an average so one sharp
    // kink is not diluted by an otherwise-straight run.
    let max_dev = pts
        .iter()
        .map(|&(px, py)| ((px - x0) * dy - (py - y0) * dx).abs() / chord)
        .fold(0.0_f64, f64::max);
    let ratio = max_dev / chord;
    if ratio <= 0.001 {
        return 0;
    }
    // class = floor(log2(ratio / 0.001)), clamped to [0, 15] — doubling steps
    // from the 0.1% floor.
    let class = (ratio / 0.001).log2().floor().max(0.0) as u8;
    class.min(15)
}

/// Pack one edge's `(direction, bending)` into a byte.
#[must_use]
pub fn pack_edge(direction: u8, bending: u8) -> u8 {
    (bending & 0x0F) << 4 | (direction & 0x0F)
}

/// Unpack a byte back into `(direction, bending)`.
#[must_use]
pub fn unpack_edge(b: u8) -> (u8, u8) {
    (b & 0x0F, (b >> 4) & 0x0F)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direction_round_trips_at_the_16_representative_bearings() {
        for d in 0..DIRECTIONS {
            let bearing = direction_to_bearing(d);
            assert_eq!(exit_direction(bearing), d, "direction {d} at {bearing}°");
        }
    }

    #[test]
    fn direction_wraps_360_back_to_0_not_16() {
        assert_eq!(exit_direction(359.99), 0);
        assert_eq!(exit_direction(360.0), 0);
        assert_eq!(exit_direction(0.0), 0);
    }

    #[test]
    fn direction_handles_negative_and_over_360_bearings() {
        // rem_euclid, not %, so a caller that computed atan2 in [-180,180) or
        // summed past 360 still lands correctly rather than panicking or
        // silently going negative.
        assert_eq!(exit_direction(-1.0), exit_direction(359.0));
        assert_eq!(exit_direction(361.0), exit_direction(1.0));
    }

    #[test]
    fn cardinal_directions_land_where_a_compass_reader_expects() {
        assert_eq!(exit_direction(0.0), 0); // N
        assert_eq!(exit_direction(90.0), 4); // E
        assert_eq!(exit_direction(180.0), 8); // S
        assert_eq!(exit_direction(270.0), 12); // W
    }

    #[test]
    fn straight_segment_is_bending_class_zero() {
        let pts: Vec<(f64, f64)> = (0..20).map(|i| (f64::from(i) * 10.0, 0.0)).collect();
        assert_eq!(bending_class(&pts), 0);
    }

    #[test]
    fn a_hairpin_saturates_rather_than_panicking_or_overflowing() {
        // A near-U-turn: far out and almost back, chord tiny relative to path.
        let pts = vec![(0.0, 0.0), (0.0, 100.0), (1.0, 100.0), (1.0, 0.5)];
        let class = bending_class(&pts);
        assert!(class <= 15, "class {class} must fit 4 bits");
        assert!(
            class >= 10,
            "a near-U-turn should read as strongly bent, got {class}"
        );
    }

    #[test]
    fn bending_increases_monotonically_with_deviation_not_just_in_the_middle() {
        // Same chord (0,0)->(100,0), growing bulge — the class must not
        // decrease as the road bends harder.
        let mut prev = 0u8;
        for bulge in [0.0, 1.0, 5.0, 20.0, 50.0, 90.0] {
            let pts = vec![(0.0, 0.0), (50.0, bulge), (100.0, 0.0)];
            let class = bending_class(&pts);
            assert!(
                class >= prev,
                "bulge {bulge} gave class {class}, less than prior {prev}"
            );
            prev = class;
        }
    }

    #[test]
    fn empty_or_single_point_is_bending_class_zero_not_a_panic() {
        assert_eq!(bending_class(&[]), 0);
        assert_eq!(bending_class(&[(1.0, 1.0)]), 0);
    }

    #[test]
    fn pack_unpack_round_trips_every_nibble_combination() {
        for direction in 0..16u8 {
            for bending in 0..16u8 {
                let packed = pack_edge(direction, bending);
                assert_eq!(unpack_edge(packed), (direction, bending));
            }
        }
    }

    #[test]
    fn eight_edges_fit_in_eight_bytes_half_a_value_slot() {
        // The claim this module exists to make cheap: EDGE_SLOTS edges of
        // (direction, bending) cost half of one 16-byte value slot, against
        // Signed360's 12 bytes for just TWO headings.
        assert_eq!(
            std::mem::size_of::<u8>() * usize::from(crate::street::EDGE_SLOTS),
            8
        );
    }
}
