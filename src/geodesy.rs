//! WGS84 distance on the ellipsoid — the metre, which nothing else in the
//! stack computes.
//!
//! `helix` is a unit-sphere residue codec with no datum; `ndarray`'s
//! `cesium::esri_crs` carries `WGS84_A` but is a stub. Neither returns a
//! length. A routing graph needs edge lengths and a Fahrtenbuch's legally
//! load-bearing figure is kilometres, so this is the gap.
//!
//! # Why not a sphere
//!
//! A mean-radius sphere is biased, not noisy — the error never cancels over a
//! journey. At 52.5°N the meridional radius of curvature is 6,375,743 m and
//! the normal (prime-vertical) radius is 6,391,616 m, against a mean sphere's
//! 6,371,009 m: **+0.074% north-south, +0.32% east-west**. Over 20,000 km
//! that is tens of kilometres, in one direction.
//!
//! # Two functions, one fast and one exact
//!
//! [`segment_metres`] is the hot path: a local-tangent-plane step using the
//! *correct* radii of curvature at the segment's own latitude. Exact to second
//! order in (length / Earth radius), which for road segments — 99% of OSM
//! drivable edges are well under a kilometre — is far below GNSS noise, at the
//! cost of two divisions and a square root.
//!
//! [`vincenty_metres`] is the reference path: Vincenty's inverse solution,
//! iterated to convergence. Correct at any distance, but iterative and
//! undefined for near-antipodal pairs (it returns `None` rather than a wrong
//! number).
//!
//! Both are validated against Karney's `geographiclib` — the reference
//! implementation, not a second copy of my own arithmetic. See
//! `tests/geodesy_parity.rs` and the vectors it consumes.

/// WGS84 semi-major axis (metres).
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS84 semi-minor axis (metres).
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);
/// First eccentricity squared, `e² = f(2−f)`.
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

/// Normalize a longitude difference into `[-180, 180]` so a segment crossing
/// the antimeridian measures its short way round instead of most of a lap.
#[inline]
#[must_use]
pub fn wrap_lon_delta(mut d: f64) -> f64 {
    while d > 180.0 {
        d -= 360.0;
    }
    while d < -180.0 {
        d += 360.0;
    }
    d
}

/// Meridional (north-south) radius of curvature at geodetic latitude `lat_rad`.
#[inline]
#[must_use]
pub fn meridional_radius(lat_rad: f64) -> f64 {
    let s = lat_rad.sin();
    let t = 1.0 - WGS84_E2 * s * s;
    WGS84_A * (1.0 - WGS84_E2) / (t * t.sqrt())
}

/// Normal / prime-vertical (east-west) radius of curvature at `lat_rad`.
#[inline]
#[must_use]
pub fn normal_radius(lat_rad: f64) -> f64 {
    let s = lat_rad.sin();
    WGS84_A / (1.0 - WGS84_E2 * s * s).sqrt()
}

/// Distance in metres between two WGS84 points, for **short** segments.
///
/// Local tangent plane at the segment's mid-latitude, with the true
/// meridional and normal radii of curvature there. This is the hot path: a
/// road segment, a GPS fix pair, one step of a polyline.
#[must_use]
pub fn segment_metres(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let phi_m = (0.5 * (lat1 + lat2)).to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlam = wrap_lon_delta(lon2 - lon1).to_radians();
    let dn = meridional_radius(phi_m) * dphi;
    let de = normal_radius(phi_m) * phi_m.cos() * dlam;
    (dn * dn + de * de).sqrt()
}

/// Vincenty's inverse solution — accurate at any distance.
///
/// Returns `None` if the iteration does not converge, which happens for
/// near-antipodal pairs. A wrong number is never returned in place of a
/// non-answer.
#[must_use]
pub fn vincenty_metres(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> Option<f64> {
    let (a, b, f) = (WGS84_A, WGS84_B, WGS84_F);
    let l = wrap_lon_delta(lon2 - lon1).to_radians();
    let u1 = ((1.0 - f) * lat1.to_radians().tan()).atan();
    let u2 = ((1.0 - f) * lat2.to_radians().tan()).atan();
    let (sin_u1, cos_u1) = (u1.sin(), u1.cos());
    let (sin_u2, cos_u2) = (u2.sin(), u2.cos());

    // All of Vincenty's intermediates are recomputed each iteration and are
    // only read within the iteration that converges, so they are scoped to the
    // loop body rather than hoisted.
    let mut lambda = l;

    for _ in 0..200 {
        let (sin_l, cos_l) = (lambda.sin(), lambda.cos());
        let t1 = cos_u2 * sin_l;
        let t2 = cos_u1 * sin_u2 - sin_u1 * cos_u2 * cos_l;
        let sin_sigma = (t1 * t1 + t2 * t2).sqrt();
        if sin_sigma == 0.0 {
            return Some(0.0); // coincident points
        }
        let cos_sigma = sin_u1 * sin_u2 + cos_u1 * cos_u2 * cos_l;
        let sigma = sin_sigma.atan2(cos_sigma);
        let sin_alpha = cos_u1 * cos_u2 * sin_l / sin_sigma;
        let cos_sq_alpha = 1.0 - sin_alpha * sin_alpha;
        let cos2_sigma_m = if cos_sq_alpha == 0.0 {
            0.0 // equatorial line
        } else {
            cos_sigma - 2.0 * sin_u1 * sin_u2 / cos_sq_alpha
        };
        let c = f / 16.0 * cos_sq_alpha * (4.0 + f * (4.0 - 3.0 * cos_sq_alpha));
        let lambda_prev = lambda;
        lambda = l
            + (1.0 - c)
                * f
                * sin_alpha
                * (sigma
                    + c * sin_sigma
                        * (cos2_sigma_m
                            + c * cos_sigma * (-1.0 + 2.0 * cos2_sigma_m * cos2_sigma_m)));
        if (lambda - lambda_prev).abs() < 1e-12 {
            let u_sq = cos_sq_alpha * (a * a - b * b) / (b * b);
            let big_a =
                1.0 + u_sq / 16384.0 * (4096.0 + u_sq * (-768.0 + u_sq * (320.0 - 175.0 * u_sq)));
            let big_b = u_sq / 1024.0 * (256.0 + u_sq * (-128.0 + u_sq * (74.0 - 47.0 * u_sq)));
            let d_sigma = big_b
                * sin_sigma
                * (cos2_sigma_m
                    + big_b / 4.0
                        * (cos_sigma * (-1.0 + 2.0 * cos2_sigma_m * cos2_sigma_m)
                            - big_b / 6.0
                                * cos2_sigma_m
                                * (-3.0 + 4.0 * sin_sigma * sin_sigma)
                                * (-3.0 + 4.0 * cos2_sigma_m * cos2_sigma_m)));
            return Some(b * big_a * (sigma - d_sigma));
        }
    }
    None
}

/// Cumulative length of a polyline in metres, via [`segment_metres`].
///
/// Summation is in `f64` and in order — a route's total is the number a
/// Fahrtenbuch reports, so the accumulation order is part of the result.
#[must_use]
pub fn polyline_metres(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|w| segment_metres(w[0].0, w[0].1, w[1].0, w[1].1))
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radii_match_the_documented_values_at_berlin() {
        let phi = 52.520008_f64.to_radians();
        // The two numbers the module doc quotes, and the reason a sphere is
        // biased rather than merely imprecise.
        assert!((meridional_radius(phi) - 6_375_743.0).abs() < 50.0);
        assert!((normal_radius(phi) - 6_391_616.0).abs() < 50.0);
        assert!(meridional_radius(phi) < normal_radius(phi));
    }

    #[test]
    fn a_mean_sphere_is_biased_not_noisy() {
        // Falsifier for "just use a sphere": a purely east-west kilometre in
        // Berlin measures SHORT on a mean sphere, every time, by ~0.3%.
        const R: f64 = 6_371_008.8;
        let (lat, lon1, lon2) = (52.520008, 13.40, 13.42);
        let ell = segment_metres(lat, lon1, lat, lon2);
        let sph = R * lat.to_radians().cos() * (lon2 - lon1).to_radians();
        let rel = (ell - sph) / ell;
        assert!(
            rel > 0.002,
            "expected sphere to under-measure E-W, got {rel}"
        );
        assert!(rel < 0.004);
    }

    #[test]
    fn zero_length_is_zero_and_symmetric() {
        assert_eq!(segment_metres(52.5, 13.4, 52.5, 13.4), 0.0);
        let a = segment_metres(52.5, 13.4, 52.6, 13.5);
        let b = segment_metres(52.6, 13.5, 52.5, 13.4);
        assert!((a - b).abs() < 1e-9, "distance must be symmetric");
    }

    #[test]
    fn antimeridian_takes_the_short_way() {
        // Without the longitude wrap this measures ~40,000 km instead of ~2 km.
        let d = segment_metres(60.0, 179.99, 60.0, -179.99);
        assert!(d < 3_000.0, "crossing 180° must be short, got {d} m");
    }

    #[test]
    fn polyline_sums_its_segments() {
        let pts = [(52.50, 13.40), (52.51, 13.41), (52.52, 13.42)];
        let total = polyline_metres(&pts);
        let manual =
            segment_metres(52.50, 13.40, 52.51, 13.41) + segment_metres(52.51, 13.41, 52.52, 13.42);
        assert!((total - manual).abs() < 1e-9);
        assert!(
            polyline_metres(&pts[..1]) == 0.0,
            "a single point has no length"
        );
    }

    #[test]
    fn vincenty_declines_rather_than_guessing_near_antipodal() {
        // The can-stay-silent half: a near-antipodal pair must return None,
        // not a plausible-looking wrong number.
        assert!(vincenty_metres(0.0, 0.0, 0.5, 179.7).is_none());
        // ...paired with a case that DOES converge, so the guard discriminates.
        assert!(vincenty_metres(52.5, 13.4, 48.1, 11.6).is_some());
    }
}
