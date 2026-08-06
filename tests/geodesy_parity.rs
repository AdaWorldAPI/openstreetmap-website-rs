//! Parity of [`geodesy`] against Karney's `geographiclib` — the reference
//! implementation, not a second copy of our own arithmetic.
//!
//! Vectors in `geodesy_vectors.tsv` were produced by the Python
//! `geographiclib` package (v2.1) and are committed, so the test is hermetic:
//! it never reaches for `/tmp` or the network. Two populations:
//!
//! - **`real`** — consecutive node pairs sampled from drivable OSM ways in the
//!   Berlin extract. This is the actual workload; a synthetic sweep alone
//!   would not exercise the length and bearing distribution roads have.
//! - **`synth`** — a grid over latitude (0…80°), length (1 m…1000 km) and
//!   azimuth, to reach the regimes real road segments never visit.
//!
//! The generator lives at `tools/genvec.py`.

use osm_soa_bake::geodesy::{segment_metres, vincenty_metres};

struct Vec6 {
    kind: String,
    lat1: f64,
    lon1: f64,
    lat2: f64,
    lon2: f64,
    karney: f64,
}

fn load() -> Vec<Vec6> {
    let raw = include_str!("geodesy_vectors.tsv");
    raw.lines()
        .skip(1)
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let c: Vec<&str> = l.split('\t').collect();
            Vec6 {
                kind: c[0].to_string(),
                lat1: c[1].parse().unwrap(),
                lon1: c[2].parse().unwrap(),
                lat2: c[3].parse().unwrap(),
                lon2: c[4].parse().unwrap(),
                karney: c[5].parse().unwrap(),
            }
        })
        .collect()
}

fn pct(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[(((sorted.len() - 1) as f64) * p) as usize]
}

#[test]
fn segment_metres_matches_karney_on_real_road_segments() {
    let v: Vec<Vec6> = load().into_iter().filter(|x| x.kind == "real").collect();
    assert!(v.len() > 2_000, "vector file must carry a real population");

    let mut abs: Vec<f64> = Vec::with_capacity(v.len());
    let mut worst = 0.0f64;
    let mut total_ours = 0.0f64;
    let mut total_karney = 0.0f64;
    for x in &v {
        let ours = segment_metres(x.lat1, x.lon1, x.lat2, x.lon2);
        let e = (ours - x.karney).abs();
        abs.push(e);
        worst = worst.max(e);
        total_ours += ours;
        total_karney += x.karney;
    }
    abs.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Per-segment: the local-curvature step must be far below GNSS noise.
    assert!(
        worst < 1e-3,
        "worst per-segment error {worst} m (p50 {}, p95 {}, p99 {})",
        pct(&abs, 0.50),
        pct(&abs, 0.95),
        pct(&abs, 0.99)
    );

    // Accumulated: the number a Fahrtenbuch reports. Relative drift over the
    // whole sampled network must be negligible AND the sum must not be
    // systematically short or long.
    let rel = (total_ours - total_karney).abs() / total_karney;
    assert!(
        rel < 1e-9,
        "accumulated relative drift {rel} over {total_karney} m"
    );
}

#[test]
fn vincenty_matches_karney_across_latitude_and_length() {
    let v: Vec<Vec6> = load().into_iter().filter(|x| x.kind == "synth").collect();
    assert!(v.len() > 100);
    let mut worst = 0.0f64;
    let mut checked = 0;
    for x in &v {
        if let Some(ours) = vincenty_metres(x.lat1, x.lon1, x.lat2, x.lon2) {
            worst = worst.max((ours - x.karney).abs());
            checked += 1;
        }
    }
    // Anti-vacuity: the loop must actually have compared things.
    assert!(checked > 100, "only {checked} vectors converged");
    // Bound provenance, so it is not a number tuned until the test passed:
    //   measured worst  = 7.26e-6 m
    //   Vincenty's published accuracy on WGS84 = 5e-4 m
    // 1e-4 sits ~14x above the measurement and ~5x below the algorithm's own
    // envelope, so it still detects a real regression.
    //
    // A first draft set this at 1e-6 and failed, and the tempting diagnosis
    // was that the error came from the 1e-12 rad convergence cut-off
    // (1e-12 x 6.37e6 m = 6.4 um — almost exactly the observed 7.2 um).
    // FALSIFIED by sweeping the tolerance 1e-12 -> 1e-15: the worst error
    // does not move (7.160 -> 7.261 um). The gap is Vincenty vs Karney as
    // algorithms, and tightening the iteration buys nothing but iterations.
    assert!(
        worst < 1e-4,
        "worst Vincenty error {worst} m over {checked}"
    );
}

#[test]
fn the_local_step_degrades_with_length_which_is_why_vincenty_exists() {
    // Two-sided: the fast path is excellent at road scale and NOT a
    // substitute at continental scale. If this ever passes at 1000 km the
    // fast path has silently become the only implementation anyone needs,
    // and the split should be revisited rather than assumed.
    let v = load();
    let mut short_worst = 0.0f64;
    let mut long_worst = 0.0f64;
    for x in v.iter().filter(|x| x.kind == "synth") {
        let e = (segment_metres(x.lat1, x.lon1, x.lat2, x.lon2) - x.karney).abs();
        if x.karney <= 1_000.0 {
            short_worst = short_worst.max(e);
        }
        if x.karney >= 500_000.0 {
            long_worst = long_worst.max(e);
        }
    }
    assert!(short_worst < 1e-3, "short-range worst {short_worst} m");
    assert!(
        long_worst > 1.0,
        "long-range worst {long_worst} m — expected the local step to degrade"
    );
}
