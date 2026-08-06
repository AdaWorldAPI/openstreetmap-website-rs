//! Print the measured parity of `geodesy` against Karney, so the numbers the
//! tests assert on are visible rather than implied.
//!
//! `cargo run --release --example geodesy_report`

use osm_soa_bake::geodesy::{segment_metres, vincenty_metres};

fn pct(s: &[f64], p: f64) -> f64 {
    if s.is_empty() {
        0.0
    } else {
        s[(((s.len() - 1) as f64) * p) as usize]
    }
}

fn main() {
    let raw = include_str!("../tests/geodesy_vectors.tsv");
    let rows: Vec<Vec<&str>> = raw
        .lines()
        .skip(1)
        .filter(|l| !l.trim().is_empty())
        .map(|l| l.split('\t').collect())
        .collect();

    for kind in ["real", "synth"] {
        let mut seg: Vec<f64> = Vec::new();
        let mut vin: Vec<f64> = Vec::new();
        let (mut ours_tot, mut ref_tot, mut len_min, mut len_max) = (0.0, 0.0, f64::MAX, 0.0f64);
        for r in rows.iter().filter(|r| r[0] == kind) {
            let (a, b, c, d): (f64, f64, f64, f64) = (
                r[1].parse().unwrap(),
                r[2].parse().unwrap(),
                r[3].parse().unwrap(),
                r[4].parse().unwrap(),
            );
            let k: f64 = r[5].parse().unwrap();
            let s = segment_metres(a, b, c, d);
            seg.push((s - k).abs());
            if let Some(v) = vincenty_metres(a, b, c, d) {
                vin.push((v - k).abs());
            }
            ours_tot += s;
            ref_tot += k;
            len_min = len_min.min(k);
            len_max = len_max.max(k);
        }
        seg.sort_by(|a, b| a.partial_cmp(b).unwrap());
        vin.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!(
            "\n=== {kind} — {} vectors, length {:.2} m … {:.0} m ===",
            seg.len(),
            len_min,
            len_max
        );
        println!(
            "segment_metres  vs Karney   p50 {:.3e}  p95 {:.3e}  p99 {:.3e}  max {:.3e}  m",
            pct(&seg, 0.5),
            pct(&seg, 0.95),
            pct(&seg, 0.99),
            seg.last().copied().unwrap_or(0.0)
        );
        println!("vincenty_metres vs Karney   p50 {:.3e}  p95 {:.3e}  p99 {:.3e}  max {:.3e}  m  ({} converged)",
                 pct(&vin,0.5), pct(&vin,0.95), pct(&vin,0.99), vin.last().copied().unwrap_or(0.0), vin.len());
        println!(
            "accumulated  ours {:.6} m   karney {:.6} m   rel drift {:.3e}",
            ours_tot,
            ref_tot,
            (ours_tot - ref_tot).abs() / ref_tot
        );
    }

    // What a mean sphere would have cost on the same real segments.
    const R: f64 = 6_371_008.8;
    let (mut ell, mut sph) = (0.0f64, 0.0f64);
    for r in rows.iter().filter(|r| r[0] == "real") {
        let (a, b, c, d): (f64, f64, f64, f64) = (
            r[1].parse().unwrap(),
            r[2].parse().unwrap(),
            r[3].parse().unwrap(),
            r[4].parse().unwrap(),
        );
        ell += segment_metres(a, b, c, d);
        let (p1, p2) = (a.to_radians(), c.to_radians());
        let (dp, dl) = (p2 - p1, (d - b).to_radians());
        let h = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
        sph += 2.0 * R * h.sqrt().asin();
    }
    println!("\n=== the cost of a mean sphere, on the same real road segments ===");
    println!(
        "WGS84 {:.3} m   haversine {:.3} m   sphere is {:+.4}% ({:+.3} m over {:.0} m)",
        ell,
        sph,
        100.0 * (sph - ell) / ell,
        sph - ell,
        ell
    );
    println!(
        "extrapolated to 20,000 km driven: {:+.1} km",
        20_000.0 * (sph - ell) / ell
    );
}
