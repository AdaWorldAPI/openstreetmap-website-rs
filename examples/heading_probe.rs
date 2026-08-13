//! Proves `heading::{exit_direction, bending_class}` against REAL chain
//! geometry, not synthetic fixtures — the same discipline the unit tests
//! couldn't provide alone (a hand-built fixture can't be wrong about what
//! real OSM ways actually look like).
//!
//! ```text
//! heading_probe <region.chains>
//! ```

use osm_soa_bake::chains::Chains;
use osm_soa_bake::heading::{bending_class, direction_to_bearing, exit_direction};

fn bearing_deg(a: (f64, f64), b: (f64, f64)) -> f64 {
    // z32 TileXy is a uniform grid (no lat-dependent scale distortion at this
    // scale), so atan2 on raw coordinates gives a real compass bearing.
    // Screen/tile convention: y grows downward, so north is -y.
    (b.0 - a.0).atan2(-(b.1 - a.1)).to_degrees().rem_euclid(360.0)
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: heading_probe <chains>");
    let bytes = std::fs::read(&path).expect("read chains");
    let chains = Chains::from_bytes(bytes).expect("parse chains");

    println!("chain records: {}", chains.len());

    let (mut n, mut dir_hist, mut bend_hist) = (0usize, [0u32; 16], [0u32; 16]);
    let mut sample_lines = Vec::new();

    // Ordinals are sparse (not every index is populated); walk a wide range
    // and take what resolves, same pattern `parity` uses.
    for ordinal in 0..8_000_000u32 {
        let Ok(Some(cells)) = chains.get(ordinal) else { continue };
        if cells.len() < 3 {
            continue;
        }
        let pts: Vec<(f64, f64)> = cells.iter().map(|c| (f64::from(c.x), f64::from(c.y_xyz))).collect();
        let bearing = bearing_deg(pts[0], pts[1]);
        let dir = exit_direction(bearing);
        let bend = bending_class(&pts);
        dir_hist[dir as usize] += 1;
        bend_hist[bend as usize] += 1;
        n += 1;
        if sample_lines.len() < 5 {
            sample_lines.push(format!(
                "  ordinal={ordinal} pts={} bearing={bearing:.1}° dir={dir} ({:.1}°) bend_class={bend}",
                pts.len(),
                direction_to_bearing(dir)
            ));
        }
        if n >= 20_000 {
            break;
        }
    }

    println!("real ways measured: {n}");
    println!("\nfirst 5:");
    for l in &sample_lines {
        println!("{l}");
    }
    println!("\ndirection histogram (16 compass points):");
    for (d, c) in dir_hist.iter().enumerate() {
        if *c > 0 {
            println!("  dir {d:>2} ({:>5.1}°): {c}", direction_to_bearing(d as u8));
        }
    }
    println!("\nbending histogram (0=straight .. 15=hairpin):");
    for (b, c) in bend_hist.iter().enumerate() {
        if *c > 0 {
            println!("  class {b:>2}: {c}");
        }
    }
    let straight_pct = 100.0 * f64::from(bend_hist[0]) / n as f64;
    println!("\nstraight (class 0): {straight_pct:.1}% — sanity check: most real roads should be near-straight");
}
