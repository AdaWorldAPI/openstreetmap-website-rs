//! Curve fitting and chain-encoding primitives, shared by the probes.
//!
//! Extracted from `areal_probe` when a second probe needed them. The alternative
//! was a third copy of the same 300 lines — after `Frame` had already been
//! copied twice — and a copy is where two probes start disagreeing about what
//! they measured.
//!
//! Everything here is **pure geometry over a metre frame**: no OSM types, no
//! tags, no tolerance policy. The thresholds live with the callers, because the
//! bar a fit is graded against is a decision and not a property of the fit.
//!
//! Two constants DO live here, because they are properties of the
//! reconstruction rather than of any caller's policy.

/// Sub-steps per original segment when a clothoid is re-sampled as a polyline.
pub const CLOTHOID_SUB: usize = 16;

/// How many original segments either side of a point's own arc position the
/// search looks, in the re-sampled polyline. Searching a window can only return
/// a distance >= the true minimum, so the result is an upper bound.
pub const CLOTHOID_WINDOW: usize = 4;

/// Bytes an LEB128 varint of `v` occupies.
pub fn varint_len(v: u64) -> usize {
    let mut n = 1;
    let mut v = v >> 7;
    while v > 0 {
        n += 1;
        v >>= 7;
    }
    n
}
/// Zig-zag, so a signed delta costs by magnitude rather than by sign.
pub fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}
/// Max deviation, metres, of a **single cubic Bézier** fitted through `pts`.
///
/// Schneider's classic fit: endpoints are pinned, tangents come from the data,
/// and the two interior control points are the least-squares solution along
/// those tangents under chord-length parametrisation. No reparametrisation
/// iteration — one shot, which is what a bake would afford.
///
/// The deviation is the **true distance from each point to the curve**, found by
/// a coarse scan plus ternary refinement — see the comment at the measurement.
pub fn fit_cubic_bezier(pts: &[(f64, f64)]) -> Option<f64> {
    fit_cubic_bezier_at(pts).map(|(e, _)| e)
}
/// As [`fit_cubic_bezier`], plus the index of the worst-fitting point — which is
/// where an adaptive subdivision must split, per Schneider.
pub fn fit_cubic_bezier_at(pts: &[(f64, f64)]) -> Option<(f64, usize)> {
    let n = pts.len();
    if n < 4 {
        return None;
    }
    let p0 = pts[0];
    let p3 = pts[n - 1];
    let unit = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt();
        if l <= 0.0 {
            None
        } else {
            Some((dx / l, dy / l))
        }
    };
    // End tangents by the SECOND-ORDER one-sided difference, not the first
    // chord. A chord is off by half the sampling step (0.7 deg on a 64-sample
    // quarter circle) and over a ~0.55r control arm that alone lands the
    // control point ~5e-3*r away. `-3p0 + 4p1 - p2` is O(h^2) instead of O(h).
    let dir = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        let v = (4.0 * b.0 - c.0 - 3.0 * a.0, 4.0 * b.1 - c.1 - 3.0 * a.1);
        let l = (v.0 * v.0 + v.1 * v.1).sqrt();
        if l <= 0.0 {
            None
        } else {
            Some((v.0 / l, v.1 / l))
        }
    };
    let t1 = dir(pts[0], pts[1], pts[2]).or_else(|| unit(pts[0], pts[1]))?;
    let t2 = dir(pts[n - 1], pts[n - 2], pts[n - 3]).or_else(|| unit(pts[n - 1], pts[n - 2]))?;

    // Initial parametrisation: chord length.
    let mut u = Vec::with_capacity(n);
    let mut acc = 0.0;
    u.push(0.0);
    for w in pts.windows(2) {
        acc += ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
        u.push(acc);
    }
    if acc <= 0.0 {
        return None;
    }
    for v in &mut u {
        *v /= acc;
    }

    let eval = |p1: (f64, f64), p2: (f64, f64), t: f64| {
        let m = 1.0 - t;
        (
            p0.0 * m * m * m + 3.0 * p1.0 * t * m * m + 3.0 * p2.0 * t * t * m + p3.0 * t * t * t,
            p0.1 * m * m * m + 3.0 * p1.1 * t * m * m + 3.0 * p2.1 * t * t * m + p3.1 * t * t * t,
        )
    };
    // Closest point on the curve: coarse scan, then ternary refine. Used BOTH
    // for the final error and to reparametrise between refits — a fixed
    // chord-length parametrisation is not the best-fitting one, which is why
    // Schneider's algorithm iterates here and why a single pass sat ~15x above
    // the published quarter-circle figure.
    let project = |p1: (f64, f64), p2: (f64, f64), pt: (f64, f64)| {
        const SCAN: usize = 64;
        let d2 = |t: f64| {
            let (bx, by) = eval(p1, p2, t);
            (pt.0 - bx).powi(2) + (pt.1 - by).powi(2)
        };
        let (mut best, mut bd) = (0usize, f64::MAX);
        for k in 0..=SCAN {
            let d = d2(k as f64 / SCAN as f64);
            if d < bd {
                bd = d;
                best = k;
            }
        }
        let (mut lo, mut hi) = (
            best.saturating_sub(1) as f64 / SCAN as f64,
            (best + 1).min(SCAN) as f64 / SCAN as f64,
        );
        for _ in 0..60 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            if d2(m1) < d2(m2) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        let t = (lo + hi) / 2.0;
        (t, d2(t).sqrt())
    };

    let mut p1 = p0;
    let mut p2 = p3;
    for pass in 0..6 {
        // Least squares for the two tangent magnitudes (Graphics Gems normal eqs).
        let (mut c00, mut c01, mut c11, mut x0, mut x1) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (i, &pt) in pts.iter().enumerate() {
            let t = u[i];
            let m = 1.0 - t;
            let (b0, b1, b2, b3) = (m * m * m, 3.0 * t * m * m, 3.0 * t * t * m, t * t * t);
            let a0 = (t1.0 * b1, t1.1 * b1);
            let a1 = (t2.0 * b2, t2.1 * b2);
            c00 += a0.0 * a0.0 + a0.1 * a0.1;
            c01 += a0.0 * a1.0 + a0.1 * a1.1;
            c11 += a1.0 * a1.0 + a1.1 * a1.1;
            let tmp = (
                pt.0 - (p0.0 * (b0 + b1) + p3.0 * (b2 + b3)),
                pt.1 - (p0.1 * (b0 + b1) + p3.1 * (b2 + b3)),
            );
            x0 += a0.0 * tmp.0 + a0.1 * tmp.1;
            x1 += a1.0 * tmp.0 + a1.1 * tmp.1;
        }
        let det = c00 * c11 - c01 * c01;
        let (a, b) = if det.abs() > 1e-12 {
            ((x0 * c11 - x1 * c01) / det, (c00 * x1 - c01 * x0) / det)
        } else {
            // Degenerate normal equations: fall back to the standard
            // third-of-chord heuristic rather than fabricating a zero error.
            let d = ((p3.0 - p0.0).powi(2) + (p3.1 - p0.1).powi(2)).sqrt() / 3.0;
            (d, d)
        };
        p1 = (p0.0 + t1.0 * a, p0.1 + t1.1 * a);
        p2 = (p3.0 + t2.0 * b, p3.1 + t2.1 * b);
        if pass == 5 {
            break;
        }
        for (i, &pt) in pts.iter().enumerate() {
            u[i] = project(p1, p2, pt).0;
        }
    }

    let mut worst = (0.0f64, 0usize);
    for (i, &pt) in pts.iter().enumerate() {
        let d = project(p1, p2, pt).1;
        if d > worst.0 {
            worst = (d, i);
        }
    }
    Some(worst)
}
/// Cubic segments needed to carry `pts` within `tol` metres, by adaptive
/// subdivision at the worst-fitting point.
///
/// This is the RENDERING question, which is not the fitting question. A client
/// does not care whether one cubic could carry the whole way — it cares how many
/// control points cross the wire and reach the vertex shader. WebGL has no
/// tessellation stage (that is WebGPU / GL4), so a curve is either expanded on
/// the CPU or evaluated per-vertex from control points uploaded once; either
/// way the win is bandwidth and vertex count, never rasterisation.
pub fn bezier_segments(pts: &[(f64, f64)], tol: f64) -> usize {
    // Fewer than 4 points cannot constrain a cubic; such a piece still costs one
    // segment's worth of control points, so it counts as one rather than zero.
    if pts.len() < 4 {
        return 1;
    }
    match fit_cubic_bezier_at(pts) {
        Some((e, _)) if e <= tol => 1,
        Some((_, i)) => {
            // Split at the worst point, keeping it in both halves so the pieces
            // share an endpoint — a path, not a scatter of loose curves.
            let k = i.clamp(1, pts.len() - 2);
            bezier_segments(&pts[..=k], tol) + bezier_segments(&pts[k..], tol)
        }
        None => 1,
    }
}
/// Max deviation, metres, of a **clothoid** — curvature linear in arc length.
///
/// This is the storage form the shape hypothesis implies: `theta(s) = theta0 +
/// k0*s + k1*s^2/2`, so a segment is three numbers (`k0`, `k1`, `L`) and no
/// control points at all. Fitted by least squares on heading against arc length,
/// then integrated back to positions and compared with the real ones — the fit
/// is in heading, the verdict is in metres, because metres are what P2 bounds.
pub fn fit_clothoid(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    let n = pts.len();
    if n < 4 {
        return None;
    }
    let mut seg = Vec::with_capacity(n - 1);
    let mut head = Vec::with_capacity(n - 1);
    for w in pts.windows(2) {
        let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
        let l = (dx * dx + dy * dy).sqrt();
        if l <= 0.0 {
            return None;
        }
        seg.push(l);
        head.push(dy.atan2(dx));
    }
    // Unwrap the heading so the regression sees a continuous function.
    for i in 1..head.len() {
        let d = wrap_pi(head[i] - head[i - 1]);
        head[i] = head[i - 1] + d;
    }
    // Midpoint arc length of each segment.
    let mut mid = Vec::with_capacity(seg.len());
    let mut acc = 0.0;
    for &l in &seg {
        mid.push(acc + l / 2.0);
        acc += l;
    }
    // Least squares theta = a + b*s + c*s^2 (normal equations, 3x3).
    let m = mid.len() as f64;
    let (mut s1, mut s2, mut s3, mut s4) = (0.0, 0.0, 0.0, 0.0);
    let (mut ty, mut tsy, mut ts2y) = (0.0, 0.0, 0.0);
    for (i, &sv) in mid.iter().enumerate() {
        let (v2, v3, v4) = (sv * sv, sv * sv * sv, sv * sv * sv * sv);
        s1 += sv;
        s2 += v2;
        s3 += v3;
        s4 += v4;
        ty += head[i];
        tsy += sv * head[i];
        ts2y += v2 * head[i];
    }
    let a = [[m, s1, s2], [s1, s2, s3], [s2, s3, s4]];
    let rhs = [ty, tsy, ts2y];
    let sol = solve3(a, rhs)?;
    let (t0, k0, half_k1) = (sol[0], sol[1], sol[2]);

    // Integrate the fitted heading back to positions, using the REAL segment
    // lengths: the claim under test is the shape, not the sampling.
    let (mut x, mut y) = pts[0];
    // Two numbers from ONE fit, because they answer different questions and
    // conflating them is what made the earlier column incomparable:
    //
    //   drift — position at MATCHED ARC LENGTH against the original vertex.
    //           The honest cost of a chain form: quantisation and fit error
    //           integrate along the way, so this carries accumulated slip.
    //   fair  — distance to the reconstructed CURVE, the same metric the Bézier
    //           column uses. Slipping along the curve costs nothing here, so it
    //           isolates shape error from drift.
    //
    // A large gap between them means the shape is right and the travel is off.
    let mut drift: f64 = 0.0;
    let mut poly: Vec<(f64, f64)> = Vec::with_capacity(seg.len() * CLOTHOID_SUB + 1);
    poly.push((x, y));
    let mut acc = 0.0;
    for (i, &l) in seg.iter().enumerate() {
        // Sub-step the integration so the re-sample is a curve, not a chord.
        let h = l / CLOTHOID_SUB as f64;
        for k in 0..CLOTHOID_SUB {
            let sm = acc + (k as f64 + 0.5) * h;
            let th = t0 + k0 * sm + half_k1 * sm * sm;
            x += h * th.cos();
            y += h * th.sin();
            poly.push((x, y));
        }
        acc += l;
        let (tx, tyy) = pts[i + 1];
        drift = drift.max(((x - tx).powi(2) + (y - tyy).powi(2)).sqrt());
    }
    let fair = max_dist_to_polyline(pts, &poly, CLOTHOID_SUB);
    Some((drift, fair))
}
/// Greatest distance from any point in `pts` to the polyline `poly`, metres.
///
/// **This is the metric the Bézier column already uses**, applied to a
/// reconstruction. Point-to-SEGMENT, not point-to-vertex: measuring to vertices
/// would charge the reconstruction for the sampling density instead of the
/// shape. `stride` maps an original index onto its position in `poly`.
pub fn max_dist_to_polyline(pts: &[(f64, f64)], poly: &[(f64, f64)], stride: usize) -> f64 {
    if poly.len() < 2 {
        return f64::INFINITY;
    }
    let span = CLOTHOID_WINDOW * stride;
    let mut worst: f64 = 0.0;
    for (i, &p) in pts.iter().enumerate() {
        let centre = i * stride;
        let lo = centre.saturating_sub(span);
        let hi = (centre + span).min(poly.len() - 1);
        let mut best = f64::INFINITY;
        for w in poly[lo..=hi].windows(2) {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
            best = best.min(((p.0 - qx).powi(2) + (p.1 - qy).powi(2)).sqrt());
        }
        worst = worst.max(best);
    }
    worst
}
/// Max radial deviation, metres, of an algebraic (Kasa) **circle** fit.
///
/// Only meaningful where the shape really is a conic — which is why it is
/// reported for `junction=roundabout` rather than for everything.
pub fn fit_circle(pts: &[(f64, f64)]) -> Option<f64> {
    let n = pts.len();
    if n < 4 {
        return None;
    }
    let (mut sx, mut sy) = (0.0, 0.0);
    for &(x, y) in pts {
        sx += x;
        sy += y;
    }
    let (cx0, cy0) = (sx / n as f64, sy / n as f64);
    let (mut suu, mut suv, mut svv, mut suuu, mut svvv, mut suvv, mut svuu) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for &(x, y) in pts {
        let (u, v) = (x - cx0, y - cy0);
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }
    let det = suu * svv - suv * suv;
    if det.abs() < 1e-12 {
        return None;
    }
    let b1 = 0.5 * (suuu + suvv);
    let b2 = 0.5 * (svvv + svuu);
    let uc = (b1 * svv - b2 * suv) / det;
    let vc = (suu * b2 - suv * b1) / det;
    let (cx, cy) = (cx0 + uc, cy0 + vc);
    let r = pts
        .iter()
        .map(|&(x, y)| ((x - cx).powi(2) + (y - cy).powi(2)).sqrt())
        .sum::<f64>()
        / n as f64;
    Some(
        pts.iter()
            .map(|&(x, y)| (((x - cx).powi(2) + (y - cy).powi(2)).sqrt() - r).abs())
            .fold(0.0f64, f64::max),
    )
}
/// Gaussian elimination with partial pivoting on a 3x3.
fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for i in 0..3 {
        let mut piv = i;
        for r in i + 1..3 {
            if a[r][i].abs() > a[piv][i].abs() {
                piv = r;
            }
        }
        if a[piv][i].abs() < 1e-12 {
            return None;
        }
        a.swap(i, piv);
        b.swap(i, piv);
        for r in i + 1..3 {
            let f = a[r][i] / a[i][i];
            let row = a[i];
            for (c, v) in a[r].iter_mut().enumerate().skip(i) {
                *v -= f * row[c];
            }
            b[r] -= f * b[i];
        }
    }
    let mut x = [0.0f64; 3];
    for i in (0..3).rev() {
        let mut v = b[i];
        for c in i + 1..3 {
            v -= a[i][c] * x[c];
        }
        x[i] = v / a[i][i];
    }
    Some(x)
}
/// Wrap to `(-π, π]`.
pub fn wrap_pi(mut a: f64) -> f64 {
    while a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    while a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

/// Is `p` strictly inside the closed ring `ring`? Crossing-number ray cast.
///
/// The half-open rule on the y comparison (`>` on one end, `<=` on the other) is
/// what makes a vertex exactly at the ray's height count once rather than twice
/// — without it a point level with a vertex flips to the wrong answer.
#[must_use]
pub fn point_in_ring(p: (f64, f64), ring: &[(f64, f64)]) -> bool {
    let mut inside = false;
    for w in ring.windows(2) {
        let (a, b) = (w[0], w[1]);
        if (a.1 > p.1) != (b.1 > p.1) {
            let t = (p.1 - a.1) / (b.1 - a.1);
            if p.0 < a.0 + t * (b.0 - a.0) {
                inside = !inside;
            }
        }
    }
    inside
}
