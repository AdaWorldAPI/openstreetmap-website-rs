# Probe methodology — how these measurements are made, and how they failed

The `src/bin/*_probe.rs` binaries answer design questions against real OSM
extracts. This document is the **method**, not the results: what each probe
asks, the rules that emerged while building them, and — at the end — a catalogue
of the errors that produced those rules.

The catalogue is the load-bearing part. Every rule here exists because something
was measured wrong first, and a rule without its incident gets ignored under
pressure.

---

## The probes

| probe | asks |
|---|---|
| `tier_probe` | refcount histogram, anchor shift (P1) |
| `building_probe` | closed-way template census (P4) |
| `wayclass_probe` | what the encoding is asked to carry beyond the drivable road (P5) |
| `areal_probe` | water / Wald / Wiesen: shape, sharing, access, codes, curve family, wire budget (P6, P7) |
| `graph_probe` | **the routing gate** — restriction identity, phantom junctions, false joins, name assembly (P8) |
| `corridor_probe` | the corridor built from connectivity, and whether it rescues the template (P9) |
| `addr_probe` | how an address references its street, and whether the postcode settles it (P10) |
| `housenum_probe` | the address template: decimal positions, parity, turn-around, why a side is empty (P11) |

Run them directly; all take one `.osm.pbf` and need no bake:

```bash
cargo +1.97.1 build --release
./target/release/graph_probe .claude/maps/berlin-latest.osm.pbf
```

**Always run on two extracts.** Berlin is dense, urban and heavily split;
Iceland is rural, long-wayed and thinly built. Several findings only became
readable because the two disagreed — and several *claims* only survived because
they agreed.

---

## The rules

### 1. Pin against something external, never against yourself

A fit that returns its own residual passes every self-consistency check. The
cubic Bézier fit was checked against the **published** ~2.7e-4·r figure for a
90° arc, and that single pin caught three successive errors that all looked
plausible. Self-consistency would have caught none of them.

Where no published constant exists, pin against a *different implementation of
the same question* — `areal_probe` recomputes the building rectilinear share
from independent code and reproduces P4's 94.73 % to the digit. That agreement
is what makes the other rows in the same table comparable rather than merely
adjacent.

### 2. Every test must be able to fail, and you must watch it fail

Write the test, then **break the mechanism** and confirm the intended test goes
red. This is not optional and it is not satisfied by reasoning.

Three vacuous assertions shipped in this session's tests despite the author
knowing this rule. Each was found only by breaking the code and seeing
**nothing** fail. The most serious was on the headline claim of its own change.

### 3. A conservation law beats a behaviour test

Six two-sided falsifiers covered the corridor stitching logic — T-junctions,
interior references, reversal, rings, class declines — and all six passed while
the probe reported **more corridors than ways**, which stitching cannot produce.
The defect sat one line past the tested code.

Behaviour tests check the thing you were thinking about. A conservation law
("the count can only go down", "distance to the curve can never exceed distance
at matched arc length") checks the thing you were not.

### 4. Measure both directions of a proposal

`(street, postcode)` was proposed as a disambiguator. Measuring only "does it
resolve the ambiguous cases" would have missed that a street can **span**
postcodes, in which case the pair names a stretch rather than the street. Both
columns ship.

### 5. Separate missing data from real failure

An address whose postcode matches no candidate might be a genuine mismatch, or
the candidates might have no postcode on record. Reported together, Berlin's
figure was 17.5 %; separated, the real mismatch is **3.5 %** and the rest is
absent information. The first number defamed the idea.

### 6. State the error direction of every approximation

Not "this is approximate" but **which way it is wrong**. The windowed
polyline search can only over-report distance, so a class that passes under it
passes for real. The PCA along-street coordinate misreads a street that doubles
back, so the template share is a floor. The vertex-granular tile split is a
floor for *both* forms equally, so the ratio is unbiased even though the
absolute numbers are not.

An approximation whose direction is unstated is indistinguishable from a bias.

### 7. Name the population a ratio was computed over

`areal_probe` reports a wire ratio of 1.23× for roads; `corridor_probe` reports
0.93× for the same class. Both are right: the first measures only chains of ≥ 4
points (68.2 % of road vertices), the second measures all ways. The excluded
short chains are exactly where the curve form loses.

A ratio without its population is not a measurement.

### 8. The threshold is part of the claim

A run of constant turn means nothing below **17** steps, because the
stride-4-over-17 walk only permutes its residues after 17. Scored at ≥ 3 the
same data reads 17.7 % template coverage; at the real floor it is 0.0 %.

Where a threshold is a choice rather than a derivation, report at two values so
the choice is visible (proximity at 8 m and 25 m; postcode at 100 m and 30 m).

### 9. Tolerance is not the criterion for topology

Everything graded against P2's 1.69 m is **geometry**. Whether a driver is sent
somewhere that does not exist is **topology**, and topology is binary. A 1.7 m
displacement is invisible to a router; a missing turn-off, a phantom junction or
a false connection is not, and none of the three is a tolerance.

`graph_probe` exists because eight probes of geometry never answered that
question.

### 10. Check your own recommendation against its own risk

Three columns concluded "the way is the wrong unit, the corridor is", and the
proposal was to assemble corridors **by name**. `graph_probe`'s fourth column
measured what that would cost: a quarter of Berlin's named groups and half of
Iceland's are not one connected road. "Rosenweg" is 83 separate streets.

The recommendation was refuted by a column written to test it.

---

## The failure catalogue

Every entry is a real error in this session's own work. They are grouped by kind
because the kinds recur and the instances do not.

### Measurement errors — the number was wrong

| what | how it showed | caught by |
|---|---|---|
| Bézier error measured at the point's own parameter, not the curve | 27× too large | the published quarter-circle constant |
| End tangents from the first chord (off by half a sampling step) | 5e-3·r | the same constant |
| Fixed chord-length parametrisation instead of iterated | still 15× off | the same constant |
| Turn-bit form priced with no reconstruction-error column | looked cheapest for every class while being unusable for four | reading the table and asking what it invited |
| Wire ratio quoted over the ≥ 4-point subset without saying so | 1.23× against a true 0.93× | a second probe measuring the whole population |
| Round-trip error read as cell size | f32 loss stated as 1.7 m instead of 2.39 m | a unit test that recomputed the cell |

### Classification errors — the buckets were wrong

| what | how it showed | caught by |
|---|---|---|
| Corridor class looked up by scanning for a matching endpoint | a class reported **more corridors than ways** | an impossible number, not a test |
| "Ordered one side, too few on the other" merged with "…scrambled on the other" | template share understated by 12–15 points | the operator naming the industrial-complex case |
| Run floor set at 3 instead of 17 | 17.7 % coverage claimed where there is 0.0 % | the operator naming the threshold |
| Park conflated with garden | "37 % of parks forbid walking" — an artefact of private gardens | splitting the label and re-reading |

### Test errors — the test could not fail

| what | caught by |
|---|---|
| `results.len() <= 10` after `truncate(10)` | reading it |
| `fair <= drift` while `fair = drift` was the defect under test | breaking the code and watching nothing fail |
| Digit-guard test whose inputs were all declined by the length guard first | breaking the guard and watching nothing fail |
| Zero-shift collapse (0 and 1 sharing a codeword) | breaking the code and watching nothing fail |

### Fixture errors — the input was wrong

| what | why |
|---|---|
| 40-gon asserted "inside the 5° straight bar" | it turns 9° a step |
| Clothoid fixture built with the coarse midpoint rule | shared its quadrature error with the reconstruction, so both agreed on being wrong |
| "Clothoid" with θ_end = 40 rad, R_min 2.5 m | a spiral, not a trassierung — it exercised the chord-vs-arc approximation instead of the fit |
| Slipped points running past the end of the polyline | distance to a curve that is not there is not shape error |
| Ten points asserted to give a 90 % monotone share | N points are N−1 **windows**: 8/9 = 88.9 % |
| Solid rectangles as text glyphs | `filter_blobs` rejects them by density |

### Process errors — the work was not what was reported

| what |
|---|
| A run reported as "in flight" that never started, because a failed test short-circuited the `&&` chain |
| A doc comment and a unit test corrected while the program's own printed output kept the retracted figure |
| A PR body describing the first two commits of four at merge time |
| `printf >>` into a manifest one directory up, because the shell's cwd had reset |

---

## What none of this establishes

The probes measure OSM extracts. They do **not** validate an encoder, a bake, or
a renderer, because none of those was built here. Where a probe reports that a
scheme "works", it means the data has the structure the scheme assumes — not
that an implementation preserves it.

The gate in `graph_probe` is the exception in kind: it reports exposures that any
implementation must handle, and those are yes/no. It still does not check an
implementation, because there is not one to check.
