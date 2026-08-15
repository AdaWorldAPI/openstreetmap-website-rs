#!/usr/bin/env bash
# Bake job: fetch a .osm.pbf, bake it to a V3 slab, publish to the object store.
#
# Exits non-zero on any failure. A partially-written slab is never published:
# the upload step runs only after `bake` returns 0, and the checksums are
# computed from the files that were actually written, not from the bake's own
# reported digest.
set -euo pipefail

WORK="${RAILWAY_VOL:-/tmp}"
if [ ! -d "$WORK" ] || [ ! -w "$WORK" ]; then
    # A volume that was declared but not attached is the shape that silently
    # fills a container's writable layer. Say so rather than degrading quietly.
    echo "note: '$WORK' is not a writable directory; falling back to /tmp" >&2
    WORK=/tmp
fi

OUT="$WORK/osm-bake-out"
mkdir -p "$OUT"

NAME="${BAKE_NAME:-berlin}"
PBF="$OUT/$NAME.osm.pbf"
SLAB="$OUT/$NAME.soa"

echo "==> workspace: $OUT"
df -h "$WORK" | tail -1

# ── 1. Source PBF ────────────────────────────────────────────────────────────
# THIS JOB DOES NOT DOWNLOAD. The PBF must already be on the volume.
#
# It used to `curl` the file from a third-party mirror on every run that found
# no cached copy — and because this image exits when it finishes, anything that
# runs it as a long-lived service restarts it forever, turning that into a
# repeated automated pull from one shared egress IP. That is exactly the traffic
# such a mirror blocks, and the bytes are not free either.
#
# There is nothing to fetch in any case: the sources already live in the shared
# object store this job publishes to. Put the file on the volume before running
# (or teach step 1 to read from that store) — do not reach out to a third party
# from here.
if [ ! -s "$PBF" ]; then
    echo "error: no PBF at $PBF" >&2
    echo "       this job does not download. Place the .osm.pbf there first —" >&2
    echo "       the sources are in the object store, not on a public mirror." >&2
    exit 1
fi
echo "==> using PBF: $PBF ($(du -h "$PBF" | cut -f1))"

# ── 2. Bake ──────────────────────────────────────────────────────────────────
# Writes $SLAB plus the $SLAB.books codebook sidecar. The sidecar is not
# optional: it carries the identity/tag codebooks the ordinals in the rows
# resolve through.
echo "==> baking"
bake "$PBF" "$SLAB"

ls -l "$SLAB" "$SLAB.books"

# ── 3. Publish ───────────────────────────────────────────────────────────────
# Skipped, not failed, when the object store is unconfigured — that makes the
# image useful for a local "just bake it" run, and matches how the consumer
# treats a missing slab (unavailable, not broken).
if [ -z "${AWS_S3_BUCKET_NAME:-}" ] || [ -z "${AWS_ACCESS_KEY_ID:-}" ]; then
    echo "==> no object store configured (AWS_S3_BUCKET_NAME / AWS_ACCESS_KEY_ID unset)"
    echo "    slab left at $SLAB"
    exit 0
fi

echo "==> publishing to s3://$AWS_S3_BUCKET_NAME/${BAKE_S3_PREFIX:-q2/bakes}/${BAKE_VERSION:-osm-$NAME-v0.1.0}/"
python3 /usr/local/bin/upload_bake.py "$SLAB"

echo "==> done"
