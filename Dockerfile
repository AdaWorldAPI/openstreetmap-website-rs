# `bake` as a one-off Railway JOB — not a web service.
#
# This repo produces a Morton-sorted slab of 512-byte V3 ABI rows from an
# `.osm.pbf`; it serves nothing. Deploying it as a long-running service would
# give you a container with no listener. Run it as a Railway one-off / cron
# job instead: it bakes, uploads to the object store, and exits.
#
# The consumer is a *different* service — q2's `cockpit-server`, which serves
# `/osm` and `GET /api/osm/features/:z/:x/:y` by mmap'ing the slab this job
# produced. The object store is the seam between them; nothing here needs to
# know the consumer exists.
#
# Why the sibling clone: `osm-soa-bake` path-deps
# `../lance-graph/crates/lance-graph-contract` (the V3 `canonical_node` /
# `facet` / `identity_quad` types are contract-owned, deliberately — the bake
# consumes them rather than re-declaring the ABI). `ogar-osm` / `ogar-vocab`
# are git deps and resolve over the network. All three repos are public, so no
# build secret is needed to fetch them.
#
# Toolchain: 1.97.1. `ogar-osm` / `ogar-vocab` declare `rust-version = 1.95`,
# so the default 1.94 in some images fails resolution outright with a
# "not supported by the following packages" error rather than a compile error.

FROM rust:1.97-bookworm AS builder

RUN apt-get update && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

# The sibling layout the path dep expects: /build/lance-graph next to
# /build/openstreetmap-website-rs. Cloned before the source copy so a source
# edit does not invalidate this layer.
RUN git clone --depth 1 https://github.com/AdaWorldAPI/lance-graph.git /build/lance-graph

COPY . /build/openstreetmap-website-rs
WORKDIR /build/openstreetmap-website-rs

# Only the baker. The other bins in this crate are probes and are not part of
# the job's contract.
RUN cargo build --release --bin bake

FROM debian:bookworm-slim

# ca-certificates: HTTPS to both the PBF source and the object store.
# python3-boto3: the upload. Chosen over the AWS CLI (a much larger install)
# and over hand-rolled SigV4 in shell (signing a multipart upload correctly in
# bash is not a thing worth owning here). boto3 also handles the custom
# endpoint + multipart chunking for the ~1.2 GiB slab without extra flags.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl python3 python3-boto3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=builder /build/openstreetmap-website-rs/target/release/bake /usr/local/bin/bake
COPY docker/bake-entrypoint.sh /usr/local/bin/bake-entrypoint.sh
COPY docker/upload_bake.py /usr/local/bin/upload_bake.py
RUN chmod +x /usr/local/bin/bake-entrypoint.sh

# Scratch lives on the attached volume when there is one: the Berlin slab is
# ~1.2 GiB and a container's writable layer is the wrong place for it.
# Falls back to /tmp so the image is runnable without a volume.
ENV RAILWAY_VOL=/volume01
ENV OSM_PBF_URL=https://download.geofabrik.de/europe/germany/berlin-latest.osm.pbf
ENV BAKE_NAME=berlin
ENV BAKE_VERSION=osm-berlin-v0.1.0
ENV BAKE_S3_PREFIX=q2/bakes

ENTRYPOINT ["/usr/local/bin/bake-entrypoint.sh"]
