#!/usr/bin/env python3
"""Publish a baked slab + its codebook sidecar + a SHA256SUMS to the object store.

Layout matches the convention already in the bucket
(``<repo>/bakes/<version>/<artifact>``), so a consumer's hydrate path is the
same shape for every bake regardless of which repo produced it.

The checksums are computed here, from the bytes that were actually written --
not copied from the baker's reported digest. That is the point: the consumer
verifies what it downloaded against what was uploaded, so a truncated or
substituted object fails loudly instead of mmap'ing as a short slab. The
``SHA256SUMS`` format is ``sha256sum -c`` compatible.

Credentials come from the environment at the call site and are never logged.
"""

import hashlib
import os
import sys

import boto3
from boto3.s3.transfer import TransferConfig

# 64 MiB parts: the slab is ~1.2 GiB, which is over the 5 GiB single-PUT limit's
# comfort zone and benefits from concurrent parts on a cold container.
_TRANSFER = TransferConfig(
    multipart_threshold=64 << 20,
    multipart_chunksize=64 << 20,
    max_concurrency=4,
    use_threads=True,
)


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(8 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: upload_bake.py <slab.soa>", file=sys.stderr)
        return 2
    slab = sys.argv[1]
    books = slab + ".books"
    for p in (slab, books):
        if not os.path.isfile(p):
            print(f"missing expected artifact: {p}", file=sys.stderr)
            return 1

    out_dir = os.path.dirname(slab) or "."
    name = os.path.basename(slab)
    version = os.environ.get("BAKE_VERSION") or f"osm-{name.split('.')[0]}-v0.1.0"
    prefix = f"{os.environ.get('BAKE_S3_PREFIX', 'q2/bakes')}/{version}"

    artifacts = [name, name + ".books"]
    sums = {a: sha256(os.path.join(out_dir, a)) for a in artifacts}
    for a in artifacts:
        print(f"sha256 {a}: {sums[a]}", flush=True)

    sums_path = os.path.join(out_dir, "SHA256SUMS")
    with open(sums_path, "w", encoding="utf-8") as fh:
        for a in artifacts:
            fh.write(f"{sums[a]}  {a}\n")

    s3 = boto3.client(
        "s3",
        endpoint_url=os.environ.get("AWS_ENDPOINT_URL") or None,
        region_name=os.environ.get("AWS_DEFAULT_REGION", "auto"),
    )
    bucket = os.environ["AWS_S3_BUCKET_NAME"]

    for a in artifacts + ["SHA256SUMS"]:
        path, key = os.path.join(out_dir, a), f"{prefix}/{a}"
        size = os.path.getsize(path)
        print(f"uploading {a} ({size/1e6:.1f} MB) -> s3://{bucket}/{key}", flush=True)
        s3.upload_file(path, bucket, key, Config=_TRANSFER)
        # Read the size back: an upload that "succeeded" but landed short is the
        # failure this whole checksum dance exists to catch, and it is cheap to
        # rule out here rather than at the consumer's first mmap.
        got = s3.head_object(Bucket=bucket, Key=key)["ContentLength"]
        if got != size:
            print(f"size mismatch for {key}: local {size}, remote {got}", file=sys.stderr)
            return 1
        print(f"  ok {a} ({got} bytes)", flush=True)

    print(f"published {prefix}/", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
