#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Check every pinned dependency against the OSV vulnerability database.

    python3 scripts/audit_deps.py            # all three compiled files + the Dockerfile pins
    python3 scripts/audit_deps.py <file>...  # specific requirements files

Exits non-zero if any pinned version has a known advisory, so it works as a gate.

## Why this exists, and what it is not

Pinning dependencies buys reproducibility. It does not tell you the pinned versions
are safe — and "we pinned everything" reads like a security claim, so something has
to actually check. This is that check, and it is deliberately dependency-free: it
speaks to `api.osv.dev` over stdlib HTTP, so it runs on a laptop, in CI, and inside
`make` with nothing installed.

It is NOT a replacement for a full SBOM-based scan. It covers Python distributions
by name and version only:

  - no OS packages from the container base images
  - no npm (use `npm audit` in deployment/ — the Makefile target runs both)
  - no vendored source, and in particular not `libvoxel.wasm`, whose problem is
    provenance rather than a CVE (see THIRD-PARTY-LICENSES)

For the complete picture generate an SBOM and scan that instead:

    syft . -o cyclonedx-json > sbom.json && grype sbom:sbom.json

## Known accepted findings

`setuptools==65.5.1` (docker/Dockerfile, build-time only) reports three CVEs — six
OSV ids, since GHSA and PYSEC each record all three — that cannot be fixed by moving
the pin, because it is the newest release colcon can parse. The reachability analysis is at the call site in docker/Dockerfile; this
script lists them rather than hiding them, and `ACCEPTED` below is what keeps the
exit code honest without silencing the report.
"""

from __future__ import annotations

import json
import pathlib
import re
import sys
import urllib.error
import urllib.request

OSV_BATCH = "https://api.osv.dev/v1/querybatch"

# The COMPILED files (uv pip compile output), not the hand-written .in files:
# these carry the full transitive closure AND are what `pip install -r` reads, so
# what gets scanned is what gets installed.
# requirements-arm64.txt is the same image as requirements.txt built for aarch64
# (local Docker Desktop). Mostly the same pins, but it is scanned separately rather
# than assumed equivalent: it carries different torch/torchvision builds and drops
# the nvidia-* packages entirely — see requirements-arm64.in.
DEFAULT_FILES = [
    "requirements.txt",
    "requirements-arm64.txt",
    "voice_agent/requirements.txt",
    "voice_agent/requirements-agentcore.txt",
]

# Pins that live in a Dockerfile rather than a requirements file, so they would
# otherwise go unscanned. Keep in step with docker/Dockerfile and the Lambda image.
EXTRA_PINS = {
    ("setuptools", "65.5.1"): "docker/Dockerfile (build-time only)",
    ("packaging", "21.3"): "docker/Dockerfile (build-time only)",
    ("imageio-ffmpeg", "0.6.0"): "deployment/lambdas/scene-describer/Dockerfile",
}

# (package, version) -> why a known advisory is accepted. Anything listed here is
# still PRINTED; it just does not fail the run. Removing a pin from here must be
# the easy path, so the reason is one line pointing at the full analysis.
ACCEPTED = {
    ("setuptools", "65.5.1"): (
        "Newest release colcon's setup.py parsing accepts. All advisories are in "
        "package_index/sdist paths this image never reaches — see the comment at "
        "the pin in docker/Dockerfile."
    ),
}

_PIN = re.compile(r"^([A-Za-z0-9._-]+)==([^\s;#]+)")


def read_pins(path: str) -> dict[tuple[str, str], str]:
    """Extract `name==version` pins from a requirements or lock file."""
    out: dict[tuple[str, str], str] = {}
    for raw in pathlib.Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].split(";", 1)[0].strip()
        m = _PIN.match(line)
        if m:
            out[(m.group(1).lower(), m.group(2))] = path
    return out


def query_osv(pins: list[tuple[str, str]]) -> list[list[str]]:
    """Return the advisory ids for each pin, in the order given.

    OSV caps a batch, so this chunks. A network failure is fatal on purpose: a
    silent "no vulnerabilities" because the API was unreachable is the worst
    possible outcome for a tool like this.
    """
    ids: list[list[str]] = []
    for start in range(0, len(pins), 500):
        chunk = pins[start:start + 500]
        body = json.dumps({
            "queries": [
                {"package": {"name": n, "ecosystem": "PyPI"}, "version": v}
                for n, v in chunk
            ]
        }).encode()
        req = urllib.request.Request(
            OSV_BATCH, data=body, headers={"Content-Type": "application/json"}
        )
        # B310 flags urlopen because file:// and custom schemes are reachable
        # through it. The URL here is the module constant above and never derived
        # from input, but assert the scheme anyway so the guarantee is enforced by
        # the code rather than by the reader checking one line up.
        if req.type != "https":
            raise ValueError(f"refusing non-https OSV endpoint: {OSV_BATCH!r}")
        # https is asserted immediately above, so B310 does not apply.
        with urllib.request.urlopen(req, timeout=120) as resp:  # nosec B310
            results = json.load(resp)["results"]
        ids.extend(sorted(v["id"] for v in (r.get("vulns") or [])) for r in results)
    return ids


def main(argv: list[str]) -> int:
    files = argv[1:] or DEFAULT_FILES

    pins: dict[tuple[str, str], set[str]] = {}
    for f in files:
        if not pathlib.Path(f).exists():
            print(f"[!] no such file: {f}", file=sys.stderr)
            return 2
        for key, where in read_pins(f).items():
            pins.setdefault(key, set()).add(where)
    if not argv[1:]:
        for key, where in EXTRA_PINS.items():
            pins.setdefault(key, set()).add(where)

    if not pins:
        print("[!] no exact pins found — is the file using ranges?", file=sys.stderr)
        return 2

    print(f"[*] checking {len(pins)} pinned packages against OSV...")
    keys = sorted(pins)
    try:
        advisories = query_osv(keys)
    except (urllib.error.URLError, TimeoutError) as e:
        print(f"[!] could not reach OSV ({e}). Treating as a FAILURE rather than "
              "reporting a clean run.", file=sys.stderr)
        return 2

    failures = 0
    accepted = 0
    for (name, version), ids in zip(keys, advisories):
        if not ids:
            continue
        note = ACCEPTED.get((name, version))
        label = "ACCEPTED" if note else "VULNERABLE"
        if note:
            accepted += 1
        else:
            failures += 1
        print(f"\n[{label}] {name}=={version}")
        for where in sorted(pins[(name, version)]):
            print(f"    from {where}")
        print(f"    {', '.join(ids)}")
        for i in ids:
            print(f"    https://osv.dev/vulnerability/{i}")
        if note:
            print(f"    accepted: {note}")

    print()
    if failures:
        print(f"[!] {failures} package(s) with unaccepted advisories. Bump the pin "
              "in the matching .in file and `make lock`, or add a reviewed entry "
              "to ACCEPTED in this script.")
        return 1
    print(f"[ok] no unaccepted advisories ({accepted} accepted, see above).")
    print("     Reminder: this covers PyPI pins only — run `npm audit` in "
          "deployment/ and an SBOM scan for the rest.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
