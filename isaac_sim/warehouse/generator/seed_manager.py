"""Layout RNG + instance-id helpers (Stage 1, implemented).

Only ONE random stream is used this phase (layout). Seed is NEVER part of the
layout fingerprint; it lives in metadata only.
"""
from __future__ import annotations

import numpy as np

STREAM_ORDER = ["layout"]


def layout_rng(seed: int) -> np.random.Generator:
    """Deterministic, isolated RNG for the layout stream.

    Uses numpy SeedSequence + PCG64 (no global random state touched).
    """
    ss = np.random.SeedSequence(int(seed))
    child = ss.spawn(len(STREAM_ORDER))[0]
    return np.random.Generator(np.random.PCG64(child))


class InstanceCounter:
    """Per-class monotonic instance ids: box_0001, pallet_0001, ..."""

    def __init__(self):
        self._n = {}

    def next(self, cls: str) -> str:
        n = self._n.get(cls, 0) + 1
        self._n[cls] = n
        return "%s_%04d" % (cls, n)
