"""pk.fit on synthetic logs: rows where the motors were limp never enter a gravity fit.

    python3 tests/test_fit_offline.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pk import config  # noqa: E402
from pk import fit as F  # noqa: E402


def main():
    n = 200
    L = dict(stage=np.array(["HELD"] * n), weight=np.ones(n), dq=np.zeros((n, config.N_ARM)), tau=np.zeros((n, config.N_ARM)))
    L["tau"][:, :10] = 0.08                                  # limp: what the robot reported
    limp = len(F.samples(L, every=1))
    L["tau"][n // 2:, 3] = 1.2                               # the second half really held (an elbow at 1.2 N m)
    held = len(F.samples(L, every=1))
    ok = limp == 0 and held == n // 2
    print(("PASS" if ok else "FAIL") + " limp rows left out of the gravity fit (%d of %d limp rows kept; %d of %d held rows kept)" % (
        limp, n, held, n // 2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
