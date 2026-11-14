#!/usr/bin/env python3
"""Is any of these node programs already running?  Exit 1 if so.

    dupcheck.py relocalizer_live.py accumulator.py fastlio_mapping

Prints one line per offender plus a ready-made `kill` command, and exits 1.
Exits 0 when nothing matches.

WHY IT EXISTS
Measured 2026-10-30: the whole stack was running TWICE. Two relocalizers both
published map->camera_init - one still at the identity seed, one registered
3.00 m away - so the transform alternated every sample. The accumulator logged
ORIGIN CHANGED forever and stored 0 points while inliers read a healthy 0.69.
Nothing in that picture says "duplicate", which is why this is a hard refusal
at startup rather than a warning.

Orphans are easy to make: the start scripts `setsid` their children, so a
supervisor that dies without reaping - closed terminal, SIGKILL - leaves the
nodes parentless and the next start silently doubles up.

MATCHING
By exact argv BASENAME. Never a substring: a substring search matches the
command line of the very script doing the search, which has previously killed
a shell on this robot. Self and the ancestor chain are excluded too, so a
wrapper named like a node cannot match itself.

SPEED
One pass over /proc in one process. The bash equivalent forked `tr` per entry
and took 22.8 s; this is milliseconds.
"""
import os
import sys


def ancestors(pid):
    """self + parents, so we never report the caller."""
    out, seen = set(), 0
    while pid and pid not in out and seen < 40:
        out.add(pid)
        seen += 1
        try:
            with open('/proc/%d/stat' % pid) as f:
                st = f.read()
            pid = int(st[st.rindex(')') + 2:].split()[1])
        except Exception:
            break
    return out


def main():
    wanted = set(sys.argv[1:])
    if not wanted:
        print('usage: dupcheck.py <program-basename> ...', file=sys.stderr)
        return 2

    skip = ancestors(os.getpid())
    found = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        pid = int(d)
        if pid in skip:
            continue
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as f:
                toks = f.read().split(b'\0')
        except Exception:
            continue
        for tok in toks:
            if not tok:
                continue
            base = os.path.basename(tok.decode('utf-8', 'replace'))
            if base in wanted:
                found.setdefault(base, []).append(pid)
                break

    if not found:
        return 0
    total = 0
    for name in sorted(found):
        pids = sorted(found[name])
        total += len(pids)
        print('    !! %s already running x%d  (pid %s)'
              % (name, len(pids), ' '.join(str(p) for p in pids)))
    allp = sorted(p for v in found.values() for p in v)
    print()
    print('    %d process(es) from a previous run are alive. Starting now' % total)
    print('    would duplicate them, and duplicate TF publishers give a')
    print('    SILENTLY broken stack, not an obvious error.')
    print()
    print('    Stop them by pid (never pkill -f, it matches the caller):')
    print('        kill %s' % ' '.join(str(p) for p in allp))
    return 1


if __name__ == '__main__':
    sys.exit(main())
