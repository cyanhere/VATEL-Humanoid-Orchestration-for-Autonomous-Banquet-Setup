# Drift: relocalizer on vs odometry only — 2026-11-11

## Result

| | walked | endpoint error vs tape | % of distance |
|---|---|---|---|
| relocalizer on  | 57.68 m | **0.022 m** | 0.04% |
| odometry only   | 57.62 m | **0.189 m** | 0.33% |

**8.5× better with the relocalizer.**

Second result, needing no external reference (two runs agree):

| run | correction applied | over | = |
|---|---|---|---|
| with-reloc 163256 | 2.569 m | 59.64 m | 4.3% |
| with-reloc 165105 | 2.289 m | 57.68 m | 4.0% |
| seed-only 171609  | 0.000 m | 57.62 m | ICP confirmed off |

FAST-LIO drifted ~4% of distance; the relocalizer cancelled it continuously
(net origin shift only 0.009–0.015 m). Compare 6.8% for FAST-LIO2 in the
Wildcat paper.

## Protocol

Five 5 m out-and-back laps under Nav2 (`nav/driftnav.py`), ~58 m per run, same
physical start mark, endpoint measured with a tape. Both runs completed 5/5.

Laps are repeated because `goal_checker.xy_goal_tolerance` is 0.30 m: a single
arrival can be 0.30 m off for reasons unrelated to drift, and that error does
not accumulate, while drift does.

## Files

| file | what |
|---|---|
| `drift_relocalizer.png` | the figure (2400×1480, white background) |
| `make_drift_fig.py` | regenerates it from `drift_series.json` |
| `drift_series.json` | distance-walked vs cumulative-correction, per run |
| `drift_navlaps-*.csv` | the raw logs, 10 Hz, three TF chains + inliers/rmse |
| `*.truth.json` | the tape measurements attached to each run |

Re-analyse any time:

    python3 nav/driftlog.py compare drift_navlaps-*.csv

## Three things not to get wrong

1. **Ground truth must come from outside the estimator.** Annotating with the
   estimator's own output gives 0.000 m error by construction.
2. **`--dy` is + = LEFT of the start heading.** The tape readings here were
   −0.26 and −0.27 (to the right). Flipping that sign changes the answer from
   8.5× to 1.3×.
3. **Do not plot the estimated trajectories.** Under `seed_only` the robot
   navigates in a frame that is sliding, so its estimated path looks tidy — it
   believes it returns to the start every lap. The drift is invisible from
   inside. The correction signal is the honest in-run view.

## Caveats

- One annotated run per condition; the 8.5× is a single sample each.
- 0.189 m is a small signal for a tape measurement.
- Out-and-back lets along-track error partly cancel, so 0.33% understates
  one-way odometry drift. The relocalizer's advantage is a floor.
