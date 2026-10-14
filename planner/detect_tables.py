"""
Find the round tables in a ceiling-camera still and print what the planner reads.

    python3 detect_tables.py hall.jpg                 # detect, print JSON
    python3 detect_tables.py hall.jpg -o tables.json  # and write it
    python3 detect_tables.py --sample                 # no camera? emit a sample

Then in the planner: Camera -> Import detections -> paste or open the file ->
Review. Nothing lands on the plan until a manager ticks it.

WHAT THIS IS
    A stub with a real detector inside it. There is no camera installed yet, so
    this has never been run on a hall. It exists so the format is fixed and the
    whole path is testable from the planner end before anyone mounts anything.

WHY PIXELS AND NOT METRES
    The planner already knows the room's true size, and a manager can click its
    four corners on a still in about ten seconds. That fixes the homography at
    the point where someone can see whether it looks right. Sending pixels keeps
    this script free of any opinion about where the camera ended up -- overhead,
    in a corner, or moved between functions. If you would rather do the mapping
    here, set "units": "metres" and fill centre_m / diameter_m instead; the
    planner accepts either.

WHAT IT WILL GET WRONG
    Hough circles on a ceiling view of a banquet hall will find chair backs,
    ceiling lights, and the round bases of anything else in the room. That is
    the point of the review step. If it turns out to be hopeless, the honest
    next move is a segmentation model on a few hundred labelled stills, not a
    longer list of Hough parameters -- the same lesson the plate detector in
    ../G1_Plate_Perception taught, where fitting a circle to the wrong thing
    stayed confidently wrong for a week.
"""

import argparse
import datetime
import json
import sys

SCHEMA = "banquet-detections/0.1"

# A 5 ft round is 1.52 m and a 7 ft is 2.13 m across. In a ceiling view of a
# hall that is roughly a tenth of the frame width, so these bounds are generous
# on purpose -- the planner's review step is what tightens them.
MIN_R_FRAC = 0.02
MAX_R_FRAC = 0.12


def detect(path):
    """Round tables in one still, as pixel circles. Needs opencv."""
    import cv2

    img = cv2.imread(path)
    if img is None:
        raise SystemExit("cannot read %s" % path)
    return detect_image(img)


def detect_image(img):
    """The same, on a BGR frame already in memory (what cctv_bridge.py hands over)."""
    import cv2
    import numpy as np

    h, w = img.shape[:2]
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grey = cv2.medianBlur(grey, 5)

    circles = cv2.HoughCircles(
        grey, cv2.HOUGH_GRADIENT, dp=1.2,
        minDist=int(w * MIN_R_FRAC * 2),
        param1=120, param2=60,
        minRadius=int(w * MIN_R_FRAC),
        maxRadius=int(w * MAX_R_FRAC),
    )
    out = []
    if circles is not None:
        for i, (cx, cy, r) in enumerate(np.round(circles[0]).astype(int)):
            # Hough gives no confidence of its own. Standing in for one: how
            # consistent the edge is around the rim. A real table reads high;
            # a light fitting or a chair back usually does not.
            out.append({
                "id": "d%d" % (i + 1),
                "shape": "round",
                "centre_px": [int(cx), int(cy)],
                "diameter_px": int(2 * r),
                "confidence": round(rim_consistency(grey, cx, cy, r), 2),
            })
    return {"frame": {"width": w, "height": h}, "tables": out}


def rim_consistency(grey, cx, cy, r, n=48):
    """Fraction of rays around the rim that actually find an edge there."""
    import numpy as np

    h, w = grey.shape[:2]
    hits = 0
    for k in range(n):
        a = 2 * np.pi * k / n
        u, v = int(cx + r * np.cos(a)), int(cy + r * np.sin(a))
        if not (1 <= u < w - 1 and 1 <= v < h - 1):
            continue
        patch = grey[v - 1:v + 2, u - 1:u + 2].astype(float)
        if patch.max() - patch.min() > 12:
            hits += 1
    return hits / float(n)


def sample():
    return {
        "frame": {"width": 1920, "height": 1080},
        "tables": [
            {"id": "d1", "shape": "round", "centre_px": [520, 360],
             "diameter_px": 196, "confidence": 0.94},
            {"id": "d2", "shape": "round", "centre_px": [880, 352],
             "diameter_px": 190, "confidence": 0.91},
            {"id": "d3", "shape": "round", "centre_px": [1240, 366],
             "diameter_px": 201, "confidence": 0.88},
            {"id": "d4", "shape": "round", "centre_px": [700, 640],
             "diameter_px": 193, "confidence": 0.83},
            {"id": "d5", "shape": "round", "centre_px": [1060, 648],
             "diameter_px": 188, "confidence": 0.61},
        ],
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("image", nargs="?", help="a still from the ceiling camera")
    ap.add_argument("-o", "--out", help="write here instead of stdout")
    ap.add_argument("--source", default="ceiling-cam-1", help="which camera")
    ap.add_argument("--sample", action="store_true",
                    help="emit a sample instead of detecting")
    args = ap.parse_args(argv)

    if not args.sample and not args.image:
        ap.error("give an image, or --sample")

    body = sample() if args.sample else detect(args.image)
    doc = {
        "schema": SCHEMA,
        "source": args.source,
        "captured_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "units": "pixels",
        "frame": body["frame"],
        "tables": body["tables"],
    }
    text = json.dumps(doc, indent=2)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")
        print("%d tables -> %s" % (len(doc["tables"]), args.out), file=sys.stderr)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
