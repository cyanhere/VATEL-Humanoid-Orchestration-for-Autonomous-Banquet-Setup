"""Tables (and chairs) in a hall CCTV frame: Grounded-SAM on a people-free background.

    python cctv_segment.py hall.mp4                 # segment a recording, write an overlay
    python cctv_segment.py hall.mp4 --prompt "round table with white tablecloth"

WHY THIS DESIGN, AND NOT ONE TUNED TO A ROOM
    The camera has to work in a hall we have only seen once, with tables we did not choose.
    So nothing here knows what this venue's tables look like:

    - Grounding DINO finds objects from a TEXT prompt ("table", "chair"), so a new venue's
      furniture is a new sentence, not new training data. SAM then turns each box into the
      object's exact outline.
    - Both run on a CLEAN PLATE: the per-pixel median of frames sampled over the last minute
      or so. Tables stand still and people do not, so the median keeps the furniture and
      drops the staff walking between it -- the detector never has to see past a person.
    - Everything after the models is plain geometry on masks: duplicates and "several tables
      in one box" are resolved by how masks overlap, not by any size or colour of table.

    Only the prompt and two thresholds are settings; the defaults are what works on the
    hub's three cameras, and they are the first things to change for a new hall.

WHAT IT RETURNS
    A list of Instance(cls, score, box, mask, outline) in the frame's own pixels. Turning
    those into positions on the floor is cctv_camera.py's job -- this file never needs to
    know where the camera is.

Needs torch + transformers (Grounding DINO base, SAM ViT-H weights, ~3.3 GB, fetched once from
Hugging Face) and OpenCV. A GPU makes a frame ~2 s; on a CPU it is closer to a minute, which is
still fine for tables that move a few times an hour.
"""
import argparse
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    sys.exit("cctv_segment.py needs OpenCV:  pip install opencv-python")

DINO_ID = "IDEA-Research/grounding-dino-base"
SAM_ID = "facebook/sam-vit-huge"
# What to look for: class -> the words that mean it in the prompt. A banquet hall would use
# {"table": ["round table", "table"]}, with no chairs; the hub has both.
DEFAULT_CLASSES = {"table": ["table"], "chair": ["chair"]}
WORK_WIDTH = 1280            # the models see the frame at this width; outlines come back full-size


@dataclass
class Instance:
    cls: str
    score: float
    box: np.ndarray                       # x0, y0, x1, y1 in full-frame pixels
    mask: np.ndarray = field(repr=False)  # bool, at work size
    outline: np.ndarray = None            # (N, 2) full-frame pixels, the mask's outer contour
    scale: float = 1.0                    # full-frame pixels per work pixel


# ============================================================ the people-free background
class CleanPlate:
    """Median of the most recent frames, sampled sparsely.

    `every` seconds between samples and `keep` samples: the defaults span about a minute,
    long enough for anyone crossing the view to be in a minority of samples at every pixel,
    short enough that a table moved by the staff shows up within a minute. A person who
    stands still for most of that window will be kept -- which is a person standing where a
    table might be, and the model is asked about tables, not people."""

    def __init__(self, keep=31, every=2.0):
        self.keep, self.every = keep, every
        self.frames, self.last = [], -1e9

    def offer(self, frame, t=None):
        t = time.time() if t is None else t
        if t - self.last < self.every and self.frames:
            return False
        self.last = t
        self.frames.append(frame)
        if len(self.frames) > self.keep:
            self.frames.pop(0)
        return True

    def image(self):
        if not self.frames:
            return None
        if len(self.frames) < 3:
            return self.frames[-1].copy()
        return np.median(np.stack(self.frames), axis=0).astype(np.uint8)


def clean_plate_from_video(path, samples=60):
    """The clean plate of a whole recording. Reads in ORDER: seeking an HEVC stream lands on
    frames whose references were never decoded, which come back flat grey -- a median of
    those is a grey rectangle, and both models will happily find nothing in it."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit("cannot open %s" % path)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    k = max(1, n // samples)
    frames, i = [], 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i % k == 0:
            frames.append(f)
        i += 1
    cap.release()
    if not frames:
        raise SystemExit("no frames decoded from %s" % path)
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


# ============================================================ mask geometry (no models)
def _clean_mask(m, min_frac=0.15):
    """Open away speckle, keep the components that carry the object, fill its holes."""
    m = m.astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return m.astype(bool)
    areas = st[1:, cv2.CC_STAT_AREA]
    keep = np.zeros(n, bool)
    keep[1:] = areas >= min_frac * areas.max()
    m = keep[lab].astype(np.uint8)
    # fill holes: everything not reachable from the border is inside
    inv = np.pad(1 - m, 1, constant_values=1)
    ff = inv.copy()
    cv2.floodFill(ff, None, (0, 0), 2)
    holes = (ff[1:-1, 1:-1] == 1)
    return (m.astype(bool) | holes)


def resolve(instances, iou_dup=0.5, min_area=150):
    """Turn the raw detections into one mask per object.

    Three things go wrong with box-prompted segmentation in a furnished room, and each is
    settled by how masks overlap rather than by anything about the venue:

      1. GROUP BOXES. The detector proposes a box around several tables pushed together AND
         boxes around each one. A mask that is mostly covered by smaller masks of the same
         class, and contains at least two of them, is the group -- drop it, keep the parts.
      2. DUPLICATES. Two boxes on one object: keep the higher score.
      3. ONE THING, TWO NAMES. A mask called both "table" and "chair": keep the likelier name.

    Finally a chair standing in front of a table is cut out of the table's mask, so the
    table's outline is the table's.
    """
    out = [i for i in instances if i.mask.sum() >= min_area]
    # 1 + 2, per class
    by_cls = {}
    for inst in out:
        by_cls.setdefault(inst.cls, []).append(inst)
    kept = []
    for cls, lst in by_cls.items():
        lst.sort(key=lambda i: -i.mask.sum())                      # biggest first
        extra = []
        for a in range(len(lst)):
            ma = lst[a].mask
            inside = [b for b in range(a + 1, len(lst))
                      if (lst[b].mask & ma).sum() > 0.7 * lst[b].mask.sum()]
            if len(inside) < 2:
                continue
            union = np.zeros_like(ma)
            for b in inside:
                union |= lst[b].mask
            # A group box: the smaller masks explain part of it. Do NOT drop the group --
            # on the first real frames a box spanning two rows of tables lost the whole
            # second row that way, because only the first row had boxes of its own.
            # Keep what nothing smaller explains, as its own object(s).
            rest = _clean_mask(ma & ~cv2.dilate(union.astype(np.uint8),
                                                  np.ones((5, 5), np.uint8)).astype(bool))
            n, lab, st, _ = cv2.connectedComponentsWithStats(rest.astype(np.uint8), 8)
            for k in range(1, n):
                if st[k, cv2.CC_STAT_AREA] >= max(min_area, 0.05 * ma.sum()):
                    extra.append(Instance(cls, lst[a].score, lst[a].box, _clean_mask(lab == k),
                                          scale=lst[a].scale))
            lst[a].mask = np.zeros_like(ma)                       # the group itself goes
        lst = [i for i in lst if i.mask.sum() >= min_area] + extra
        lst.sort(key=lambda i: -i.score)
        final = []
        for inst in lst:
            dup = False
            for f in final:
                inter = (inst.mask & f.mask).sum()
                union = (inst.mask | f.mask).sum()
                if union and inter / union > iou_dup:
                    dup = True
                    break
            if not dup:
                final.append(inst)
        kept.extend(final)
    # 3: cross-class
    kept.sort(key=lambda i: -i.score)
    final = []
    for inst in kept:
        clash = False
        for f in final:
            if f.cls == inst.cls:
                continue
            inter = (inst.mask & f.mask).sum()
            union = (inst.mask | f.mask).sum()
            if union and inter / union > iou_dup:
                clash = True
                break
        if not clash:
            final.append(inst)
    # chairs out of tables
    chairs = [i for i in final if i.cls == "chair"]
    if chairs:
        cm = np.zeros_like(final[0].mask)
        for c in chairs:
            cm |= c.mask
        for t in final:
            if t.cls != "chair":
                cut = t.mask & ~cm
                if cut.sum() >= 0.5 * t.mask.sum():
                    t.mask = cut
    return [i for i in final if i.mask.sum() >= min_area]


def outline_of(mask, scale=1.0, eps_px=1.5):
    """The mask's outer contour as an (N, 2) polygon, in full-frame pixels."""
    m = mask.astype(np.uint8)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs:
        return np.zeros((0, 2))
    c = max(cs, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, eps_px, True).reshape(-1, 2).astype(float)
    return (c + 0.5) * scale - 0.5


# ============================================================ the models
class Segmenter:
    # 0.22, not the usual 0.30: on the hub's sandbox camera a row of dark tables pushed
    # together scored 0.22-0.28 and vanished at 0.30. Recall is bought here and paid for
    # downstream -- duplicates by mask overlap, then anything that is not table-sized or
    # not on this camera's floor once it is projected (cctv_camera.py).
    def __init__(self, classes=None, box_threshold=0.22, text_threshold=0.20, device=None,
                 dino=DINO_ID, sam=SAM_ID, work_width=WORK_WIDTH):
        import torch
        from transformers import (AutoModelForZeroShotObjectDetection, AutoProcessor,
                                  SamModel, SamProcessor)
        self.torch = torch
        self.classes = classes or DEFAULT_CLASSES
        self.box_thr, self.text_thr = box_threshold, text_threshold
        self.work_width = work_width
        self.dev = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.dproc = AutoProcessor.from_pretrained(dino)
        self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(dino).to(self.dev).eval()
        self.sproc = SamProcessor.from_pretrained(_local_or_hub(sam))
        self.sam = SamModel.from_pretrained(_local_or_hub(sam)).to(self.dev).eval()
        words = [w for ws in self.classes.values() for w in ws]
        self.prompt = " ".join(w.strip().rstrip(".") + "." for w in words)

    def _cls_of(self, label):
        label = (label or "").lower()
        best = None
        for cls, words in self.classes.items():
            for w in words:
                if w.lower() in label and (best is None or len(w) > best[1]):
                    best = (cls, len(w))
        return best[0] if best else None

    def segment(self, bgr):
        from PIL import Image
        torch = self.torch
        H, W = bgr.shape[:2]
        s = self.work_width / W
        small = cv2.resize(bgr, (self.work_width, int(round(H * s))), interpolation=cv2.INTER_AREA)
        img = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        inp = self.dproc(images=img, text=self.prompt, return_tensors="pt").to(self.dev)
        with torch.no_grad():
            out = self.dino(**inp)
        r = self.dproc.post_process_grounded_object_detection(
            out, inp.input_ids, threshold=self.box_thr, text_threshold=self.text_thr,
            target_sizes=[img.size[::-1]])[0]
        boxes = r["boxes"].cpu().numpy()
        scores = r["scores"].cpu().numpy()
        labels = list(r.get("text_labels", r.get("labels")))
        keep = [k for k, l in enumerate(labels) if self._cls_of(str(l))]
        if not keep:
            return []
        boxes, scores = boxes[keep], scores[keep]
        clss = [self._cls_of(str(labels[k])) for k in keep]
        sin = self.sproc(images=img, input_boxes=[boxes.tolist()], return_tensors="pt").to(self.dev)
        with torch.no_grad():
            so = self.sam(**sin, multimask_output=False)
        masks = self.sproc.image_processor.post_process_masks(
            so.pred_masks.cpu(), sin["original_sizes"].cpu(), sin["reshaped_input_sizes"].cpu())[0]
        masks = masks[:, 0].numpy()
        inst = [Instance(c, float(sc), b / s, _clean_mask(m), scale=1.0 / s)
                for c, sc, b, m in zip(clss, scores, boxes, masks)]
        inst = resolve(inst)
        for i in inst:
            i.outline = outline_of(i.mask, i.scale)
        return inst


def _local_or_hub(model_id):
    """A model fetched into ~/.cache/vatel_models (plain files) if it is there, else the hub
    id. Windows without symlink rights cannot finish the hub's own cache layout."""
    local = os.path.expanduser(os.path.join("~/.cache/vatel_models", model_id.split("/")[-1]))
    return local if os.path.isdir(local) else model_id


# ============================================================ drawing
PALETTE = {"table": (90, 200, 40), "chair": (30, 170, 240)}       # BGR


def draw(bgr, instances, label=True):
    """The frame with each instance filled and outlined, at full resolution."""
    img = bgr.copy()
    over = img.copy()
    for inst in instances:
        c = PALETTE.get(inst.cls, (200, 60, 200))
        poly = inst.outline.astype(np.int32)
        if len(poly) >= 3:
            cv2.fillPoly(over, [poly], c)
    img = cv2.addWeighted(over, 0.45, img, 0.55, 0)
    lw = max(2, bgr.shape[1] // 640)
    for k, inst in enumerate(instances):
        c = PALETTE.get(inst.cls, (200, 60, 200))
        poly = inst.outline.astype(np.int32)
        if len(poly) >= 3:
            cv2.polylines(img, [poly], True, (255, 255, 255), lw + 1, cv2.LINE_AA)
            cv2.polylines(img, [poly], True, c, lw, cv2.LINE_AA)
        if label and inst.cls != "chair":
            x, y = poly[:, 0].mean(), poly[:, 1].mean()
            t = "%s %.2f" % (inst.cls, inst.score)
            fs = bgr.shape[1] / 1600
            (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)
            cv2.rectangle(img, (int(x - tw / 2) - 4, int(y - th) - 6), (int(x + tw / 2) + 4, int(y) + 6),
                          (0, 0, 0), -1)
            cv2.putText(img, t, (int(x - tw / 2), int(y)), cv2.FONT_HERSHEY_SIMPLEX, fs,
                        (255, 255, 255), 2, cv2.LINE_AA)
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("source", help="a video file, or an image")
    ap.add_argument("--prompt", nargs="*", help="words for 'table' (default: table)")
    ap.add_argument("--no-chairs", action="store_true")
    ap.add_argument("--out", help="overlay image to write (default: <source>_seg.jpg)")
    args = ap.parse_args(argv)

    classes = {"table": args.prompt or ["table"]}
    if not args.no_chairs:
        classes["chair"] = ["chair"]
    img = cv2.imread(args.source)
    if img is None:
        img = clean_plate_from_video(args.source)
    seg = Segmenter(classes=classes)
    t0 = time.time()
    inst = seg.segment(img)
    dt = time.time() - t0
    n = {}
    for i in inst:
        n[i.cls] = n.get(i.cls, 0) + 1
    print("%s: %s in %.1f s" % (os.path.basename(args.source),
                                ", ".join("%d %s" % (v, k) for k, v in n.items()) or "nothing", dt))
    out = args.out or os.path.splitext(args.source)[0] + "_seg.jpg"
    cv2.imwrite(out, draw(img, inst), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print("wrote", out)


if __name__ == "__main__":
    main()
