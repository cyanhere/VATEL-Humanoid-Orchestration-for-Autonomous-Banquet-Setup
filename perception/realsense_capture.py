"""One depth frame off a RealSense, aligned to colour, at whatever mode the USB link can carry.

On USB 3 the D435 runs depth and colour at 848 x 480, 30 fps. On USB 2 -- which is what a
camera gets behind some hubs, or with a cable that does not seat its USB 3 pins -- that
request is refused ("Couldn't resolve requests"), so the modes are tried in order, best
first, and the first that starts is kept for the rest of the run. As a last resort it runs
depth alone. Intrinsics always come from the stream the depth is expressed in, so a
smaller mode costs resolution, not correctness.
"""
import numpy as np

# (width, height, fps), best first. The USB 2 ones are the D435's documented USB 2 modes.
MODES = [(848, 480, 30), (848, 480, 15), (640, 480, 30), (640, 480, 15), (640, 480, 6),
         (424, 240, 30), (424, 240, 15)]
_chosen = None                       # (w, h, fps, with_colour) once one has started


def _start(rs, w, h, fps, colour):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    if colour:
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    return pipe, pipe.start(cfg)


def capture(warmup=30, log=print):
    """-> (colour BGR image or None, depth uint16, intrinsics dict, depth units per metre)"""
    color, depths, intr, scale = capture_frames(1, warmup, log)
    return color, depths[0], intr, scale


def capture_frames(n, warmup=30, log=print):
    """n consecutive depth frames from one start of the camera (so all of them settled):
    -> (colour of the last or None, [depth uint16] * n, intrinsics dict, units per metre)"""
    global _chosen
    import pyrealsense2 as rs

    tries = [_chosen] if _chosen else \
        [(w, h, f, True) for w, h, f in MODES] + [(w, h, f, False) for w, h, f in MODES]
    last = None
    for w, h, fps, colour in tries:
        try:
            pipe, profile = _start(rs, w, h, fps, colour)
        except RuntimeError as e:
            last = e
            continue
        if _chosen is None:
            _chosen = (w, h, fps, colour)
            if (w, h, fps, colour) != (848, 480, 30, True):
                log("camera: %d x %d at %d fps%s -- the USB link cannot carry the full mode "
                    "(USB 2?). Detection still works, at lower resolution."
                    % (w, h, fps, "" if colour else ", depth only"))
        break
    else:
        raise RuntimeError("the RealSense would not start in any mode (%s)" % last)

    try:
        align = rs.align(rs.stream.color) if colour else None
        depths = []
        for k in range(max(warmup, 1) + n - 1):   # let auto-exposure and depth settle first
            frames = pipe.wait_for_frames()
            if align is not None:
                frames = align.process(frames)
            if k >= max(warmup, 1) - 1:
                depths.append(np.asanyarray(frames.get_depth_frame().get_data()).copy())
        depth_f = frames.get_depth_frame()
        color_f = frames.get_color_frame() if colour else None
        ref = color_f if colour else depth_f
        i = ref.profile.as_video_stream_profile().intrinsics
        intr = {"fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy,
                "width": i.width, "height": i.height}
        scale = 1.0 / profile.get_device().first_depth_sensor().get_depth_scale()
        return (np.asanyarray(color_f.get_data()).copy() if colour else None,
                depths, intr, scale)
    finally:
        pipe.stop()
