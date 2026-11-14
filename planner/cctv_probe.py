"""Can this computer read a CCTV recorder's live streams? One snapshot per camera.

    python cctv_probe.py 172.16.1.10 --user viewer --channels 301 1201
    python cctv_probe.py 10.0.0.20 --user viewer --path "/cam/realmonitor?channel={ch}&subtype=0"

Asks for the password (not shown, not saved, not put on the command line). Prints each
stream's size and frame rate and writes probe_<channel>.jpg, so you can see which number
is which camera. The password goes into the address URL-encoded, so '@', '#' or ':' in it
do not break it.

THE ADDRESS
    Hikvision recorders (the default --path):  rtsp://<host>:554/Streaming/Channels/<ch>
    where <ch> is the camera number then 01 for the full stream or 02 for a small one:
    camera 3 -> 301, camera 12 -> 1201. Other makes use other paths: give --path with {ch}.

IF IT FAILS
    'cannot reach'      the computer is not on the recorder's network (cable, address)
    'refused/no stream' wrong user, password or channel -- recorders lock an account for a
                        while after several wrong passwords, so check before retrying
"""
import argparse
import getpass
import os
import socket
import sys
import time
from urllib.parse import quote

# TCP, not the UDP FFmpeg tries first: recorders and firewalls drop UDP far more often.
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|timeout;8000000")

import cv2  # noqa: E402


def reachable(host, port, timeout=3.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("host", help="the recorder's IP address")
    ap.add_argument("--user", required=True)
    ap.add_argument("--channels", nargs="+", default=["101"],
                    help="stream numbers, e.g. 301 1201 1301 (default 101)")
    ap.add_argument("--port", type=int, default=554)
    ap.add_argument("--path", default="/Streaming/Channels/{ch}",
                    help="the stream's path, with {ch} for the channel (default: Hikvision)")
    ap.add_argument("--out", default=".", help="where to write the snapshots")
    args = ap.parse_args(argv)

    if not reachable(args.host, args.port):
        print("cannot reach %s:%d -- this computer is not on the recorder's network "
              "(check the cable and this computer's address)" % (args.host, args.port))
        return 2
    print("%s:%d answers" % (args.host, args.port))
    pw = getpass.getpass("password for %s: " % args.user)
    cred = "%s:%s@" % (quote(args.user, safe=""), quote(pw, safe=""))

    ok_all = True
    for ch in args.channels:
        path = args.path.format(ch=ch)
        url = "rtsp://%s%s:%d%s" % (cred, args.host, args.port, path)
        shown = "rtsp://%s:***@%s:%d%s" % (args.user, args.host, args.port, path)
        t0 = time.time()
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        frame = None
        if cap.isOpened():
            for _ in range(15):                 # the first frames of an H.265 stream can be grey
                ok, f = cap.read()
                if ok:
                    frame = f
        fps = cap.get(cv2.CAP_PROP_FPS) if cap.isOpened() else 0
        cap.release()
        if frame is None:
            ok_all = False
            print("%-6s refused/no stream  %s" % (ch, shown))
            continue
        out = os.path.join(args.out, "probe_%s.jpg" % ch)
        cv2.imwrite(out, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        print("%-6s OK  %d x %d at %.0f fps (%.1f s to first frames)  -> %s"
              % (ch, frame.shape[1], frame.shape[0], fps, time.time() - t0, out))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
