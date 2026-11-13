"""[HARDWARE] Scansione completa in un solo comando: setup → registrazione → mesh.

    ./kinectforge                       # avvio dell'app (launcher nella root del repo)
    python src/capture.py               # uguale; nome scansione automatico: scan_AAAAMMGG_HHMMSS
    python src/capture.py --scan obj01  # nome scelto

1. SETUP: stessa schermata del viewer. Sistemi Kinect (anche col motore del tilt, frecce
   su/giù) e oggetto finché tutte le righe di controllo sono verdi.
2. Quando è tutto verde per qualche secondo parte un conto alla rovescia, poi la
   REGISTRAZIONE parte da sola (oppure SPAZIO per partire subito).
3. Giri il piatto a mano: un frame viene salvato ogni ~3° di rotazione, letta dai marker.
   Dopo un giro completo si ferma da solo (oppure SPAZIO per fermare prima).
4. Alla fine lancia pose.py → fuse.py → clean.py: il risultato è scanned_models/<scan>.ply.

Frame salvati in data/raw/<scan>/rgb/000001.png (RGB) e depth/000001.png (16-bit, mm),
stesso formato di fake_capture.py. Parametri in config/board.yaml (setup:, capture:).
Testi a schermo e messaggi in inglese (il repo è pubblico).

    --replay test01   usa frame già su disco invece del Kinect (per provare senza sensore)
    --no-process      registra soltanto, senza pose/fuse/clean alla fine
"""
import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2 as cv
import numpy as np

from board import DATA_DIR, OUTPUT_DIR
from viewer import (BTN_GREEN, BTN_GREY, BTN_RED, GREEN, ORANGE, RED, WHITE, YELLOW, Buttons, Scanner,
                    Tilt, draw_help, grab, open_window, text, tilt_buttons)

SRC = Path(__file__).resolve().parent
HELP = [
    "SCAN - setup first, then recording starts by itself",
    "1. Adjust Kinect (Tilt buttons or UP/DOWN keys) and object until all checks are green",
    "2. All green for a couple of seconds -> countdown -> recording starts",
    "3. Turn the turntable by hand, slowly (30-60 s per turn), hand on the rim, out of view",
    "4. After one full turn it stops by itself and builds the mesh",
    "Buttons below, or keys: SPACE start/stop   h help   q quit (during setup nothing is saved)",
]


def frames_live():
    while True:
        yield grab()


def frames_replay(scan, lead_s):
    """Frame da disco a ~30 fps; il primo ripetuto per lead_s secondi (tempo per setup/countdown)."""
    d = DATA_DIR / "raw" / scan
    files = sorted((d / "rgb").glob("*.png"))
    if not files:
        sys.exit(f"No frames in {d / 'rgb'}")
    load = lambda f: (cv.imread(str(d / "depth" / f.name), cv.IMREAD_UNCHANGED),
                      cv.cvtColor(cv.imread(str(f)), cv.COLOR_BGR2RGB))
    first = load(files[0])
    for _ in range(int(lead_s * 30)):
        time.sleep(1 / 30)
        yield first
    for f in files + files[:3]:   # un po' oltre il giro, come farebbe chi gira il piatto
        time.sleep(1 / 30)
        yield load(f)


def plate_angle(T, R0):
    """Rotazione del foglio attorno al suo asse z rispetto alla posa di partenza, in gradi."""
    Rrel = R0.T @ T[:3, :3]
    return np.degrees(np.arctan2(Rrel[1, 0], Rrel[0, 0]))


def banner(view, s, color, sub=None):
    """Stato in alto sul pannello depth (sopra c'è solo sfondo: non copre oggetto e marker)."""
    w = view.shape[1] // 2
    (tw, _), _ = cv.getTextSize(s, cv.FONT_HERSHEY_SIMPLEX, 1.1, 2)
    text(view, s, 105, color, x=w + (w - tw) // 2, scale=1.1, thick=2)
    if sub:
        (sw, _), _ = cv.getTextSize(sub, cv.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        text(view, sub, 140, WHITE, x=w + (w - sw) // 2)


def progress(view, frac):
    w = view.shape[1] // 2
    x0, x1, y = w + 10, 2 * w - 10, 165
    cv.rectangle(view, (x0, y - 8), (x1, y + 8), (60, 60, 60), -1)
    cv.rectangle(view, (x0, y - 8), (x0 + int((x1 - x0) * min(frac, 1)), y + 8), GREEN, -1)


def main():
    ap = argparse.ArgumentParser(description="Setup + scan + mesh in one command.")
    ap.add_argument("--scan", help="scan name, e.g. mug01 (default: scan_YYYYMMDD_HHMMSS)")
    ap.add_argument("--overwrite", action="store_true", help="overwrite an existing scan")
    ap.add_argument("--no-process", action="store_true", help="record only, skip pose/fuse/clean")
    ap.add_argument("--replay", metavar="SCAN", help="use the frames in data/raw/SCAN instead of the Kinect")
    args = ap.parse_args()
    args.scan = args.scan or time.strftime("scan_%Y%m%d_%H%M%S")

    out = DATA_DIR / "raw" / args.scan
    if args.replay == args.scan:
        sys.exit("--replay and --scan must be different")
    if out.exists() and any(out.iterdir()) and not args.overwrite:
        sys.exit(f"{out} already exists: pick another --scan name or add --overwrite")

    print(f"KinectForge - scan '{args.scan}'. Window open: set up, it starts recording by itself "
          f"(h = help, q = quit).")
    sc = Scanner()
    cc, pc = sc.cfg["capture"], sc.cfg["pose"]
    if args.replay:
        source, tilt = frames_replay(args.replay, cc["hold_s"] + cc["countdown_s"] + 1), None
    else:
        source = frames_live()
        grab()            # apre il dispositivo prima di leggere il tilt
        tilt = Tilt()

    title = f"KinectForge - {args.scan}"
    open_window(title)
    buttons = Buttons(title)
    state, show_help = "setup", True
    ready_since = countdown_from = None
    R0 = prev = None
    cum = last_saved = 0.0
    n_saved = n_lost = 0
    t_start = None

    def save(depth, rgb):
        nonlocal n_saved
        n_saved += 1
        name = f"{n_saved:06d}.png"
        cv.imwrite(str(out / "rgb" / name), cv.cvtColor(rgb, cv.COLOR_RGB2BGR))
        cv.imwrite(str(out / "depth" / name), depth)

    def start_recording():
        nonlocal state, t_start
        if out.exists():
            shutil.rmtree(out)
        (out / "rgb").mkdir(parents=True)
        (out / "depth").mkdir()
        state, t_start = "rec", time.monotonic()
        print(f"Recording started → {out}", flush=True)

    try:
        for depth, rgb in source:
            now = time.monotonic()
            res, checks = sc.check(depth, rgb)
            pose_ok = (res["T"] is not None and res["n_markers"] >= pc["min_markers"]
                       and res["reproj"] <= pc["max_reproj_px"])
            view = sc.draw(depth, rgb, res, checks if state != "rec" else checks[:1],
                           tilt=tilt if state != "rec" else None)

            if state == "setup":
                if all(ok for _, ok in checks):
                    ready_since = ready_since or now
                    if now - ready_since >= cc["hold_s"]:
                        state, countdown_from = "countdown", now
                    else:
                        banner(view, "SETUP OK", GREEN, "hold still...")
                else:
                    ready_since = None
                    banner(view, "SETUP", ORANGE, "fix the orange/red lines")

            if state == "countdown":
                left = cc["countdown_s"] - (now - countdown_from)
                if not all(ok for _, ok in checks):
                    state, ready_since = "setup", None
                elif left <= 0:
                    start_recording()
                else:
                    banner(view, f"STARTING IN {int(np.ceil(left))}", YELLOW, "get ready to turn the plate")

            if state == "rec":
                if pose_ok:
                    if R0 is None:
                        R0, prev = res["T"][:3, :3], 0.0
                        save(depth, rgb)
                    else:
                        a = plate_angle(res["T"], R0)
                        cum += (a - prev + 180) % 360 - 180   # delta dal frame prima, senza salti a ±180
                        prev = a
                        if abs(cum - last_saved) >= cc["step_deg"]:
                            save(depth, rgb)
                            last_saved = cum
                else:
                    n_lost += 1
                done = abs(cum) / cc["stop_deg"]
                banner(view, f"RECORDING  {abs(cum):.0f} / {cc['stop_deg']} deg", RED,
                       f"frames saved {n_saved}" + ("" if pose_ok else "  -  MARKERS LOST: slow down"))
                progress(view, done)
                if done >= 1:
                    break

            if show_help and state != "rec":
                draw_help(view, HELP)
            rec = state == "rec"
            view = buttons.draw(view, tilt_buttons(tilt is not None and not rec) + [
                ("STOP now" if rec else "START now", ord(" "), True, BTN_RED if rec else BTN_GREEN),
                ("Help", ord("h"), not rec, BTN_GREY),
                ("Quit", ord("q"), True, BTN_GREY)])
            cv.imshow(title, view)
            k = buttons.poll(cv.waitKeyEx(1))
            if k == -1:
                continue
            if tilt is not None and state != "rec" and tilt.handle_key(k):
                state, ready_since = "setup", None     # il Kinect si sta muovendo: ricontrolla tutto
                continue
            k &= 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("h"):
                show_help = not show_help
            if k == ord(" "):
                if state == "rec":
                    break
                start_recording()
    finally:
        if not args.replay:
            import freenect
            freenect.sync_stop()
        cv.destroyAllWindows()

    if state != "rec":
        print("Quit during setup: nothing saved.")
        return
    secs = time.monotonic() - t_start
    print(f"Recording finished: {n_saved} frames in {secs:.0f} s, {abs(cum):.0f}° covered, "
          f"{n_lost} frames without a pose (not saved)")
    if abs(cum) < 330:
        print("WARNING: incomplete turn, the mesh will have holes on the side that was not seen.")
    if args.no_process or n_saved == 0:
        print(f"To build the mesh: python src/pose.py --scan {args.scan} && "
              f"python src/fuse.py --scan {args.scan} && python src/clean.py --scan {args.scan}")
        return
    for step in ("pose", "fuse", "clean"):
        print(f"\n== {step}.py ==", flush=True)
        if subprocess.run([sys.executable, str(SRC / f"{step}.py"), "--scan", args.scan]).returncode:
            sys.exit(f"{step}.py failed: the frames are still in {out}")
    print(f"\nDone: {OUTPUT_DIR / (args.scan + '.ply')}  → open it in Blender")


if __name__ == "__main__":
    main()
