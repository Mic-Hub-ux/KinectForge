"""[HARDWARE] "Hello Kinect": RGB + depth a schermo, per verificare sensore e posizionamento.
Solo anteprima, non registra (la scansione la fa capture.py, che ha la stessa schermata).

    python src/viewer.py                    # finestra live
    python src/viewer.py --snapshot out.png # un frame salvato su file, senza finestra

Contiene anche le funzioni di disegno, i controlli di setup e il motore del tilt usati da
capture.py. I testi a schermo sono in inglese (il repo è pubblico).

Pulsanti sotto le immagini (Tilt +5/-5/0, Snapshot, Help, Quit) oppure tasti:
frecce su/giù (o u/d) tilt ±5° · r tilt a 0° · s snapshot · a marker on/off · h aiuto · q/ESC esci
"""
import argparse
import ctypes
import os
import sys
import time
from pathlib import Path

if os.environ.get("XDG_SESSION_TYPE") == "wayland":
    os.environ["XDG_SESSION_TYPE"] = "x11"   # Qt di OpenCV usa comunque X11 (XWayland): niente warning

import cv2 as cv
import numpy as np

# OpenCV con GUI Qt non porta i font e all'import punta QT_QPA_FONTDIR a una cartella vuota:
# dopo l'import la ripuntiamo ai font di sistema (Qt la legge solo all'apertura della finestra).
if os.path.isdir("/usr/share/fonts/truetype/dejavu"):
    os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"

from board import DATA_DIR, camera_matrix, crop_bounds, load_config, make_board
from pose import depth_in_box, estimate_pose, make_detector

DEPTH_MIN_MM, DEPTH_MAX_MM = 400, 1500   # range della colormap
GREEN, ORANGE, RED, WHITE, YELLOW = (0, 255, 0), (0, 200, 255), (0, 0, 255), (255, 255, 255), (0, 255, 255)

# codici tasto delle frecce: GTK/X11 e Qt
KEYS_UP, KEYS_DOWN = {0xFF52, 0x1000013, ord("u")}, {0xFF54, 0x1000015, ord("d")}

HELP = [
    "VIEWER - preview only: NOTHING is recorded (scanning is done by capture.py)",
    "Left (colour): ArUco markers found + sheet axes (red x, green y, blue z = up)",
    "  each check line: green = OK, orange/red = fix it",
    "  reproj = pose error in pixels (below ~2 px is fine)",
    "Right (depth): red = near, blue = far, black = no measurement",
    "  'centre' = distance at the cross; 'valid' = % of pixels with depth",
    "Buttons below, or keys: UP/DOWN (u/d) tilt +-5   r tilt 0   s snapshot   a markers   h help   q quit",
]


class Tilt:
    """Motore del Kinect via libfreenect_sync (il binding Python non lo espone in modalità sync).
    Usa lo stesso dispositivo già aperto da sync_get_depth/video. Angolo rispetto all'orizzontale,
    positivo = verso l'alto, limite hardware ±27°."""
    LIMIT, STEP = 27, 5

    class _State(ctypes.Structure):
        _fields_ = [("ax", ctypes.c_int16), ("ay", ctypes.c_int16), ("az", ctypes.c_int16),
                    ("angle", ctypes.c_int8), ("status", ctypes.c_int)]

    def __init__(self):
        try:
            self.lib = ctypes.CDLL("libfreenect_sync.so.0.5")
        except OSError:
            self.lib = None
        self.target = self.read()

    def read(self):
        """Angolo attuale letto dal sensore (gradi), o 0 se non disponibile."""
        if self.lib is None:
            return 0
        p = ctypes.POINTER(self._State)()
        if self.lib.freenect_sync_get_tilt_state(ctypes.byref(p), 0) != 0 or not p:
            return 0
        a = p.contents.angle
        return 0 if a == -128 else round(a / 2)   # -128 (0x80) = motore in movimento

    def set(self, deg):
        if self.lib is None:
            return
        self.target = int(np.clip(deg, -self.LIMIT, self.LIMIT))
        self.lib.freenect_sync_set_tilt_degs(self.target, 0)

    def handle_key(self, k):
        """Gestisce frecce/u/d/r. Ritorna True se il tilt è stato cambiato."""
        if k in KEYS_UP:
            self.set(self.target + self.STEP)
        elif k in KEYS_DOWN:
            self.set(self.target - self.STEP)
        elif k == ord("r"):
            self.set(0)
        else:
            return False
        return True


class Scanner:
    """Kinect + foglio: config, detector e controlli di setup condivisi da viewer e capture."""

    def __init__(self):
        self.cfg = load_config()
        self.board = make_board(self.cfg)
        self.detector = make_detector(self.board)
        self.K = camera_matrix(self.cfg)
        self.lo, self.hi = crop_bounds(self.cfg)

    def check(self, depth_mm, rgb):
        """Posa del foglio + controlli di setup. Ritorna (res, checks); checks = lista (testo, ok)."""
        res = estimate_pose(cv.cvtColor(rgb, cv.COLOR_RGB2GRAY), self.board, self.detector, self.K)
        pc, sc = self.cfg["pose"], self.cfg["setup"]
        if res["T"] is None:
            return res, [(f"markers {res['n_markers']}: sheet not found", False)]
        T = res["T"]
        dist_cm = np.linalg.norm(T[:3, 3]) * 100
        tilt = np.degrees(np.arcsin(np.clip(-T[2, 2], -1, 1)))  # asse ottico vs piano del foglio
        n_obj = depth_in_box(depth_mm, T, self.K, self.lo, self.hi)
        (d0, d1), (t0, t1) = sc["dist_cm"], sc["tilt_deg"]
        checks = [
            (f"markers {res['n_markers']}  reproj {res['reproj']:.2f}px",
             res["n_markers"] >= pc["min_markers"] and res["reproj"] <= pc["max_reproj_px"]),
            (f"distance {dist_cm:.0f} cm (ok {d0}-{d1})", d0 <= dist_cm <= d1),
            (f"view angle {tilt:.0f} deg down (ok {t0}-{t1})", t0 <= tilt <= t1),
            (f"object: {n_obj} depth px above the sheet", n_obj >= pc["min_depth_px"]),
        ]
        return res, checks

    def draw(self, depth_mm, rgb, res=None, checks=(), tilt=None):
        """Affianca RGB annotato e depth colorata."""
        bgr = cv.cvtColor(rgb, cv.COLOR_RGB2BGR)
        if res is not None:
            if res["ids"] is not None:
                cv.aruco.drawDetectedMarkers(bgr, res["corners"], res["ids"])
            if res["T"] is not None:
                cv.drawFrameAxes(bgr, self.K, np.zeros(5), res["rvec"], res["tvec"], 0.05)
            for i, (s, ok) in enumerate(checks):
                text(bgr, s, 25 + 25 * i, GREEN if ok else (RED if res["T"] is None else ORANGE))

        dvis = colorize_depth(depth_mm)
        h, w = depth_mm.shape
        cx, cy = w // 2, h // 2
        patch = depth_mm[cy - 2:cy + 3, cx - 2:cx + 3]
        centre = int(np.median(patch[patch > 0])) if np.any(patch > 0) else 0
        cv.drawMarker(dvis, (cx, cy), WHITE, cv.MARKER_CROSS, 20, 2)
        text(dvis, f"centre: {centre / 10:.1f} cm" if centre else "centre: no depth", 25)
        text(dvis, f"valid: {100 * np.mean(depth_mm > 0):.0f}%", 50)
        if tilt is not None:
            text(dvis, f"motor tilt {tilt.target:+d} deg (UP/DOWN, r = 0)", 75)
        return np.hstack([bgr, dvis])


def open_window(title):
    """Finestra senza la toolbar di OpenCV/Qt (frecce di pan, zoom...: non servono e confondono)."""
    cv.namedWindow(title, cv.WINDOW_AUTOSIZE | cv.WINDOW_GUI_NORMAL)


class Buttons:
    """Barra di pulsanti cliccabili sotto le immagini. Ogni pulsante equivale a un tasto:
    il click viene restituito da poll() come se fosse stato premuto quel tasto."""
    H = 56

    def __init__(self, title):
        self.rects, self.pending = [], None
        cv.setMouseCallback(title, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            for (x0, y0, x1, y1), key, enabled in self.rects:
                if enabled and x0 <= x <= x1 and y0 <= y <= y1:
                    self.pending = key

    def draw(self, view, buttons):
        """buttons = lista (etichetta, tasto, abilitato, colore). Ritorna view + barra."""
        h, w = view.shape[:2]
        bar = np.full((self.H, w, 3), 30, np.uint8)
        gap = 8
        bw = (w - gap * (len(buttons) + 1)) // len(buttons)
        self.rects = []
        for i, (label, key, enabled, color) in enumerate(buttons):
            x0, y0 = gap + i * (bw + gap), 8
            x1, y1 = x0 + bw, self.H - 8
            cv.rectangle(bar, (x0, y0), (x1, y1), color if enabled else (60, 60, 60), -1)
            (tw, th), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv.putText(bar, label, (x0 + (bw - tw) // 2, (y0 + y1 + th) // 2), cv.FONT_HERSHEY_SIMPLEX,
                       0.6, WHITE if enabled else (130, 130, 130), 1, cv.LINE_AA)
            self.rects.append(((x0, h + y0, x1, h + y1), key, enabled))
        return np.vstack([view, bar])

    def poll(self, k):
        """Tasto premuto (waitKeyEx) oppure pulsante cliccato."""
        if self.pending is not None:
            k, self.pending = self.pending, None
        return k


BTN_BLUE, BTN_GREEN, BTN_RED, BTN_GREY = (150, 90, 30), (40, 130, 40), (40, 40, 170), (90, 90, 90)


def tilt_buttons(enabled):
    return [("Tilt +5", ord("u"), enabled, BTN_BLUE), ("Tilt -5", ord("d"), enabled, BTN_BLUE),
            ("Tilt 0", ord("r"), enabled, BTN_BLUE)]


def grab():
    import freenect
    d = freenect.sync_get_depth(format=freenect.DEPTH_REGISTERED)
    r = freenect.sync_get_video()
    if d is None or r is None:
        sys.exit("Kinect not available: is the power adapter plugged in? "
                 "Is another program using it (LIBUSB_ERROR_BUSY)?")
    return d[0], r[0]  # depth uint16 mm, rgb uint8 RGB


def colorize_depth(depth_mm):
    d = np.clip((depth_mm.astype(np.float32) - DEPTH_MIN_MM) / (DEPTH_MAX_MM - DEPTH_MIN_MM), 0, 1)
    vis = cv.applyColorMap((255 - d * 255).astype(np.uint8), cv.COLORMAP_JET)  # vicino = rosso
    vis[depth_mm == 0] = 0                                                      # invalido = nero
    return vis


def text(img, s, y, color=WHITE, x=10, scale=0.6, thick=1):
    (tw, th), base = cv.getTextSize(s, cv.FONT_HERSHEY_SIMPLEX, scale, thick)
    cv.rectangle(img, (x - 4, y - th - 4), (x + tw + 4, y + base + 2), (0, 0, 0), -1)  # sfondo leggibile
    cv.putText(img, s, (x, y), cv.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv.LINE_AA)


def draw_help(view, lines=HELP):
    lh = 22
    y0 = view.shape[0] - lh * len(lines) - 10
    overlay = view.copy()
    cv.rectangle(overlay, (0, y0 - 18), (view.shape[1], view.shape[0]), (0, 0, 0), -1)
    cv.addWeighted(overlay, 0.75, view, 0.25, 0, dst=view)
    for i, s in enumerate(lines):
        cv.putText(view, s, (10, y0 + i * lh), cv.FONT_HERSHEY_SIMPLEX, 0.55,
                   YELLOW if i == 0 else WHITE, 1, cv.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="Live RGB + depth from the Kinect v1 (preview only).")
    ap.add_argument("--snapshot", type=Path, help="save one annotated frame and exit (no window)")
    args = ap.parse_args()

    sc = Scanner()
    import freenect
    try:
        if args.snapshot:
            for _ in range(5):  # i primi frame dopo l'avvio sono spesso vuoti/sovraesposti
                depth, rgb = grab()
            view = sc.draw(depth, rgb, *sc.check(depth, rgb), tilt=Tilt())
            draw_help(view)
            cv.imwrite(str(args.snapshot), view)
            print(f"saved {args.snapshot}")
            return

        grab()                 # apre il dispositivo prima di leggere il tilt
        tilt = Tilt()
        title = "KinectForge viewer"
        open_window(title)
        buttons = Buttons(title)
        show_markers, show_help = True, True
        snap_dir = DATA_DIR / "snapshots"
        while True:
            depth, rgb = grab()
            if show_markers:
                view = sc.draw(depth, rgb, *sc.check(depth, rgb), tilt=tilt)
            else:
                view = sc.draw(depth, rgb, tilt=tilt)
            shown = view.copy()
            if show_help:
                draw_help(shown)
            shown = buttons.draw(shown, tilt_buttons(True) + [
                ("Snapshot", ord("s"), True, BTN_GREY), ("Help", ord("h"), True, BTN_GREY),
                ("Quit", ord("q"), True, BTN_RED)])
            cv.imshow(title, shown)
            k = buttons.poll(cv.waitKeyEx(1))
            if k == -1 or tilt.handle_key(k):
                continue
            k &= 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("h"):
                show_help = not show_help
            if k == ord("a"):
                show_markers = not show_markers
            if k == ord("s"):
                snap_dir.mkdir(parents=True, exist_ok=True)
                stem = time.strftime("%Y%m%d_%H%M%S")
                cv.imwrite(str(snap_dir / f"{stem}_view.png"), view)
                cv.imwrite(str(snap_dir / f"{stem}_depth.png"), depth)  # 16-bit mm
                print(f"snapshot → {snap_dir}/{stem}_*.png")
    finally:
        freenect.sync_stop()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()
