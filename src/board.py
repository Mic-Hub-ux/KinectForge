"""Foglio ArUco di KinectForge: definizione della Board, config condivisa e generazione
del foglio da stampare.

Layout: un anello di marker lungo il bordo del foglio, centro libero per l'oggetto.
Coordinate board in metri, origine al centro del foglio, z verso l'alto (vedi board.yaml).

    python src/board.py      # rigenera assets/board.pdf (+ .png); serve solo se cambi board.yaml
"""
import argparse
from pathlib import Path

import cv2 as cv
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT / "config" / "board.yaml"
DATA_DIR = ROOT / "data"


def load_config(path=CONFIG_PATH):
    with open(path) as f:
        return yaml.safe_load(f)


def camera_matrix(cfg):
    c = cfg["camera"]
    return np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]], dtype=np.float64)


def crop_bounds(cfg):
    """(min, max) della box di crop in coordinate board, metri."""
    c = cfg["crop"]
    h = c["half_xy"]
    return np.array([-h, -h, c["z_min"]]), np.array([h, h, c["z_max"]])


def marker_layout(cfg):
    """Angolo in alto a sinistra (x, y) in mm, origine al centro del foglio, per ogni marker.

    Ordine: riga in alto da sinistra a destra, colonna destra dall'alto in basso,
    riga in basso da destra a sinistra, colonna sinistra dal basso in alto.
    L'id del marker è l'indice in questa lista.
    """
    b = cfg["board"]
    w, h = b["page_mm"]
    m, s = b["margin_mm"], b["marker_mm"]
    nx, ny = b["markers_x"], b["markers_y"]
    left, right = -w / 2 + m, w / 2 - m - s       # x dell'angolo sinistro del marker
    top, bottom = h / 2 - m, -h / 2 + m + s       # y dell'angolo superiore del marker
    xs = np.linspace(left, right, nx)
    ys = np.linspace(top, bottom, ny)

    tl = [(x, top) for x in xs]
    tl += [(right, y) for y in ys[1:]]
    tl += [(x, bottom) for x in xs[::-1][1:]]
    tl += [(left, y) for y in ys[::-1][1:-1]]
    return tl


def make_board(cfg):
    """cv.aruco.Board con objPoints in metri nel sistema board (z=0 sul foglio)."""
    b = cfg["board"]
    s = b["marker_mm"] / 1000.0
    obj_points = []
    for x, y in marker_layout(cfg):
        x, y = x / 1000.0, y / 1000.0
        # angoli in ordine ArUco: TL, TR, BR, BL (y verso l'alto → BR ha y più piccola)
        obj_points.append(np.array([[x, y, 0], [x + s, y, 0], [x + s, y - s, 0], [x, y - s, 0]],
                                   dtype=np.float32))
    dictionary = cv.aruco.getPredefinedDictionary(getattr(cv.aruco, b["dictionary"]))
    ids = np.arange(len(obj_points), dtype=np.int32)
    return cv.aruco.Board(obj_points, dictionary, ids)


def render_sheet(cfg, px_per_mm):
    """Immagine (grayscale) del foglio intero a px_per_mm, pronta da stampare al 100%."""
    b = cfg["board"]
    w, h = b["page_mm"]
    s = b["marker_mm"]
    W, H = round(w * px_per_mm), round(h * px_per_mm)
    img = np.full((H, W), 255, np.uint8)
    dictionary = cv.aruco.getPredefinedDictionary(getattr(cv.aruco, b["dictionary"]))
    side = round(s * px_per_mm)

    def to_px(x, y):  # mm board → pixel (origine in alto a sinistra, y verso il basso)
        return round((x + w / 2) * px_per_mm), round((h / 2 - y) * px_per_mm)

    for marker_id, (x, y) in enumerate(marker_layout(cfg)):
        u, v = to_px(x, y)
        img[v:v + side, u:u + side] = cv.aruco.generateImageMarker(dictionary, marker_id, side)

    # croce al centro del foglio = origine del sistema board
    cu, cv_ = to_px(0, 0)
    arm, th = round(8 * px_per_mm), max(1, round(0.4 * px_per_mm))
    cv.line(img, (cu - arm, cv_), (cu + arm, cv_), 0, th)
    cv.line(img, (cu, cv_ - arm), (cu, cv_ + arm), 0, th)

    # barra di controllo scala: deve misurare esattamente 50 mm sul foglio stampato
    x0, y0 = to_px(-25, -h / 2 + b["margin_mm"] + s + 25)
    cv.rectangle(img, (x0, y0), (x0 + round(50 * px_per_mm), y0 + round(2 * px_per_mm)), 0, -1)
    font, fs = cv.FONT_HERSHEY_SIMPLEX, 0.09 * px_per_mm
    for i, text in enumerate([f"KinectForge - marker {s:g} mm - barra 50 mm",
                              "stampa al 100% (niente 'adatta alla pagina')"]):
        tw = cv.getTextSize(text, font, fs, th)[0][0]
        cv.putText(img, text, (round(W / 2 - tw / 2), y0 + round((7 + 5 * i) * px_per_mm)),
                   font, fs, 0, th, cv.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description="Genera il foglio ArUco da stampare (PNG + PDF).")
    ap.add_argument("--out", default=str(ROOT / "assets" / "board.png"), help="PNG di uscita; accanto scrive il PDF")
    ap.add_argument("--dpi", type=int, default=300)
    args = ap.parse_args()

    cfg = load_config()
    img = render_sheet(cfg, args.dpi / 25.4)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv.imwrite(str(out), img)

    from PIL import Image
    pdf = out.with_suffix(".pdf")
    Image.fromarray(img).save(pdf, resolution=args.dpi)  # pagina della misura esatta del foglio

    b = cfg["board"]
    print(f"{out} + {pdf}: {len(marker_layout(cfg))} marker {b['dictionary']}, "
          f"lato {b['marker_mm']} mm, foglio {b['page_mm'][0]}x{b['page_mm'][1]} mm")
    print("Dopo la stampa misura la barra: deve essere 50 mm. Se no, la scala è sbagliata.")


if __name__ == "__main__":
    main()
