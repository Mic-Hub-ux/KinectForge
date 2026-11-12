"""Rileva i marker ArUco e stima la posa del foglio (T_cam_board) per ogni frame.

    python src/pose.py --image foto_foglio.jpg   # test su una foto (telefono): stampa la posa
    python src/pose.py --scan obj01              # tutti i frame → data/raw/obj01/poses.npz

poses.npz contiene solo i frame tenuti:
    frame_ids    (N,)     nomi dei frame, es. "000001"
    T_cam_board  (N,4,4)  board → camera, in metri (= extrinsic per Open3D, vedi CLAUDE.md §5.4)
"""
import argparse
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

from board import DATA_DIR, camera_matrix, crop_bounds, load_config, make_board


def estimate_pose(gray, board, detector, K):
    """Posa del foglio in un'immagine. Ritorna dict con T (4x4 o None), n_markers, reproj (px)."""
    corners, ids, _ = detector.detectMarkers(gray)
    res = {"T": None, "n_markers": 0 if ids is None else len(ids), "reproj": np.inf,
           "corners": corners, "ids": ids}
    if ids is None or len(ids) == 0:
        return res
    obj_pts, img_pts = board.matchImagePoints(corners, ids)
    if obj_pts is None or len(obj_pts) < 4:
        return res
    dist = np.zeros(5)
    # IPPE: solver per punti complanari (il foglio è piano, z=0)
    ok, rvec, tvec = cv.solvePnP(obj_pts, img_pts, K, dist, flags=cv.SOLVEPNP_IPPE)
    if not ok:
        return res
    proj, _ = cv.projectPoints(obj_pts, rvec, tvec, K, dist)
    res["reproj"] = float(np.sqrt(np.mean(np.sum((proj - img_pts.reshape(proj.shape)) ** 2, axis=-1))))
    R, _ = cv.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    res["T"], res["rvec"], res["tvec"] = T, rvec, tvec
    return res


def depth_in_box(depth_mm, T, K, lo, hi):
    """Quanti pixel depth validi, riportati in coordinate board, cadono dentro la box di crop."""
    v, u = np.nonzero(depth_mm)
    z = depth_mm[v, u].astype(np.float64) / 1000.0
    p_cam = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], axis=1)
    R, t = T[:3, :3], T[:3, 3]
    p_board = (p_cam - t) @ R          # = R^T (p - t), camera → board
    return int(np.all((p_board >= lo) & (p_board <= hi), axis=1).sum())


def describe(T):
    """Distanza camera-centro foglio e inclinazione della camera rispetto al foglio."""
    dist = np.linalg.norm(T[:3, 3])
    normal_cam = T[:3, 2]                         # asse z del foglio visto dalla camera
    elev = 90 - np.degrees(np.arccos(abs(normal_cam[2])))  # 90° = camera dritta sopra il foglio
    return f"distanza {dist * 100:.1f} cm, asse ottico a {elev:.0f}° dal piano del foglio"


def run_image(path, cfg, focal_px, save):
    img = cv.imread(str(path))
    if img is None:
        sys.exit(f"Non riesco a leggere {path}")
    h, w = img.shape[:2]
    if focal_px is None and (w, h) == (cfg["camera"]["width"], cfg["camera"]["height"]):
        K = camera_matrix(cfg)
        print("Intrinseci: Kinect v1 da config")
    else:
        # foto di telefono: intrinseci veri ignoti → focale tipica (~70° di campo orizzontale)
        f = focal_px if focal_px else 0.75 * max(w, h)
        K = np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]])
        print(f"Intrinseci: stimati per {w}x{h}, f={f:.0f}px "
              f"(distanza indicativa ±10-20%, usa --focal-px se la conosci)")

    board = make_board(cfg)
    detector = cv.aruco.ArucoDetector(board.getDictionary(), cv.aruco.DetectorParameters())
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    res = estimate_pose(gray, board, detector, K)
    print(f"Marker trovati: {res['n_markers']}" +
          ("" if res["ids"] is None else f"  id={sorted(res['ids'].ravel().tolist())}"))
    if res["T"] is None:
        sys.exit("Posa non stimata (servono marker del foglio KinectForge ben visibili).")
    print(f"Errore di reproiezione: {res['reproj']:.2f} px")
    print(f"Posa: {describe(res['T'])}")
    np.set_printoptions(precision=4, suppress=True)
    print("T_cam_board (m):\n", res["T"])

    if save:
        out = path.with_name(path.stem + "_pose.jpg")
        cv.aruco.drawDetectedMarkers(img, res["corners"], res["ids"])
        cv.drawFrameAxes(img, K, np.zeros(5), res["rvec"], res["tvec"], 0.05, max(2, w // 300))
        cv.imwrite(str(out), img)
        print(f"Immagine annotata: {out}  (assi: x rosso, y verde, z blu verso l'alto)")


def run_scan(scan, cfg):
    scan_dir = DATA_DIR / "raw" / scan
    rgb_files = sorted((scan_dir / "rgb").glob("*.png"))
    if not rgb_files:
        sys.exit(f"Nessun frame in {scan_dir / 'rgb'}")
    K = camera_matrix(cfg)
    lo, hi = crop_bounds(cfg)
    pc = cfg["pose"]
    board = make_board(cfg)
    detector = cv.aruco.ArucoDetector(board.getDictionary(), cv.aruco.DetectorParameters())

    kept_ids, kept_T = [], []
    rejected = {"marker": 0, "reproj": 0, "depth": 0}
    for f in rgb_files:
        gray = cv.imread(str(f), cv.IMREAD_GRAYSCALE)
        res = estimate_pose(gray, board, detector, K)
        if res["T"] is None or res["n_markers"] < pc["min_markers"]:
            rejected["marker"] += 1
            continue
        if res["reproj"] > pc["max_reproj_px"]:
            rejected["reproj"] += 1
            continue
        depth = cv.imread(str(scan_dir / "depth" / f.name), cv.IMREAD_UNCHANGED)
        if depth is None or depth_in_box(depth, res["T"], K, lo, hi) < pc["min_depth_px"]:
            rejected["depth"] += 1
            continue
        kept_ids.append(f.stem)
        kept_T.append(res["T"])

    print(f"Frame: {len(rgb_files)} totali, {len(kept_ids)} tenuti, scartati: "
          f"{rejected['marker']} pochi marker (<{pc['min_markers']}), "
          f"{rejected['reproj']} reproiezione >{pc['max_reproj_px']}px, "
          f"{rejected['depth']} poca depth nella box (<{pc['min_depth_px']}px)")
    if not kept_ids:
        sys.exit("Nessun frame valido: poses.npz non scritto.")
    np.savez(scan_dir / "poses.npz", frame_ids=np.array(kept_ids), T_cam_board=np.array(kept_T))
    print(f"Scritto {scan_dir / 'poses.npz'}")

    gt_path = scan_dir / "poses_gt.npz"   # presente solo nei dati sintetici di fake_capture.py
    if gt_path.exists():
        gt = np.load(gt_path)
        gt_T = dict(zip(gt["frame_ids"], gt["T_cam_board"]))
        rot_err, pos_err = [], []
        for fid, T in zip(kept_ids, kept_T):
            G = gt_T[fid]
            dR = T[:3, :3] @ G[:3, :3].T
            rot_err.append(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
            # errore sulla posizione della camera nel sistema board
            pos_err.append(np.linalg.norm(T[:3, :3].T @ T[:3, 3] - G[:3, :3].T @ G[:3, 3]))
        print(f"Confronto con ground truth: rotazione media {np.mean(rot_err):.2f}° "
              f"(max {np.max(rot_err):.2f}°), posizione camera media {np.mean(pos_err) * 1000:.1f} mm "
              f"(max {np.max(pos_err) * 1000:.1f} mm)")


def main():
    ap = argparse.ArgumentParser(description="Stima posa del foglio ArUco.")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--image", type=Path, help="foto singola del foglio")
    g.add_argument("--scan", help="scan_id in data/raw/")
    ap.add_argument("--focal-px", type=float, help="(--image) focale in pixel, se nota")
    ap.add_argument("--no-save", action="store_true", help="(--image) non salvare l'immagine annotata")
    args = ap.parse_args()

    cfg = load_config()
    if args.image:
        run_image(args.image, cfg, args.focal_px, not args.no_save)
    else:
        run_scan(args.scan, cfg)


if __name__ == "__main__":
    main()
