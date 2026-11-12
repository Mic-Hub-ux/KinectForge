"""Genera frame RGB-D sintetici di un oggetto noto sul foglio ArUco, nello stesso formato
di capture.py. Serve a collaudare pose.py / fuse.py / clean.py senza Kinect.

    python src/fake_capture.py --scan test01 --shape sphere

Scena: foglio ArUco (A4 da config) su un piatto grigio, oggetto al centro; la camera
gira attorno (equivalente al piatto che gira con il Kinect fermo).

Scrive in data/raw/<scan_id>/:
    rgb/000001.png     RGB 640x480
    depth/000001.png   uint16, mm, 0 = nessuna misura (rumore ~1 mm + pixel spuri)
    poses.npz          pose vere (stesso formato di pose.py)
    poses_gt.npz       copia delle pose vere: pose.py la usa per misurare il proprio errore
"""
import argparse
import shutil

import cv2 as cv
import numpy as np
import open3d as o3d

from board import DATA_DIR, camera_matrix, load_config, render_sheet

SPHERE_RADIUS = 0.05   # m
CUBE_SIDE = 0.08       # m


def make_object(shape):
    if shape == "sphere":
        m = o3d.geometry.TriangleMesh.create_sphere(SPHERE_RADIUS, resolution=120)
        m.translate((0, 0, SPHERE_RADIUS))
    else:
        s = CUBE_SIDE
        m = o3d.geometry.TriangleMesh.create_box(s, s, s)
        m.translate((-s / 2, -s / 2, 0))
        m.rotate(o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.radians(20))), center=(0, 0, 0))
    return m


def look_at(eye, target):
    """T_cam_board per una camera in `eye` che guarda `target` (convenzione OpenCV: y in giù)."""
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])          # righe = assi camera in coordinate board
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ eye
    return T


def render(scene, obj_id, T, K, w, h, sheet_tex, sheet_mm, rng, outliers):
    """Raycasting di un frame. Ritorna (rgb uint8 HxWx3, depth uint16 mm)."""
    R, t = T[:3, :3], T[:3, 3]
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    d_cam = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    d_board = d_cam @ R                       # R^T d
    eye = -R.T @ t
    rays = np.concatenate([np.broadcast_to(eye, d_board.shape), d_board], -1).astype(np.float32)
    hit = scene.cast_rays(o3d.core.Tensor(rays))
    # con d_cam.z = 1, t_hit coincide con la profondità z in camera
    z = hit["t_hit"].numpy()
    gid = hit["geometry_ids"].numpy()
    normals = hit["primitive_normals"].numpy()
    valid = np.isfinite(z) & (z > 0.5) & (z < 4.0)    # range utile del Kinect v1
    p = eye + d_board * np.where(valid, z, 0)[..., None]

    # colore: foglio stampato (texture) sul piatto grigio; oggetto a strisce colorate
    rgb = np.full((h, w, 3), 90.0)
    W_mm, H_mm = sheet_mm
    tex_h, tex_w = sheet_tex.shape
    map_x = ((p[..., 0] * 1000 + W_mm / 2) / W_mm * tex_w - 0.5).astype(np.float32)
    map_y = ((H_mm / 2 - p[..., 1] * 1000) / H_mm * tex_h - 0.5).astype(np.float32)
    on_sheet = valid & (gid != obj_id) & (map_x >= 0) & (map_x < tex_w) & (map_y >= 0) & (map_y < tex_h)
    sampled = cv.remap(sheet_tex, map_x, map_y, cv.INTER_LINEAR).astype(float)
    rgb[on_sheet] = sampled[on_sheet, None] * 0.9    # carta un po' meno che bianca pura

    on_obj = valid & (gid == obj_id)
    hue = (np.degrees(np.arctan2(p[..., 1], p[..., 0])) % 360) / 2       # tinta = angolo attorno a z
    stripes = (np.floor(p[..., 2] / 0.01) % 2) * 60 + 150                 # bande ogni 1 cm di altezza
    hsv = np.stack([hue, np.full_like(hue, 200), stripes], -1).astype(np.uint8)
    obj_rgb = cv.cvtColor(hsv, cv.COLOR_HSV2RGB).astype(float)
    shade = 0.4 + 0.6 * np.abs(np.sum(normals * d_board / np.linalg.norm(d_board, axis=-1, keepdims=True), -1))
    rgb[on_obj] = obj_rgb[on_obj] * shade[on_obj, None]
    rgb[~valid] = 40
    rgb = np.clip(rgb + rng.normal(0, 2, rgb.shape), 0, 255).astype(np.uint8)

    depth = np.where(valid, z * 1000 + rng.normal(0, 1.0, z.shape), 0)
    # "flying pixel" del Kinect: misure spurie sparse davanti alla scena
    fly = rng.random(z.shape) < outliers
    depth[fly] = rng.uniform(500, 1000, fly.sum())
    return rgb, np.round(depth).astype(np.uint16)


def main():
    ap = argparse.ArgumentParser(description="Frame RGB-D sintetici con pose note.")
    ap.add_argument("--scan", required=True)
    ap.add_argument("--shape", choices=["sphere", "cube"], default="sphere")
    ap.add_argument("--frames", type=int, default=36, help="frame su un giro completo")
    ap.add_argument("--distance", type=float, default=0.65, help="camera-centro foglio, m")
    ap.add_argument("--elevation", type=float, default=35, help="inclinazione camera, gradi")
    ap.add_argument("--outliers", type=float, default=0.002, help="frazione di pixel depth spuri")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    cfg = load_config()
    K = camera_matrix(cfg)
    w, h = cfg["camera"]["width"], cfg["camera"]["height"]
    sheet_mm = cfg["board"]["page_mm"]
    sheet_tex = render_sheet(cfg, px_per_mm=2)   # ~1,2 mm per pixel camera: 2 px/mm basta

    scene = o3d.t.geometry.RaycastingScene()
    plate = o3d.geometry.TriangleMesh.create_box(1.0, 1.0, 0.01)
    plate.translate((-0.5, -0.5, -0.01))         # faccia superiore a z=0
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(plate))
    obj_id = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(make_object(args.shape)))

    out = DATA_DIR / "raw" / args.scan
    if out.exists():
        shutil.rmtree(out)
    (out / "rgb").mkdir(parents=True)
    (out / "depth").mkdir()

    rng = np.random.default_rng(args.seed)
    el = np.radians(args.elevation)
    target = np.array([0, 0, 0.04])
    ids, poses = [], []
    for i in range(args.frames):
        az = 2 * np.pi * i / args.frames
        eye = target + args.distance * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        T = look_at(eye, target)
        rgb, depth = render(scene, obj_id, T, K, w, h, sheet_tex, sheet_mm, rng, args.outliers)
        name = f"{i + 1:06d}"
        cv.imwrite(str(out / "rgb" / f"{name}.png"), cv.cvtColor(rgb, cv.COLOR_RGB2BGR))
        cv.imwrite(str(out / "depth" / f"{name}.png"), depth)
        ids.append(name)
        poses.append(T)

    for fname in ("poses.npz", "poses_gt.npz"):
        np.savez(out / fname, frame_ids=np.array(ids), T_cam_board=np.array(poses))
    size = f"raggio {SPHERE_RADIUS * 100:g} cm" if args.shape == "sphere" else f"lato {CUBE_SIDE * 100:g} cm"
    print(f"{args.frames} frame in {out}  ({args.shape}, {size}, camera a {args.distance * 100:g} cm, "
          f"{args.elevation:g}°)")


if __name__ == "__main__":
    main()
