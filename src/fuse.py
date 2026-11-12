"""Fonde i frame RGB-D con pose note (TSDF) e ritaglia la mesh alla box sopra il foglio.

    python src/fuse.py --scan test01     # → data/meshes/test01_raw.ply

Legge data/raw/<scan_id>/{rgb,depth,poses.npz}: non sa (e non deve sapere) se i frame
vengono da capture.py o da fake_capture.py. La mesh grezza poi passa a clean.py.
"""
import argparse
import sys

import cv2 as cv
import numpy as np
import open3d as o3d

from board import DATA_DIR, crop_bounds, load_config


def main():
    ap = argparse.ArgumentParser(description="TSDF fusion → mesh grezza ritagliata.")
    ap.add_argument("--scan", required=True)
    ap.add_argument("--voxel", type=float, default=0.002, help="lato voxel TSDF, m")
    args = ap.parse_args()

    cfg = load_config()
    scan_dir = DATA_DIR / "raw" / args.scan
    poses_path = scan_dir / "poses.npz"
    if not poses_path.exists():
        sys.exit(f"Manca {poses_path}: lancia prima pose.py --scan {args.scan}")
    poses = np.load(poses_path)

    c = cfg["camera"]
    intr = o3d.camera.PinholeCameraIntrinsic(c["width"], c["height"], c["fx"], c["fy"], c["cx"], c["cy"])
    # Volume uniforme (cubo) che copre la box di crop più un margine sotto il foglio.
    # Niente ScalableTSDFVolume: in Open3D 0.20 estrae mesh vuote.
    lo, hi = crop_bounds(cfg)
    margin = 0.02
    origin = np.array([lo[0], lo[1], -margin])
    length = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] + margin)
    volume = o3d.pipelines.integration.UniformTSDFVolume(
        length=length,
        resolution=int(np.ceil(length / args.voxel)),
        sdf_trunc=4 * args.voxel,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        origin=origin)

    for fid, T in zip(poses["frame_ids"], poses["T_cam_board"]):
        bgr = cv.imread(str(scan_dir / "rgb" / f"{fid}.png"))
        depth = cv.imread(str(scan_dir / "depth" / f"{fid}.png"), cv.IMREAD_UNCHANGED)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(cv.cvtColor(bgr, cv.COLOR_BGR2RGB))),
            o3d.geometry.Image(depth),
            depth_scale=1000.0,      # PNG in mm → metri, come le pose
            depth_trunc=1.0,
            convert_rgb_to_intensity=False)
        # T_cam_board è già world→camera: è l'extrinsic che Open3D vuole (CLAUDE.md §5.4)
        volume.integrate(rgbd, intr, T)
    print(f"Integrati {len(poses['frame_ids'])} frame")

    mesh = volume.extract_triangle_mesh()
    mesh = mesh.crop(o3d.geometry.AxisAlignedBoundingBox(lo, hi))
    if len(mesh.triangles) == 0:
        sys.exit("Mesh vuota dopo il crop: controlla pose (§5.4) e box di crop in board.yaml.")
    mesh.compute_vertex_normals()

    out = DATA_DIR / "meshes" / f"{args.scan}_raw.ply"
    out.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out), mesh)
    ext = mesh.get_axis_aligned_bounding_box().get_extent() * 100
    print(f"Scritto {out}: {len(mesh.vertices)} vertici, {len(mesh.triangles)} triangoli, "
          f"ingombro {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} cm")


if __name__ == "__main__":
    main()
