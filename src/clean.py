"""Pulisce la mesh grezza di fuse.py e scrive il PLY di consegna. Qui lo scanner finisce.

    python src/clean.py --scan test01          # data/meshes/test01_raw.ply → scanned_models/test01.ply
    python src/clean.py --scan test01 --stl    # (opz.) anche scanned_models/test01.stl, senza colore

Toglie frammenti staccati e triangoli rotti. NON rende la mesh watertight e NON chiude
il fondo: quello si fa a mano in Blender.
"""
import argparse
import sys

import numpy as np
import open3d as o3d

from board import DATA_DIR, OUTPUT_DIR


def main():
    ap = argparse.ArgumentParser(description="Pulizia mesh → PLY di consegna.")
    ap.add_argument("--scan", required=True)
    ap.add_argument("--min-fraction", type=float, default=0.05,
                    help="tieni i pezzi con almeno questa frazione dei triangoli del pezzo più grande")
    ap.add_argument("--smooth", type=int, default=0, help="iterazioni di smoothing Taubin (0 = nessuno)")
    ap.add_argument("--stl", action="store_true", help="esporta anche STL (comodità, senza colore)")
    args = ap.parse_args()

    raw = DATA_DIR / "meshes" / f"{args.scan}_raw.ply"
    if not raw.exists():
        sys.exit(f"Manca {raw}: lancia prima fuse.py --scan {args.scan}")
    mesh = o3d.io.read_triangle_mesh(str(raw))
    n_before = len(mesh.triangles)

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()

    # frammenti staccati (rumore, flying pixel, resti della mano): via quelli piccoli
    cluster_idx, cluster_size, _ = mesh.cluster_connected_triangles()
    cluster_idx, cluster_size = np.asarray(cluster_idx), np.asarray(cluster_size)
    keep = cluster_size >= args.min_fraction * cluster_size.max()
    mesh.remove_triangles_by_mask(~keep[cluster_idx])
    mesh.remove_unreferenced_vertices()

    if args.smooth > 0:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=args.smooth)
    mesh.compute_vertex_normals()

    OUTPUT_DIR.mkdir(exist_ok=True)
    out = OUTPUT_DIR / f"{args.scan}.ply"
    o3d.io.write_triangle_mesh(str(out), mesh)
    ext = mesh.get_axis_aligned_bounding_box().get_extent() * 100
    print(f"Pezzi: {len(cluster_size)} trovati, {keep.sum()} tenuti. "
          f"Triangoli: {n_before} → {len(mesh.triangles)}")
    print(f"Scritto {out}: ingombro {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} cm")

    if args.stl:
        stl = out.with_suffix(".stl")
        mesh.compute_triangle_normals()
        o3d.io.write_triangle_mesh(str(stl), mesh)
        print(f"Scritto {stl} (STL: niente colore)")


if __name__ == "__main__":
    main()
