# KinectForge

Low-cost turntable 3D scanner built on an Xbox 360 Kinect (v1).

## What it does

The object sits on a hand-turned turntable on top of a printed ArUco marker sheet; the Kinect
stays fixed. The markers give the sheet's pose in every frame, and Open3D fuses the RGB-D
frames into a single coloured mesh (TSDF fusion). The output is a PLY with per-vertex colour;
closing the bottom, touch-ups and STL export for printing are done by hand in Blender.

## Status

Work in progress. The software pipeline is complete and tested on synthetic data; the
hardware side is not written yet.

- [x] Printable ArUco sheet (`board.py`)
- [x] Per-frame marker detection and pose estimation (`pose.py`)
- [x] Synthetic RGB-D generator for testing without a Kinect (`fake_capture.py`)
- [x] TSDF fusion (`fuse.py`) and mesh cleanup (`clean.py`)
- [ ] Live viewer and capture from the Kinect (`viewer.py`, `capture.py`)
- [ ] First real scan

On synthetic data (a 50 mm radius sphere, 36 frames, 1 mm depth noise) the reconstructed
radius is 50.1 ± 0.2 mm using poses estimated from the markers.

## Hardware

- Xbox 360 Kinect (v1) **with its mains power adapter** (USB alone does not power the sensor)
- A turntable: anything that spins (a microwave glass plate works)
- The ArUco sheet printed on A4 at 100% scale
- A Linux PC

## How it works

```
 capture ──► pose ──────► fuse ──────► clean ──────► PLY ──► Blender
 RGB +       ArUco        TSDF          drop          (hand-made bottom,
 depth (mm)  → sheet      volume        fragments      STL export)
             pose/frame   → mesh, crop
```

1. **Capture**: RGB and depth registered to RGB (640x480, depth in mm) saved as PNGs.
2. **Pose**: ArUco markers on the sheet → camera pose for each frame via `solvePnP`.
   Frames with too few markers, high reprojection error or no depth on the object are dropped.
3. **Fuse**: frames are integrated into a TSDF volume (2 mm voxels) in the sheet's coordinate
   system.
4. **Mesh**: the surface is extracted and cropped to a 20x20x20 cm box just above the sheet,
   which removes the sheet, the background and the hand.
5. **Clean**: small disconnected fragments are removed and the PLY is written.

The world frame is the sheet: origin at its centre, z pointing up.

## Install

Ubuntu/Debian:

```bash
sudo apt install -y freenect python3-freenect libfreenect-dev
python3 -m venv --system-site-packages .venv   # system-site-packages: freenect comes from apt
source .venv/bin/activate
pip install -r requirements.txt
```

For USB access without sudo, add a udev rule for the Kinect devices (vendor `045e`,
products `02ae`, `02ad`, `02b0`) with `MODE="0666"`.

## Usage

Without a Kinect (synthetic data):

```bash
python src/board.py --out board.png                       # sheet to print (also board.pdf)
python src/pose.py  --image photo_of_sheet.jpg            # pose test on a phone photo
python src/fake_capture.py --scan test01 --shape sphere   # synthetic frames → data/raw/test01/
python src/pose.py  --scan test01                         # marker poses → poses.npz
python src/fuse.py  --scan test01                         # → data/meshes/test01_raw.ply
python src/clean.py --scan test01                         # → data/meshes/test01.ply
```

`fake_capture.py` writes ground-truth poses, so you can also skip `pose.py`; when you run it,
it reports its error against the ground truth.

With a Kinect (not implemented yet):

```bash
python src/viewer.py                 # live RGB + depth
python src/capture.py --scan obj01   # frames → data/raw/obj01/
# then pose.py / fuse.py / clean.py as above with --scan obj01
```

Setup: object in the centre of the sheet, Kinect 60-70 cm away tilted 30-40° down, one
full turn in 30-60 s, hand on the outer edge of the turntable and out of frame.

Board geometry, camera intrinsics, crop box and frame rejection thresholds are in
`config/board.yaml`.

## Limitations

- Depth noise is about 1-2 mm: no details below 2-3 mm.
- The Kinect v1 cannot measure closer than ~50-60 cm.
- Shiny, transparent and very dark surfaces return little or no depth; direct sunlight
  disturbs the IR projector.
- The bottom of the object is not captured; it is closed by hand in Blender.
- With an A4 sheet the free area in the centre is about 12 cm wide.

## Roadmap

- Bottom scan: a second pass with the object flipped, aligned with ICP
- Checkerboard calibration of the camera intrinsics
- Averaging frames in stop-and-go mode
- Possibly a motorised turntable

## License

MIT, see [LICENSE](LICENSE).
