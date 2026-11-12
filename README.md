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

Today you can run the full pipeline on synthetic data (see [Quick start](#quick-start-no-kinect-needed)).

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

## Installation

Tested on Ubuntu 24.04 with Python 3.12. Requirements: Python 3.10+, `git`, and the
`python3-venv` package.

### 1. Clone the repository

```bash
git clone https://github.com/Mic-Hub-ux/KinectForge.git
cd KinectForge
```

### 2. Create the virtual environment and install the Python libraries

```bash
sudo apt install -y python3-venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

This installs NumPy, OpenCV (with the ArUco module), Open3D, PyYAML and Pillow.
`--system-site-packages` lets the venv see the Kinect driver installed via apt (step 4).

### 3. Check the installation

```bash
python -c "import cv2, open3d, numpy; print('ok', cv2.__version__, open3d.__version__)"
```

It should print `ok` followed by the OpenCV and Open3D versions.

### 4. Kinect driver (only needed for the hardware scripts, not yet implemented)

```bash
sudo apt install -y freenect python3-freenect libfreenect-dev
```

For USB access without sudo, add a udev rule for the Kinect devices (vendor `045e`,
products `02ae`, `02ad`, `02b0`) with `MODE="0666"`.

## Quick start (no Kinect needed)

This runs the whole pipeline on a synthetic scan: a virtual 5 cm radius sphere on the
ArUco sheet, seen from 36 angles with Kinect-like noise. Run every command from the
repository root with the venv active (`source .venv/bin/activate`).

```bash
# 1. Generate 36 synthetic RGB + depth frames → data/raw/test01/
python src/fake_capture.py --scan test01 --shape sphere

# 2. Find the markers in each frame and compute the camera pose → data/raw/test01/poses.npz
python src/pose.py --scan test01

# 3. Fuse the frames into a mesh → data/meshes/test01_raw.ply
python src/fuse.py --scan test01

# 4. Remove noise fragments → data/meshes/test01.ply  (the final result)
python src/clean.py --scan test01
```

Open `data/meshes/test01.ply` in Blender (*File → Import → Stanford PLY*) or MeshLab.
You should see a coloured sphere with an open bottom. The bottom is open by design: it is
closed by hand in Blender.

Use `--shape cube` for an 8 cm cube instead of a sphere, and `clean.py --stl` to also
write an STL file (without colour).

## Testing

There is no Kinect yet, so the pipeline is tested in two ways.

### Synthetic scan: does the whole pipeline work?

`fake_capture.py` renders the frames from a known object and known camera positions, so
every step can be checked against the truth:

- **`pose.py`** prints a line `Confronto con ground truth: ...`. This compares the camera
  poses it estimated from the markers with the real ones. Expected: rotation error well
  under 1°, camera position error a few mm.
- **`fuse.py` / `clean.py`** print the size of the mesh (`ingombro`). Expected: about
  10 x 10 cm in plan for the sphere (5 cm radius) and about 9 cm in height (the lowest
  part of the sphere is hidden from the camera, so it is missing). For the cube: 10.3 x 10.3 cm (the diagonal of an
  8 cm cube rotated by 20°) and about 7.8 cm in height.

Our run with the sphere measured a radius of 50.1 ± 0.2 mm against the true 50 mm.

### Real photo: does marker detection work on a printed sheet?

```bash
python src/board.py --out board.png
```

1. Print `board.pdf` on A4 at **100% scale** (not "fit to page").
2. Measure the black bar on the sheet: it must be exactly 50 mm. If it isn't, the print
   scale is wrong and every measurement will be too.
3. Take a photo of the sheet at an angle with your phone and run:

```bash
python src/pose.py --image photo.jpg
```

Expected: most of the 16 markers found, reprojection error around 1 px or less, and a
distance close to the real one. The phone's focal length is guessed, so the distance is only
approximate; pass `--focal-px` if you know it. The script also saves `photo_pose.jpg` with
the detected markers and the sheet's axes drawn on it. The blue axis (z) must point up
out of the sheet.

## Scanning with the Kinect (not implemented yet)

```bash
python src/viewer.py                 # live RGB + depth
python src/capture.py --scan obj01   # frames → data/raw/obj01/
# then pose.py, fuse.py and clean.py as in the quick start, with --scan obj01
```

Setup: object in the centre of the sheet, Kinect 60-70 cm away tilted 30-40° down, one
full turn in 30-60 s, hand on the outer edge of the turntable and out of frame.

## Configuration

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
