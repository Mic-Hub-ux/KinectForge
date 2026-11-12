# KinectForge

Low-cost turntable 3D scanner built on an Xbox 360 Kinect (v1).

## What it does

The object sits on a hand-turned turntable on top of a printed ArUco marker sheet; the Kinect
stays fixed. The markers give the sheet's pose in every frame, and Open3D fuses the RGB-D
frames into a single coloured mesh (TSDF fusion). The output is a PLY with per-vertex colour;
closing the bottom, touch-ups and STL export for printing are done by hand in Blender.

## Status

Work in progress. The processing pipeline is written; capturing from the Kinect is not
implemented yet, so the scanner cannot be used end-to-end yet.

- [x] Printable ArUco sheet (`board.py`)
- [x] Per-frame marker detection and pose estimation (`pose.py`)
- [x] TSDF fusion (`fuse.py`) and mesh cleanup (`clean.py`)
- [ ] Live viewer and capture from the Kinect (`viewer.py`, `capture.py`)
- [ ] First real scan

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

### 4. Kinect driver

```bash
sudo apt install -y freenect python3-freenect libfreenect-dev
```

For USB access without sudo, add a udev rule for the Kinect devices (vendor `045e`,
products `02ae`, `02ad`, `02b0`) with `MODE="0666"`.

## Usage

Step-by-step instructions will be added here once capturing from the Kinect is
implemented. The one step that is ready is the marker sheet.

Every command runs from the repository root with the virtual environment active
(once per terminal):

```bash
cd KinectForge
source .venv/bin/activate      # the prompt now starts with (.venv)
python src/board.py --out board.png
```

Print `board.pdf` on A4 at **100% scale** (not "fit to page"), then measure the black bar
on the sheet: it must be exactly 50 mm, otherwise the print scale is wrong.

Planned setup: object in the centre of the sheet, Kinect 60-70 cm away tilted 30-40° down,
one full turn in 30-60 s, hand on the outer edge of the turntable and out of frame.

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
