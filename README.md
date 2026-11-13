# KinectForge

Low-cost turntable 3D scanner built on an Xbox 360 Kinect (v1).

## What it does

The object sits on a hand-turned turntable on top of a printed ArUco marker sheet; the Kinect
stays fixed. The markers give the sheet's pose in every frame, and Open3D fuses the RGB-D
frames into a single coloured mesh (TSDF fusion). The output is a PLY with per-vertex colour;
closing the bottom, touch-ups and STL export for printing are done by hand in Blender.

One command, `./kinectforge`, runs the whole app in a single window: live setup screen with
tilt motor control, automatic start of the recording once the Kinect and the object are in
the right place, automatic stop after one full turn, and the PLY saved to `scanned_models/`.

## Status

Work in progress. The full pipeline runs, both on synthetic data and live from the Kinect;
the first real scan has not been validated yet.

- [x] Printable ArUco sheet (`board.py`)
- [x] Per-frame marker detection and pose estimation (`pose.py`)
- [x] TSDF fusion (`fuse.py`) and mesh cleanup (`clean.py`)
- [x] Live viewer with setup checks and tilt motor control (`viewer.py`)
- [x] One-command app: setup, tilt, automatic recording, mesh (`./kinectforge`, `capture.py`)
- [ ] First real scan

## Hardware

- Xbox 360 Kinect (v1) **with its mains power adapter** (USB alone does not power the sensor)
- A turntable: anything that spins (a microwave glass plate works)
- The ArUco sheet ([`assets/board.pdf`](assets/board.pdf)) printed on A4 at 100% scale
- A Linux PC

## How it works

```
 capture ──► pose ──────► fuse ──────► clean ──────► PLY ──► Blender
 RGB +       ArUco        TSDF          drop          (hand-made bottom,
 depth (mm)  → sheet      volume        fragments      STL export)
             pose/frame   → mesh, crop
```

1. **Capture**: RGB and depth registered to RGB (640x480, depth in mm) saved as PNGs.
   A frame is saved every ~3° of turntable rotation, measured from the markers.
2. **Pose**: ArUco markers on the sheet → camera pose for each frame via `solvePnP`.
   Frames with too few markers, high reprojection error or no depth on the object are dropped.
3. **Fuse**: frames are integrated into a TSDF volume (2 mm voxels) in the sheet's coordinate
   system.
4. **Mesh**: the surface is extracted and cropped to a 20x20x20 cm box just above the sheet,
   which removes the sheet, the background and the hand.
5. **Clean**: small disconnected fragments are removed and the PLY is written.

The world frame is the sheet: origin at its centre, z pointing up.

## Installation

Tested on Ubuntu 24.04 with Python 3.12. Requirements: Python 3.10+ and `git`.

### 1. Clone the repository

```bash
git clone https://github.com/Mic-Hub-ux/KinectForge.git
cd KinectForge
```

### 2. Install the Kinect driver

```bash
sudo apt update
sudo apt install -y freenect python3-freenect libfreenect-dev
```

### 3. Allow USB access without sudo

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the Kinect after this step.

### 4. Create the virtual environment and install the Python libraries

```bash
sudo apt install -y python3-venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

This installs NumPy, OpenCV (with the ArUco module), Open3D, PyYAML and Pillow.
`--system-site-packages` lets the venv see the Kinect driver installed via apt in step 2.

### 5. Check the installation

```bash
python -c "import freenect, cv2, open3d, numpy; print('ok', cv2.__version__, open3d.__version__)"
```

It should print `ok` followed by the OpenCV and Open3D versions.

## Physical setup

1. Print [`assets/board.pdf`](assets/board.pdf) on A4 at **100% scale** (not "fit to page").
2. Measure the black bar on the sheet: it must be exactly 50 mm, otherwise the print scale
   is wrong.
3. Lay the sheet flat on the turntable and put the object in its centre, inside the ring
   of markers. Fix it with a bit of Blu-Tack so it does not slide.
4. Place the Kinect **60-70 cm** from the object, looking down at **30-40°**.
   The Kinect's own tilt motor only reaches **27°** down, so put the Kinect higher than the
   turntable (a shelf, a stack of books) or tilt its base, then fine-tune with the motor
   (`Tilt` buttons in the app).
5. Use diffuse light and avoid direct sunlight: it blinds the Kinect's infrared projector.

## Usage

### Start the app

From the repository folder:

```bash
./kinectforge
```

That is the only command you need. One window opens and the whole scan happens inside it:
setup, tilt motor, recording, and saving the result. The launcher uses the project's
virtual environment by itself, so there is no need to activate it.

Each scan gets an automatic name like `scan_20261113_154210`. To choose the name yourself:

```bash
./kinectforge --scan mug01
```

### What the window shows

- **Left, colour image**: the ArUco markers that were found and the sheet's axes
  (red = x, green = y, blue = up). The setup checks are in the top-left corner.
- **Right, depth image**: red = near, blue = far, black = no measurement.
  `centre` is the distance at the white cross, `valid` is the share of pixels with depth,
  `motor tilt` is the current angle of the Kinect's motor. The status banner
  (`SETUP`, `STARTING IN 3`, `RECORDING`) and the progress bar are shown here too, so they
  never cover the object.
- **Help panel** at the bottom of the images: a summary of the steps. `Help` hides it.
- **Button bar** at the very bottom: click the buttons with the mouse (keyboard shortcuts
  are listed below).

  | Button | What it does |
  |---|---|
  | `Tilt +5` / `Tilt -5` | moves the Kinect's tilt motor up / down by 5° (setup only) |
  | `Tilt 0` | brings the tilt motor back to 0° (level) |
  | `START now` | starts recording straight away, without waiting for the countdown |
  | `STOP now` | (while recording) stops early and builds the mesh from what was recorded |
  | `Help` | shows / hides the help panel |
  | `Quit` | closes the app; during setup nothing is saved |

### Step by step

1. **Setup.** The window opens with an orange `SETUP` banner. Move the Kinect (use the
   `Tilt +5` / `Tilt -5` buttons for the tilt motor, `Tilt 0` to level it) and the object until
   **all four check lines in the top-left corner are green**:

   | Check line | Means | If it is not green |
   |---|---|---|
   | `markers N  reproj X px` | the sheet is found and its pose is accurate | make sure at least 3 markers are visible and in focus, add light |
   | `distance N cm` | Kinect → sheet centre is 55-75 cm | move the Kinect closer or further |
   | `view angle N deg down` | the Kinect looks down at 25-45° | raise the Kinect or tilt it more (Tilt buttons) |
   | `object: N depth px` | the Kinect sees the object above the sheet | object too close (<60 cm), too small, or shiny/black/transparent |

2. **Countdown.** When everything stays green for 2 seconds the banner turns green
   (`SETUP OK`), then a countdown starts (`STARTING IN 3, 2, 1`). If something turns
   orange during the countdown (e.g. your hand passes in front), it goes back to setup.
   Click `START now` if you want to start straight away.
3. **Recording.** The banner turns red: `RECORDING 120 / 370 deg`. Turn the turntable
   **slowly** by hand (30-60 seconds per turn), holding it by the outer rim and keeping
   your hand out of the picture. A green progress bar shows how much of the turn is done.
   If you see `MARKERS LOST: slow down`, turn more slowly. Stopping and restarting is fine:
   frames are saved by angle, not by time. The tilt motor is locked while recording.
4. **Automatic stop.** After one full turn plus a 10° overlap (370°) recording stops by
   itself. Click `STOP now` to stop earlier (the mesh will have a hole on the side that was
   not seen).
5. **Saving.** The window closes and the terminal builds the mesh (pose estimation, fusion,
   cleanup). It takes from a few seconds to a minute. At the end it prints:

   ```
   Done: .../KinectForge/scanned_models/mug01.ply  → open it in Blender
   ```

If you click `Quit` during setup, the app closes and nothing is saved.

### Open the result

The finished scan is saved automatically to **`scanned_models/<scan name>.ply`** inside the
repository folder. Open it in Blender (`File → Import → Stanford PLY`); to see the colours,
set the viewport shading colour to *Attribute*. From there you close the bottom, fix what
needs fixing and export the STL for printing.

The `scanned_models/` folder is part of the repository, but its contents are ignored by git:
your scans stay on your computer.

### Keyboard shortcuts

Every button also has a key (the window must be in focus):

| Key | During setup | During recording |
|---|---|---|
| `UP` / `DOWN` (or `u` / `d`) | tilt motor +5° / -5° | — (locked) |
| `r` | tilt motor back to 0° (level) | — (locked) |
| `SPACE` | start recording now | stop recording now |
| `h` | show/hide the help panel | — |
| `q` / `ESC` | quit, nothing is saved | stop and keep what was recorded |

### Options

```bash
./kinectforge --scan mug01 --overwrite    # redo a scan with an existing name
./kinectforge --scan mug01 --no-process   # record only, build the mesh later
./kinectforge --help                      # list all options
```

### Advanced: running single steps

These are not needed for normal use. Activate the venv first (`source .venv/bin/activate`).

```bash
python src/viewer.py                      # live preview only, records nothing (+ snapshots with s)
python src/clean.py --scan mug01 --stl    # also write scanned_models/mug01.stl (no colour)
```

To build the mesh by hand from recorded frames (for example after `--no-process`, or after
changing a parameter):

```bash
python src/pose.py  --scan mug01   # pose for every frame → data/raw/mug01/poses.npz
python src/fuse.py  --scan mug01   # → data/meshes/mug01_raw.ply (raw mesh)
python src/clean.py --scan mug01   # → scanned_models/mug01.ply
```

### Trying it without a Kinect

Synthetic frames of a 10 cm sphere (or a cube) can stand in for the sensor:

```bash
.venv/bin/python src/fake_capture.py --scan test01 --shape sphere   # → data/raw/test01/
./kinectforge --scan demo01 --replay test01                         # same app, frames read from disk
```

`--replay` plays the saved frames through the same window (setup, countdown, recording)
and ends with `scanned_models/demo01.ply`, a 10 cm sphere. The tilt motor is not available
in replay mode.

## Where files go

```
data/raw/<scan>/rgb/000001.png     colour frames
data/raw/<scan>/depth/000001.png   depth frames, 16-bit PNG, values in millimetres
data/raw/<scan>/poses.npz          sheet pose for every kept frame
data/meshes/<scan>_raw.ply         raw fused mesh
data/snapshots/                    snapshots from viewer.py
scanned_models/<scan>.ply          final scan (open this one in Blender)
```

`data/` and the contents of `scanned_models/` are not committed to git.

## Configuration

Everything is in `config/board.yaml`:

- `board`: sheet geometry. If you change it, regenerate the sheet with
  `python src/board.py` (it overwrites `assets/board.pdf`) and print it again.
- `camera`: camera intrinsics (Kinect v1 defaults).
- `crop`: the box above the sheet that is kept in the mesh.
- `pose`: frame rejection thresholds.
- `setup`: the distance and view angle ranges that count as green.
- `capture`: how long the setup must stay green, countdown length, degrees between saved
  frames, and the angle at which recording stops.

## Troubleshooting

- **`Kinect not available` / `LIBUSB_ERROR_BUSY`**: another program is using the Kinect
  (close it), or the power adapter is not plugged in.
- **`Virtual environment not found`**: the installation was not completed; follow the
  Installation steps above.
- **`ModuleNotFoundError: freenect`**: the venv was created without
  `--system-site-packages`; delete `.venv` and redo installation step 4.
- **`markers 0: sheet not found`**: the sheet is out of view, too far, too dark or blurred.
- **`object: 0 depth px`**: the object is closer than ~60 cm, or its surface is shiny,
  black or transparent (try a matte spray or a light coat of powder).
- **Mesh looks doubled or blurred**: the turntable was turned too fast; turn more slowly
  and add diffuse light.
- **Background or table in the mesh**: the object is not centred on the sheet.

## Limitations

- Depth noise is about 1-2 mm: no details below 2-3 mm.
- The Kinect v1 cannot measure closer than ~50-60 cm.
- Shiny, transparent and very dark surfaces return little or no depth; direct sunlight
  disturbs the IR projector.
- The bottom of the object is not captured; it is closed by hand in Blender.
- With an A4 sheet the free area in the centre is about 12 cm wide.
- The tilt motor reaches only ±27°.

## Roadmap

- Bottom scan: a second pass with the object flipped, aligned with ICP
- Checkerboard calibration of the camera intrinsics
- Averaging frames in stop-and-go mode
- Possibly a motorised turntable

## License

MIT, see [LICENSE](LICENSE).
