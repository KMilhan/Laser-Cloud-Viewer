Laser_Cloud_Viewer
==================

View, Merge, Segment, Noise Remove Laser Scanned Point Cloud. Based on PCL and its dependencies and FLTK.

Currently, parameters are fine-tuned for 3D scanned ASCII data file from "LEICA C10", though you can adjust them as you wish.

How To Use
==========
Dependency:
-----------
PCL(and its dependencies: Boost, Eigen, Flann, VTK, and more)

FLTK for GUI App

OpenMP(Optional)

You must install PCL along with FLTK before building. On Debian-based systems,
`sudo apt-get install libpcl-dev libfltk1.3-dev` should do the trick.

KITTI dataset loader is included to parse .bin files and project points to camera images using calibration matrices.

Build:
------
```bash
  $ git clone git@github.com:KMilhan/Laser_Cloud_Viewer.git # Or HTTPS protocol if you wish
```
```bash
  $ cd Laser_Cloud_Viewer

  $ mkdir build && cd build

  $ cmake ..

  $ make [-j]
```

Automation
----------
This repository includes a `Makefile` providing common tasks:

```bash
make            # build the project
make test       # run unit tests
make lint       # run cpplint on sources
make format     # apply clang-format
make clean      # remove the build directory
```


Contact: kimmilhan@gmail.com
