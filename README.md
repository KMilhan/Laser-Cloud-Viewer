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

Build:
------
  $ git clone git@github.com:KMilhan/Laser_Cloud_Viewer.git

  $ cd Laser_Cloud_Viewer

  $ mkdir build && cd build

  $ cmake ..

  $ make



Contact: kimmilhan@gmail.com
