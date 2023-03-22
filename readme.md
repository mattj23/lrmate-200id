# FANUC LRMate 200iD Models/Kinematics

This repository is a collection of tooling, code, and models related to the FANUC LRMate 200iD industrial robot.  The LRMate 200iD is a small, prolific 6-axis serial arm with a 7-kg payload and a 717mm reach introduced in the 2000s.  There are several different models of the 200iD with different payloads and reaches, but this repository currently has information related to the standard 717mm version.

## Contents

### Unified Robotics Description Format (URDF)

The `urdf/` folder contains a URDF created by the [Solidworks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) plugin.  The original model I used was [this one from Grabcad](https://grabcad.com/library/fanuc-lr-mate-200id-1).

To use the URDF format in ROS or with native tooling, you will need to copy the contents of one of the meshes directories to `./urdf/meshes/` so that it can find them.


### Meshes

The `meshes/` folder contains several subdirectories of STL files which are representations of the different links in the robot arm.  All units are in meters.

* The `meshes/med-res/` folder contains the medium resolution STL exports created directly by the Solidworks to URDF plugin described in the [URDF](#unified-robotics-description-format--urdf-) section.  They are exterior triangulations of the [Grabcad model](https://grabcad.com/library/fanuc-lr-mate-200id-1).
* The `meshes/low-res/` folder contains the same STL exports but ones that I've manually thinned and de-featured.  They have significantly reduced triangle counts and some features like holes and inter-component seams have been removed.  The majority of the surfaces will be roughly within 0.5mm of the original medium resolution meshes, except for the de-featured regions.  I created these meshes to use them for more complicated collision detection than the convex bodies.
* The `meshes/convex/` folder contains convex hulls generated from the low resolution meshes by the `scripts/generate_convex.py` script.  These are ideal for fast collision checking.

### Scripts

