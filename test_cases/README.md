# Test Cases

In this folder I've collected test case data which can be used to verify your own code.  What follows are descriptions of the individual test cases, how the data is structured, and how it was produced.

## R-30iB Forward Kinematics

**File: `r30ib_forward_kinematics.json`**

This is a set of test cases which consist of joint values as they would be entered into the LRMate's R-30iB controller and the resulting transformation matrix which represents the pose of the robot end flange when the robot is moved to those joint values.

These are stored `json` consisting of a list of 30 objects.  Each object has:

* `joints`: field which is six decimal values representing the joint angle in *degrees*, in the order of *J1*, *J2*, *J3*...*J6*
* `end`: a field consisting of 16 decimal values representing the elements of the 4x4 transformation matrix from the world origin to the robot end flange pose. The values are in sequential order by column.  The first four values form column 1, the second four are column 2, the third four are column 3, and the last four are column 4.

**Be aware of the special J2/J3 interaction common in FANUC robots.**  On the LRMate and the R-30iB controller, the value of J3 as entered into the controller, and as displayed on the controller, is the angle between J3 and the robot base, *not* the angle between J3 and J2.  The kinematic angle (the angle used by normal kinematic models) is found by adding the values of J2 and J3.

These test cases were randomly generated from an existing Denavit-Hartenberg forward kinematics model based on the documented parameters of the LRMate 200iD.  That model was verified against RoboDK and a real LRMate 200iD when it was created, and has been operating in a production system for many years, but the individual test cases generated here were random joint positions that have not been directly verified.

