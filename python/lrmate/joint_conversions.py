import open3d
from lrmate.robot import RobotWithMeshes, LinkWithMesh, Transform, Vector, Joint
from pathlib import Path
from typing import List, Optional, Dict, Union


def main():
    # fanuc = {"J1": 0, "J2": -90, "J3": 180, "J4": 0, "J5": 0, "J6": 0}
    # raw = fanuc_to_raw(fanuc)

    raw = {"J1": 0, "J2": 90, "J3": 0,"J4": 0, "J5": 0, "J6": 0}
    fanuc = raw_to_fanuc(raw)


    print(raw)
    print(fanuc)


def fanuc_to_raw(joint_bundle: Dict[str, float]) -> Dict:
    raw_joints = {"J1": joint_bundle["J1"], "J2": -joint_bundle["J2"], "J3": joint_bundle["J2"] + joint_bundle["J3"],
                  "J4": joint_bundle["J4"], "J5": joint_bundle["J5"], "J6": joint_bundle["J6"]}
    return raw_joints


def raw_to_fanuc(joint_bundle: Dict[str, float]) -> Dict:
    fanuc_joints = {"J1": joint_bundle["J1"], "J2": -joint_bundle["J2"], "J3": joint_bundle["J2"] + joint_bundle["J3"],
                    "J4": joint_bundle["J4"], "J5": joint_bundle["J5"], "J6": joint_bundle["J6"]}
    return fanuc_joints


if __name__ == '__main__':
    main()
