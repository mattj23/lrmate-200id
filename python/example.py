import open3d
from lrmate.robot import Robot, Link, Transform, Vector, Joint
from pathlib import Path
from lrmate.joint_conversions import fanuc_to_raw, raw_to_fanuc


def main():
    root_path = Path.cwd().parent
    robot = Robot(root_path.joinpath("urdf", "urdf", "LRMate-200iD.urdf"),
                  mesh_path=root_path.joinpath("meshes", "med-res"),
                  collision_mesh_path=root_path.joinpath("meshes", "convex"))

    # Note that these are raw degrees from each joint's geometric zero position, and are not the same as the values
    # which would be put into the R-30iB controller to achieve the same pose. There is a direct conversion, but it
    # requires handling the J2/J3 joint interaction.

    """for raw to fanuc"""
    # raw_joints = {"J1": 13, "J2": 82, "J3": 12,"J4": 30, "J5": 17, "J6": 0}
    # robot.set_joints_deg(raw_joints)
    # fanuc_joints = raw_to_fanuc(raw_joints)
    # print(f"Enter these joint values into RDK: {fanuc_joints}\n")

    """for fanuc to raw"""
    fanuc_joints = {"J1": 13, "J2": 82, "J3": 12,"J4": 30, "J5": 17, "J6": 0}
    raw_joints = fanuc_to_raw(fanuc_joints)
    robot.set_joints_deg(raw_joints)


    links = [link.mesh for link in robot.links.values()]
    print(links)
    crosses = [link.cross for link in robot.links.values()]
    to_draw = links + crosses

    end_transform = robot.end_link.current_transform
    print(end_transform.matrix)
    open3d.visualization.draw_geometries(to_draw)


if __name__ == '__main__':
    main()
