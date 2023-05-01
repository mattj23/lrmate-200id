import open3d
from lrmate.robot import Robot, Link, Transform, Vector, Joint
from pathlib import Path


def main():
    root_path = Path.cwd().parent
    robot = Robot(root_path.joinpath("urdf", "urdf", "LRMate-200iD.urdf"),
                  mesh_path=root_path.joinpath("meshes", "med-res"),
                  collision_mesh_path=root_path.joinpath("meshes", "convex"))

    # Note that these are raw degrees from each joint's geometric zero position, and are not the same as the values
    # which would be put into the R-30iB controller to achieve the same pose. There is a direct conversion, but it
    # requires handling the J2/J3 joint interaction.
    robot.set_joints_deg({"J1": 30, "J2": 30, "J3": 30, "J4": 30, "J5": 30, "J6": 30})

    links = [link.mesh for link in robot.links.values()]
    crosses = [link.cross for link in robot.links.values()]
    to_draw = links + crosses

    end_transform = robot.end_link.transform
    open3d.visualization.draw_geometries(to_draw)


if __name__ == '__main__':
    main()
