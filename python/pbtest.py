import numpy
import math
from pathlib import Path
from pybotics.robot import Robot
from lrmate.transforms import XyzWpr, Transform, Vector
from lrmate.robot import RobotWithMeshes, LinkWithMesh, Transform, Vector, Joint
import open3d


def _from_deg(deg):
    return deg * math.pi / 180.0


def main():
    expected = XyzWpr(465, 0, 365, -180, -90, 0)
    root_path = Path.cwd().parent
    robot = RobotWithMeshes(root_path.joinpath("urdf", "urdf", "LRMate-200iD.urdf"),
                            mesh_path=root_path.joinpath("meshes", "med-res"),
                            collision_mesh_path=root_path.joinpath("meshes", "convex"))

    to_draw = [link.mesh for link in robot.links.values()]
    for mesh in to_draw:
        mesh.scale(1000, center=[0, 0, 0])
        mesh.translate([0, 0, -330])

    cross = open3d.geometry.TriangleMesh.create_coordinate_frame(size=300)
    t = output(0, 0, 0, 0, 0, 30)
    cross.transform(t)
    to_draw.append(cross)

    # end_cross = open3d.geometry.TriangleMesh.create_coordinate_frame(size=300)
    # end_cross.transform(t)
    # to_draw.append(end_cross)

    open3d.visualization.draw_geometries(to_draw)


def output(*joints):
    params = numpy.array([[_from_deg(0), 0, _from_deg(0), 0],
                          [_from_deg(90), 50, _from_deg(90), 0],
                          [_from_deg(0), 330, _from_deg(0), 0],
                          [_from_deg(90), 35, _from_deg(0), 335],
                          [_from_deg(-90), 0, _from_deg(0), 0],
                          [_from_deg(90), 0, _from_deg(0), 80]][:len(joints)])
    joints = numpy.deg2rad(joints)
    robot = Robot.from_parameters(params)
    pose = robot.fk(joints)
    t = Transform(pose)
    print(XyzWpr.from_transform(t))
    # print(Vector(1, 0, 0) * t)
    print(t.matrix.round(3))
    return t.matrix


if __name__ == '__main__':
    main()
