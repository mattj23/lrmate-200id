"""
    Generic robot visualizer and simple FK

    Requires the following libraries:
    * numpy (pip install numpy)
    * scipy (pip install scipy)
    * open3d (pip install open3d)
    * urdf_parser_py (pip install urdf-parser-py)

"""
from __future__ import annotations
import copy
import math
import os
from typing import List, Optional, Dict, Union
from pathlib import Path

import numpy

from .transforms import Transform, Vector

from open3d import geometry
import open3d

from urdf_parser_py import urdf


class Joint:
    def __init__(self, name: str, parent: Link, child: Link, **kwargs):
        self.name = name
        self.parent = parent
        self.child = child
        self.base_transform: Transform = kwargs.get("transform", Transform.identity())
        self.axis = kwargs.get("axis")
        self.position = kwargs.get("position", 0)

        if self not in self.parent.joints:
            self.parent.joints.append(self)

    def get_transform(self) -> Transform:
        joint_transform = Transform.rotate_around_axis(self.position, self.axis)
        return self.base_transform * joint_transform

    def internal_data(self) -> dict:
        return {
            "name": self.name,
            "parent": self.parent.name,
            "child": self.child.name,
            "transform": self.base_transform.matrix.tolist(),
            "axis": self.axis,
            "position": self.position
        }


class Link:
    def __init__(self, name: str, **kwargs):
        self.name = name
        self.joints: List[Joint] = []
        self.current_transform: Optional[Transform] = None

    def adjust(self, transform: Transform, joint_bundle: Dict[str, float]):
        self.current_transform = transform
        for joint in self.joints:
            if joint.name in joint_bundle:
                joint.position = joint_bundle[joint.name]
            stacked = self.current_transform * joint.get_transform()
            joint.child.adjust(stacked, joint_bundle)

    def internal_data(self) -> dict:
        return {
            "name": self.name,
            "joints": [joint.name for joint in self.joints]
        }


class LinkWithMesh(Link):
    def __init__(self, name: str, mesh: geometry.TriangleMesh, **kwargs):
        super().__init__(name, **kwargs)
        self.mesh = mesh
        self.collision_mesh: Optional[open3d.geometry.TriangleMesh] = kwargs.get("collision_mesh", None)
        self.cross = open3d.geometry.TriangleMesh.create_coordinate_frame(0.125)

    def get_untransformed_mesh(self) -> geometry.TriangleMesh:
        if self.current_transform is not None:
            inverse = self.current_transform.invert().matrix
            temp_mesh = copy.deepcopy(self.mesh)
            temp_mesh.transform(inverse)
            return temp_mesh
        return self.mesh

    def adjust(self, transform: Transform, joint_bundle: Dict[str, float]):
        if self.current_transform is not None:
            reverse = self.current_transform.invert().matrix
            self.mesh.transform(reverse)
            self.cross.transform(reverse)

        self.current_transform = transform
        self.mesh.transform(self.current_transform.matrix)
        self.cross.transform(self.current_transform.matrix)
        for joint in self.joints:
            if joint.name in joint_bundle:
                joint.position = joint_bundle[joint.name]
            stacked = self.current_transform * joint.get_transform()
            joint.child.adjust(stacked, joint_bundle)


def _find_urdf_package_root(file_path: str) -> Optional[str]:
    starting_folder, _ = os.path.split(file_path)

    working_path = os.path.abspath(starting_folder)
    while not os.path.exists(os.path.join(working_path, "package.xml")):
        parent_path = os.path.abspath(os.path.join(working_path, ".."))
        if parent_path == working_path:
            return None
        working_path = parent_path
    return working_path


def _mesh_from_package_filename(file_name: Union[str, Path],
                                package_root: Union[str, Path],
                                path_override: Optional[Path] = None) -> open3d.geometry.TriangleMesh:
    filename: str = str(file_name).replace("package://", "")
    path, mesh_file = os.path.split(filename)
    mesh_path = Path(os.path.join(package_root, *os.path.split(path)[1:], mesh_file))

    if path_override is not None:
        mesh_path = path_override / mesh_path.name
    mesh: geometry.TriangleMesh = open3d.io.read_triangle_mesh(str(mesh_path))
    mesh.compute_vertex_normals()
    return mesh


class Robot:
    def __init__(self, **kwargs):
        self.origin: Transform = Transform.identity()
        if "origin" in kwargs:
            self.origin = Transform(numpy.matrix(kwargs["origin"]))

        self.links = {}
        for link_name, link_data in kwargs.get("links", {}).items():
            self.links[link_name] = Link(link_name)

        self.joints = {}
        for joint_name, joint_data in kwargs.get("joints", {}).items():
            self.joints[joint_name] = Joint(joint_name,
                                            self.links[joint_data["parent"]],
                                            self.links[joint_data["child"]],
                                            transform=Transform(numpy.matrix(joint_data["transform"])),
                                            axis=joint_data["axis"],
                                            position=joint_data["position"])

        self.end_link: Optional[Link] = next((l for l in self.links.values() if not l.joints), None)
        all_children = []
        for link in self.links.values():
            all_children.extend(link.joints)

        self.base_link: Optional[Link] = next((l for l in self.links.values() if l not in all_children), None)

        starting_joints = {k: 0 for k in self.joints.keys()}
        self.set_joints(starting_joints)

    @staticmethod
    def default_lrmate():
        return Robot(**_default_lr_mate)

    def set_joints(self, joint_bundle: Dict[str, float]):
        self.base_link.adjust(self.origin, joint_bundle)

    def set_joints_deg(self, joint_bundle: Dict[str, float]):
        self.set_joints({k: math.radians(v) for k, v in joint_bundle.items()})

    def link_meshes(self) -> List[open3d.geometry.TriangleMesh]:
        return [link.mesh for link in self.links.values()]


class RobotWithMeshes:
    def __init__(self, file_path: Union[str, Path], **kwargs):
        self.urdf: urdf.Robot = urdf.Robot.from_xml_file(str(file_path))
        self.origin: Transform = kwargs.get("origin", Transform.identity())
        self.mesh_path: Path = kwargs.get("mesh_path", None)
        self.collision_mesh_path: Path = kwargs.get("collision_mesh_path", None)

        folder, _ = os.path.split(file_path)
        package_root = _find_urdf_package_root(file_path)

        self.links = {}
        self.base_link: Optional[LinkWithMesh] = None
        self.end_link: Optional[LinkWithMesh] = None
        for link in self.urdf.links:
            visual: urdf.Visual = link.visual
            if isinstance(visual.geometry, urdf.Mesh):
                mesh = _mesh_from_package_filename(visual.geometry.filename, package_root, self.mesh_path)
                collision_mesh = _mesh_from_package_filename(link.collision.geometry.filename, package_root,
                                                             self.collision_mesh_path)

                self.links[link.name] = LinkWithMesh(link.name, mesh, collision_mesh=collision_mesh)
                if link.name not in self.urdf.parent_map.keys():
                    self.base_link = self.links[link.name]
                if link.name not in self.urdf.child_map.keys():
                    self.end_link = self.links[link.name]
            else:
                raise Exception("Non mesh geometry")

        self.joints = {}
        for joint in self.urdf.joints:
            # we will assume these are all revolute joints
            origin: urdf.Pose = joint.origin

            r = Transform.from_euler("xyz", origin.rpy)
            t = Transform.translate(*origin.xyz)

            j = Joint(joint.name, self.links[joint.parent], self.links[joint.child],
                      transform=(t * r), axis=Vector(*joint.axis))
            # self.links[joint.parent].joints.append(j)
            self.joints[j.name] = j

        starting_joints = {k: 0 for k in self.joints.keys()}
        self.set_joints(starting_joints)

    def set_joints(self, joint_bundle: Dict[str, float]):
        self.base_link.adjust(self.origin, joint_bundle)

    def set_joints_deg(self, joint_bundle: Dict[str, float]):
        self.set_joints({k: math.radians(v) for k, v in joint_bundle.items()})

    def link_meshes(self) -> List[open3d.geometry.TriangleMesh]:
        return [link.mesh for link in self.links.values()]

    def internal_data(self) -> dict:
        return {
            "origin": self.origin.matrix.tolist(),
            "links": {k: v.internal_data() for k, v in self.links.items()},
            "joints": {k: v.internal_data() for k, v in self.joints.items()}
        }


_default_lr_mate = {
    'origin': [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    'links': {'Base': {'name': 'Base', 'joints': ['J1']},
              'J1': {'name': 'J1', 'joints': ['J2']},
              'J2': {'name': 'J2', 'joints': ['J3']},
              'J3': {'name': 'J3', 'joints': ['J4']},
              'J4': {'name': 'J4', 'joints': ['J5']},
              'J5': {'name': 'J5', 'joints': ['J6']},
              'J6': {'name': 'J6', 'joints': []}},
    'joints': {'J1': {'name': 'J1', 'parent': 'Base', 'child': 'J1',
                      'transform': [[1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.042741],
                                    [0.0, 0.0, 0.0, 1.0]],
                      'axis': Vector(x=0.0, y=0.0, z=1.0),
                      'position': 0},
               'J2': {'name': 'J2', 'parent': 'J1', 'child': 'J2',
                      'transform': [[1.0, 0.0, 0.0, 0.05],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.28726],
                                    [0.0, 0.0, 0.0, 1.0]],
                      'axis': Vector(x=0.0, y=-1.0, z=0.0),
                      'position': 0},
               'J3': {'name': 'J3', 'parent': 'J2', 'child': 'J3',
                      'transform': [[1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.33],
                                    [0.0, 0.0, 0.0, 1.0]],
                      'axis': Vector(x=0.0, y=-1.0, z=0.0),
                      'position': 0},
               'J4': {'name': 'J4', 'parent': 'J3', 'child': 'J4',
                      'transform': [[1.0, 0.0, 0.0, 0.088001],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.035027],
                                    [0.0, 0.0, 0.0, 1.0]],
                      'axis': Vector(x=-1.0, y=0.0, z=0.0),
                      'position': 0},
               'J5': {'name': 'J5', 'parent': 'J4', 'child': 'J5',
                      'transform': [[1.0, 0.0, 0.0, 0.2454],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0]],
                      'axis': Vector(x=0.0, y=-1.0, z=0.0),
                      'position': 0},
               'J6': {'name': 'J6', 'parent': 'J5', 'child': 'J6',
                      'transform': [
                          [0.9999999999999999, 0.0, 0.0, 0.05],
                          [0.0, -3.673205103305044e-06,
                           -0.9999999999932537, 0.0],
                          [0.0, 0.9999999999932537,
                           -3.673205103305044e-06, 0.0],
                          [0.0, 0.0, 0.0, 1.0]],
                      'axis': Vector(x=-1.0, y=0.0, z=0.0),
                      'position': 0}}}
