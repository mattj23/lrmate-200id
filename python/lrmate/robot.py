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


from open3d import geometry
import open3d

from urdf_parser_py import urdf


class Link:
    def __init__(self, name: str, mesh: geometry.TriangleMesh, **kwargs):
        self.name = name
        self.mesh = mesh
        self.collision_mesh: Optional[open3d.geometry.TriangleMesh] = kwargs.get("collision_mesh", None)
        self.cross = open3d.geometry.TriangleMesh.create_coordinate_frame(0.125)
        self.joints: List[Joint] = []
        self.current_transform: Optional[Transform] = None

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


class Joint:
    def __init__(self, name: str, parent: Link, child: Link, **kwargs):
        self.name = name
        self.parent = parent
        self.child = child
        self.base_transform: Transform = kwargs.get("transform", Transform.identity())
        self.axis = kwargs.get("axis")
        self.position = kwargs.get("position", 0)

    def get_transform(self) -> Transform:
        joint_transform = Transform.rotate_around_axis(self.position, self.axis)
        return self.base_transform * joint_transform


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
    def __init__(self, file_path: Union[str, Path], **kwargs):
        self.urdf: urdf.Robot = urdf.Robot.from_xml_file(str(file_path))
        self.origin: Transform = kwargs.get("origin", Transform.identity())
        self.mesh_path: Path = kwargs.get("mesh_path", None)
        self.collision_mesh_path: Path = kwargs.get("collision_mesh_path", None)

        folder, _ = os.path.split(file_path)
        package_root = _find_urdf_package_root(file_path)

        self.links = {}
        self.base_link: Optional[Link] = None
        self.end_link: Optional[Link] = None
        for link in self.urdf.links:
            visual: urdf.Visual = link.visual
            if isinstance(visual.geometry, urdf.Mesh):
                mesh = _mesh_from_package_filename(visual.geometry.filename, package_root, self.mesh_path)
                collision_mesh = _mesh_from_package_filename(link.collision.geometry.filename, package_root,
                                                             self.collision_mesh_path)

                self.links[link.name] = Link(link.name, mesh, collision_mesh=collision_mesh)
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
            self.links[joint.parent].joints.append(j)
            self.joints[j.name] = j

        starting_joints = {k: 0 for k in self.joints.keys()}
        self.set_joints(starting_joints)

    def set_joints(self, joint_bundle: Dict[str, float]):
        self.base_link.adjust(self.origin, joint_bundle)

    def set_joints_deg(self, joint_bundle: Dict[str, float]):
        self.set_joints({k: math.radians(v) for k, v in joint_bundle.items()})

    def link_meshes(self) -> List[open3d.geometry.TriangleMesh]:
        return [link.mesh for link in self.links.values()]


