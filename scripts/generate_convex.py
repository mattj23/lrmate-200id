"""
    This script generates the convex hull mesh for each stl file in meshes/low-res. The resulting stl is saved in the
    meshes/convex folder with the same name as the original.

    This script requires the Open3D library is installed on your interpreter.
"""

import open3d
from open3d.geometry import TriangleMesh
from pathlib import Path


def main():
    mesh_folder = Path.cwd().parent.joinpath("meshes")
    output_folder = mesh_folder.joinpath("convex")
    if not output_folder.exists():
        output_folder.mkdir()

    for item in mesh_folder.joinpath("low-res").glob("*.stl"):
        if not item.is_file():
            continue

        print(f"Generating convex hull for {item.name}")

        mesh: TriangleMesh = open3d.io.read_triangle_mesh(str(item))
        hull, _ = mesh.compute_convex_hull()
        hull: TriangleMesh
        hull.compute_vertex_normals()

        output_file = output_folder.joinpath(item.name)
        open3d.io.write_triangle_mesh(str(output_file), hull)


if __name__ == '__main__':
    main()
