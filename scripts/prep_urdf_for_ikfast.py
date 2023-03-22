"""
    This script simplifies the URDF file for IKFast by stripping out everything but the link names and the joints, and
    switches revolute joints to continuous as I've had some problems compiling with them.
"""

import sys
from pathlib import Path
from xml.etree.ElementTree import parse, ElementTree


def main():
    if len(sys.argv) < 2:
        print("No urdf file specified, searching for one in lrmate package")
        local_file = Path.cwd().parent.joinpath("urdf", "urdf", "LRMate-200iD.urdf")
        if local_file.exists():
            tree: ElementTree = parse(str(local_file))
        else:
            print("Could not find the local urdf, specify a file directly as an argument")
            return
    else:
        tree: ElementTree = parse(sys.argv[1])

    for element in tree.getroot():
        if element.tag == "link":
            children = list(element)
            for item in children:
                element.remove(item)
        elif element.tag == "joint":
            if element.get("type") == "revolute":
                element.set("type", "continuous")

                children = list(element)
                for item in children:
                    if item.tag == "limit":
                        element.remove(item)
        else:
            raise NotImplementedError()

    tree.write(sys.stdout, encoding="unicode")


if __name__ == '__main__':
    main()
