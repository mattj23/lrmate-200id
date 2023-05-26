import json
from pathlib import Path
import numpy


def main():
    source_file = Path.cwd().parent / "test_cases" / "r30ib_forward_kinematics.json"
    with source_file.open("r") as handle:
        data = json.load(handle)

    all_data = []
    for case in data:
        joints = case["joints"]
        end = case["end"]
        t = numpy.matrix(end).reshape(4, 4).transpose()
        print(t)
        all_data.append((joints, t.tolist()))
    print(all_data)


if __name__ == '__main__':
    main()
