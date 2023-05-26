import pytest
import numpy
from lrmate.robot import Robot
from lrmate.joint_conversions import raw_to_fanuc, fanuc_to_raw
from lrmate.transforms import Transform
from ._test_data import _fanuc_joints_and_end_transforms


@pytest.mark.parametrize('joints,transform', _fanuc_joints_and_end_transforms)
def test_forward_kinematics(joints, transform):
    simple = Robot.default_lrmate()
    fanuc_joints = {f"J{i + 1}": v for i, v in enumerate(joints)}
    raw_joints = fanuc_to_raw(fanuc_joints)
    expected = (numpy.identity(4) * 0.001) * numpy.matrix(transform)
    calculated = simple.end_link.current_transform.matrix

    assert numpy.allclose(calculated, expected, rtol=1e-4, atol=1e-5)
