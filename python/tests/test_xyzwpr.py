import pytest
import numpy
from lrmate.transforms import Transform, XyzWpr, Vector
from ._test_data import _transforms_and_xyzwpr


@pytest.mark.parametrize('matrix,xyzrpw', _transforms_and_xyzwpr)
def test_xyzwpr_to_transform(matrix, xyzrpw):
    transform = Transform(numpy.matrix(matrix))
    frame = XyzWpr(*xyzrpw[:3], xyzrpw[5], xyzrpw[4], xyzrpw[3])
    converted = frame.to_transform()
    assert numpy.allclose(converted.matrix, transform.matrix, rtol=1e-4, atol=1e-5)


@pytest.mark.parametrize('matrix,xyzrpw', _transforms_and_xyzwpr)
def test_transform_to_xyzwpr(matrix, xyzrpw):
    transform = Transform(numpy.matrix(matrix))
    frame = XyzWpr(*xyzrpw[:3], xyzrpw[5], xyzrpw[4], xyzrpw[3])
    converted = XyzWpr.from_transform(transform)
    assert frame.x == pytest.approx(converted.x, abs=1e-4)
    assert frame.y == pytest.approx(converted.y, abs=1e-4)
    assert frame.z == pytest.approx(converted.z, abs=1e-4)
    assert frame.w == pytest.approx(converted.w, abs=1e-3)
    assert frame.p == pytest.approx(converted.p, abs=1e-3)
    assert frame.r == pytest.approx(converted.r, abs=1e-3)
