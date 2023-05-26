from __future__ import annotations

import math

import numpy
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from typing import List, Optional, Dict, Union


@dataclass
class Vector:
    x: float
    y: float
    z: float

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: Union[float, Transform]) -> Vector:
        if isinstance(other, float):
            return Vector(self.x * other, self.y * other, self.z * other)
        elif isinstance(other, Transform):
            moved = other * numpy.array([self.x, self.y, self.z, 1])
            return Vector(moved[0], moved[1], moved[2])
        else:
            raise TypeError("Can only multiply by scalar")

    def __truediv__(self, other: float) -> Vector:
        return Vector(self.x / other, self.y / other, self.z / other)

    def norm(self):
        return numpy.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def unit(self):
        return self / self.norm()


class Transform:
    def __init__(self, matrix: numpy.ndarray):
        self.matrix = matrix

    def __mul__(self, other: Union[numpy.ndrray, Transform]) -> Transform:
        if isinstance(other, numpy.ndarray):
            return Transform(self.matrix @ other)
        elif isinstance(other, Transform):
            return Transform(self.matrix @ other.matrix)
        else:
            raise TypeError("Can only multiply by numpy array or Transform")

    @staticmethod
    def identity() -> Transform:
        return Transform(numpy.eye(4))

    @staticmethod
    def from_euler(order: str, angles: List[float]):
        matrix = numpy.identity(4)
        matrix[:3, :3] = Rotation.from_euler(order, angles).as_matrix()
        return Transform(matrix)

    @staticmethod
    def rotate_around_axis(theta, axis_vector):
        from math import cos, sin
        u = axis_vector.unit()
        m = [
            [cos(theta) + u.x ** 2 * (1 - cos(theta)), u.x * u.y * (1 - cos(theta)) - u.z * sin(theta),
             u.x * u.z * (1 - cos(theta)) + u.y * sin(theta), 0],
            [u.y * u.x * (1 - cos(theta)) + u.z * sin(theta), cos(theta) + u.y ** 2 * (1 - cos(theta)),
             u.y * u.z * (1 - cos(theta)) - u.x * sin(theta), 0],
            [u.z * u.x * (1 - cos(theta)) - u.y * sin(theta), u.z * u.y * (1 - cos(theta)) + u.x * sin(theta),
             cos(theta) + u.z ** 2 * (1 - cos(theta)), 0],
            [0, 0, 0, 1]
        ]
        return Transform(numpy.array(m))

    @staticmethod
    def translate(*args):
        if len(args) == 3:
            return Transform._translate(*args)
        elif len(args) == 1:
            arg = args[0]
            if hasattr(arg, "x") and hasattr(arg, "y") and hasattr(arg, "z"):
                return Transform.translate(arg.x, arg.y, arg.z)
        raise TypeError(f"Could not create translation from {args}")

    @staticmethod
    def _translate(x, y, z):
        m = numpy.identity(4)
        m[0, 3] = x
        m[1, 3] = y
        m[2, 3] = z
        return Transform(m)

    def invert(self) -> Transform:
        return Transform(numpy.linalg.inv(self.matrix))


@dataclass
class XyzWpr:
    """ Represents a transform in Fanuc's XYZWPR format, where X, Y, and Z are in mm and W, P, and R are in degrees. """
    x: float
    y: float
    z: float
    w: float
    p: float
    r: float

    def to_transform(self) -> Transform:
        yaw = Transform.rotate_around_axis(numpy.radians(self.w), Vector(0, 0, 1))
        pitch = Transform.rotate_around_axis(numpy.radians(self.p), Vector(0, 1, 0))
        roll = Transform.rotate_around_axis(numpy.radians(self.r), Vector(1, 0, 0))
        tilted = yaw * pitch * roll
        return Transform.translate(self.x, self.y, self.z) * tilted

    @staticmethod
    def from_transform(transform: Transform) -> XyzWpr:
        # Check that transform.matrix is a 4x4 matrix
        if transform.matrix.shape != (4, 4):
            raise ValueError(f"Expected a 4x4 matrix, got {transform.matrix.shape}")

        x = transform.matrix[0, 3]
        y = transform.matrix[1, 3]
        z = transform.matrix[2, 3]

        # Check that the rotation matrix is orthogonal
        if transform.matrix[2, 0] > (1.0 - 1e-6):
            p = -math.pi * 0.5
            r = 0
            w = math.atan2(-transform.matrix[0, 1], -transform.matrix[0, 2])
        elif transform.matrix[2, 0] < (-1.0 + 1e-6):
            p = math.pi * 0.5
            r = 0
            w = math.atan2(transform.matrix[1, 2], transform.matrix[1, 1])
        else:
            p = math.atan2(-transform.matrix[2, 0], math.sqrt(transform.matrix[0, 0] ** 2 + transform.matrix[1, 0] ** 2))
            w = math.atan2(transform.matrix[1, 0], transform.matrix[0, 0])
            r = math.atan2(transform.matrix[2, 1], transform.matrix[2, 2])

        return XyzWpr(x, y, z, _from_radians(w), _from_radians(p), _from_radians(r))


def _from_radians(radians: float) -> float:
    return radians * 180 / math.pi