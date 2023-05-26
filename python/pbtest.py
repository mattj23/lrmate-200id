import numpy
import math
from pybotics.robot import Robot
from lrmate.transforms import XyzWpr, Transform, Vector


def _from_deg(deg):
    return deg * math.pi / 180.0


def main():
    expected = XyzWpr(465, 0, 365, -180, -90, 0)

    # params = numpy.array(
    #     [
    #         [_from_deg(90), 50, _from_deg(0), 0],
    #         [_from_deg(0), 330, _from_deg(90), 0],
    #         [_from_deg(90), 35, _from_deg(0), 0],
    #         [_from_deg(-90), 0, _from_deg(0), 335],
    #         [_from_deg(90), 0, _from_deg(0), 0],
    #         [_from_deg(0), 0, _from_deg(0), 80],
    #     ]
    # )
    #

    # params = numpy.array(
    #     [
    #         [_from_deg(0), 0, _from_deg(90), 50, ],
    #         [_from_deg(90), 0, _from_deg(0), 330, ],
    #         [_from_deg(0), 0, _from_deg(90), 35, ],
    #         [_from_deg(0), 335, _from_deg(-90), 0, ],
    #         [_from_deg(0), 0, _from_deg(90), 0, ],
    #         [_from_deg(0), 80, _from_deg(0), 0, ],
    #     ]
    # )
    # params = numpy.array(
    #     [
    #         [_from_deg(90), 75, _from_deg(0), 330],
    #         [_from_deg(0), 300, _from_deg(90), 0],
    #         [_from_deg(90), 75, _from_deg(0), 0],
    #         [_from_deg(-90), 0, _from_deg(0), 320],
    #         [_from_deg(90), 0, _from_deg(0), 0],
    #         [_from_deg(0), 0, _from_deg(0), 140],
    #     ]
    # )
    #
    # params = numpy.array([[_from_deg(90), 50, _from_deg(0), 0],
    #                       [_from_deg(0), 330, _from_deg(90), 0],
    #                       [_from_deg(90), 35, _from_deg(0), 0],
    #                       [_from_deg(-90), 0, _from_deg(0), 335],
    #                       [_from_deg(90), 0, _from_deg(0), 0],
    #                       [_from_deg(0), 0, _from_deg(0), 80], ])
    # joints = numpy.deg2rad([0, 0, 0, 0, 0, 0])
    #
    # n = 1
    #
    # robot = Robot.from_parameters(params[:n])
    # joints = numpy.deg2rad(joints[:n])
    # pose = robot.fk(joints)
    # t = Transform(pose)
    # print(t.matrix)
    # # print(XyzWpr.from_transform(t))
    # params = numpy.array([[_from_deg(90), 50, _from_deg(0), 0],
    #                       [_from_deg(0), 330, _from_deg(90), 0],
    #                       [_from_deg(90), 35, _from_deg(0), 0],
    #                       [_from_deg(-90), 0, _from_deg(0), 335],
    #                       [_from_deg(90), 0, _from_deg(0), 0],
    #                       [_from_deg(0), 0, _from_deg(0), 80]][:len(joints)])

    # output(0, 0, 0, 0, 0, 0)
    output(0, 0)

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


if __name__ == '__main__':
    main()
