"""The models must expand and keep the frame and joint names the rest of the stack relies on.

Every test runs against both the full model and mock_apollo, so either can stand in for the other."""
import math
import os
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip('xacro')
yaml = pytest.importorskip('yaml')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEOMETRY = os.path.join(PKG, 'config', 'geometry.yaml')


MODELS = ['apollo_agv', 'mock_apollo']


@pytest.fixture(scope='module', params=MODELS)
def robot(request):
    path = os.path.join(PKG, 'urdf', f'{request.param}.urdf.xacro')
    doc = xacro.process_file(path, mappings={'geometry_file': GEOMETRY})
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def geometry():
    with open(GEOMETRY) as f:
        return yaml.safe_load(f)


def joints(robot):
    return {j.get('name'): j for j in robot.findall('joint')}


def test_moving_joints(robot):
    moving = {name: j.get('type') for name, j in joints(robot).items() if j.get('type') != 'fixed'}
    assert moving == {'left_wheel_joint': 'continuous', 'right_wheel_joint': 'continuous', 'ring_joint': 'continuous'}


def test_frames_exist(robot):
    links = {link.get('name') for link in robot.findall('link')}
    expected = {'base_footprint', 'base_link', 'ring_link', 'camera_link', 'gimbal_mount'}
    expected |= {f'ultrasonic_{side}_link' for side in ('front', 'rear', 'left', 'right')}
    assert expected <= links


def test_single_tree(robot):
    children = [j.find('child').get('link') for j in robot.findall('joint')]
    links = {link.get('name') for link in robot.findall('link')}
    assert len(children) == len(set(children)), 'a link has two parents'
    assert links - set(children) == {'base_footprint'}, 'exactly one root expected'


def test_wheels_match_geometry(robot, geometry):
    left = [float(v) for v in joints(robot)['left_wheel_joint'].find('origin').get('xyz').split()]
    right = [float(v) for v in joints(robot)['right_wheel_joint'].find('origin').get('xyz').split()]
    assert left[1] - right[1] == pytest.approx(geometry['wheel']['separation'])
    assert left[0] == right[0] == 0.0, 'drive axle must sit under base_footprint'
    # Wheels touch the floor: wheel centre is one radius above base_footprint.
    base_z = float(joints(robot)['base_joint'].find('origin').get('xyz').split()[2])
    assert base_z + left[2] == pytest.approx(geometry['wheel']['radius'])


def test_ring_turns_about_vertical(robot):
    assert joints(robot)['ring_joint'].find('axis').get('xyz').split() == ['0', '0', '1']


def test_ultrasonics_face_outwards(robot):
    yaw = {side: float(joints(robot)[f'ultrasonic_{side}_joint'].find('origin').get('rpy').split()[2])
           for side in ('front', 'rear', 'left', 'right')}
    assert yaw['front'] == 0.0
    assert abs(yaw['rear']) == pytest.approx(math.pi)
    assert yaw['left'] == pytest.approx(math.pi / 2)
    assert yaw['right'] == pytest.approx(-math.pi / 2)


def test_mock_apollo_has_an_imu_frame():
    doc = xacro.process_file(os.path.join(PKG, 'urdf', 'mock_apollo.urdf.xacro'), mappings={'geometry_file': GEOMETRY})
    robot = ET.fromstring(doc.toxml())
    assert robot.get('name') == 'mock_apollo'
    assert joints(robot)['imu_joint'].find('parent').get('link') == 'base_link'
