from pathlib import Path
from xml.etree import ElementTree

import pytest


MODEL = Path(__file__).parents[1] / 'models' / 'arm.xml'


def test_model_defines_two_joints_and_six_muscles():
    root = ElementTree.parse(MODEL).getroot()
    joints = root.findall(".//joint[@type='hinge']")
    muscles = root.findall('.//actuator/muscle')
    fixed_tendons = root.findall('.//tendon/fixed')
    assert [joint.attrib['name'] for joint in joints] == [
        'shoulder_joint',
        'elbow_joint',
    ]
    assert len(muscles) == 6
    assert len(fixed_tendons) == 6
    assert root.find(".//site[@name='end_effector']") is not None


def test_muscle_names_match_controller_contract():
    root = ElementTree.parse(MODEL).getroot()
    names = [
        muscle.attrib['name']
        for muscle in root.findall('.//actuator/muscle')
    ]
    assert names == [
        'shoulder_flexor',
        'shoulder_extensor',
        'elbow_flexor',
        'elbow_extensor',
        'biarticular_flexor',
        'biarticular_extensor',
    ]


def test_fixed_tendon_length_ranges_cover_full_joint_excursion():
    root = ElementTree.parse(MODEL).getroot()
    joint_ranges = {
        joint.attrib['name']: tuple(
            float(value) for value in joint.attrib['range'].split()
        )
        for joint in root.findall(".//joint[@type='hinge']")
    }
    fixed_tendons = {
        tendon.attrib['name']: tendon
        for tendon in root.findall('.//tendon/fixed')
    }
    muscles = root.findall('.//actuator/muscle')

    for muscle in muscles:
        tendon = fixed_tendons[muscle.attrib['tendon']]
        excursion = 0.0
        for joint in tendon.findall('joint'):
            lower, upper = joint_ranges[joint.attrib['joint']]
            coefficient = abs(float(joint.attrib['coef']))
            excursion += coefficient * max(abs(lower), abs(upper))
        lower, upper = (
            float(value)
            for value in muscle.attrib['lengthrange'].split()
        )
        assert lower == pytest.approx(-upper)
        assert excursion <= upper
