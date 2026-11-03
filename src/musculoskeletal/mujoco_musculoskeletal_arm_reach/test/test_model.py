from pathlib import Path
from xml.etree import ElementTree


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
