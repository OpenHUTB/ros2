from pathlib import Path
from xml.etree import ElementTree


def test_mjcf_declares_elbow_antagonists_and_feedback_sensors() -> None:
    root = ElementTree.parse(
        Path(__file__).parents[1] / "models" / "elbow.xml"
    ).getroot()
    assert root.find(".//joint[@name='elbow_joint']") is not None
    assert root.find(".//fixed[@name='flexor_tendon']") is not None
    assert root.find(".//fixed[@name='extensor_tendon']") is not None
    assert root.find(".//muscle[@name='flexor_muscle']") is not None
    assert root.find(".//muscle[@name='extensor_muscle']") is not None
    assert root.find(".//actuatorfrc[@name='flexor_force']") is not None
    assert root.find(".//actuatorfrc[@name='extensor_force']") is not None


def test_mujoco_can_load_model_when_installed() -> None:
    try:
        import mujoco
    except ImportError:
        return
    path = Path(__file__).parents[1] / "models" / "elbow.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nq == 1
    assert model.nu == 2
