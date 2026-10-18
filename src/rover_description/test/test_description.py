import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_ROOT / "urdf" / "rover.urdf.xacro"
PARAMS_FILE = PACKAGE_ROOT / "config" / "model_parameters.yaml"


def rendered_robot(payload_mass="0.0"):
    completed = subprocess.run(
        [
            "xacro",
            str(XACRO_FILE),
            f"params_file:={PARAMS_FILE}",
            f"payload_mass_kg:={payload_mass}",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


def test_model_contains_front_casters_fixed_rears_and_three_axis_hitch():
    root = rendered_robot()
    joint_names = {joint.attrib["name"] for joint in root.findall("joint")}

    assert {
        "wagon_front_left_caster_swivel_joint",
        "wagon_front_right_caster_swivel_joint",
    }.issubset(joint_names)
    assert "wagon_rear_left_caster_swivel_joint" not in joint_names
    assert "wagon_rear_right_caster_swivel_joint" not in joint_names
    assert {
        "hitch_yaw_joint",
        "hitch_pitch_joint",
        "hitch_roll_joint",
    }.issubset(joint_names)


def test_payload_is_optional():
    empty_links = {link.attrib["name"] for link in rendered_robot().findall("link")}
    loaded_links = {
        link.attrib["name"] for link in rendered_robot("20.0").findall("link")
    }

    assert "payload_link" not in empty_links
    assert "payload_link" in loaded_links
