cat << 'EOF' > ~/geofence_ws/src/geofence_demo/test/test_geofence_logic.py
from geofence_demo.geofence_node import is_point_in_polygon

def test_normal_polygon():
    polygon = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]
    assert is_point_in_polygon(0.0, 0.0, polygon) is True
    assert is_point_in_polygon(2.0, 2.0, polygon) is False

def test_edge_cases():
    assert is_point_in_polygon(0.0, 0.0, []) is False
    assert is_point_in_polygon(0.0, 0.0, [(0.0, 0.0)]) is False
    assert is_point_in_polygon(0.0, 0.0, [(0.0, 0.0), (1.0, 1.0)]) is False

def test_point_on_boundary():
    polygon = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)]
    assert is_point_in_polygon(0.0, 1.0, polygon) is False
EOF
